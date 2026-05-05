use crate::mesh::math::Math;
use crate::mesh::outline::builder_join::JoinBuilder;
use crate::mesh::outline::builder_join::{BevelJoinBuilder, MiterJoinBuilder, RoundJoinBuilder};
use crate::mesh::outline::section::OffsetSection;
use crate::mesh::outline::uniq_iter::{UniqueSegment, UniqueSegmentsIter};
use crate::mesh::style::LineJoin;
use crate::segm::boolean::ShapeCountBoolean;
use crate::segm::segment::{Segment, tag_pair};

/// Reserved tag value identifying a join/fillet edge emitted by the
/// outline builder. User tags must lie strictly below this sentinel
/// (`0..JOIN_TAG_SENTINEL`), so callers reading per-edge tags off the
/// outline output can distinguish "this came from input edge N" from
/// "this is a constructed corner fillet". The rmesh fork mirrors this
/// constant in `path/buffer/discretize.rs::JOIN_TAG`.
pub const JOIN_TAG_SENTINEL: u32 = u32::MAX;
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::marker::PhantomData;
use i_float::adapter::FloatPointAdapter;
use i_float::float::compatible::FloatPointCompatible;
use i_float::float::number::FloatNumber;
use i_float::float::vector::FloatPointMath;

trait OutlineBuild<P: FloatPointCompatible> {
    fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    );

    fn build_and_tag(
        &self,
        path: &[P],
        tags: &[u32],
        tangent_start: &[P],
        tangent_end: &[P],
        adapter: &FloatPointAdapter<P>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    );

    fn capacity(&self, points_count: usize) -> usize;
    fn additional_offset(&self, radius: P::Scalar) -> P::Scalar;
}

pub struct OutlineBuilder<P: FloatPointCompatible> {
    builder: Box<dyn OutlineBuild<P>>,
}

struct Builder<J: JoinBuilder<P>, P: FloatPointCompatible> {
    extend: bool,
    radius: P::Scalar,
    join_builder: J,
    _phantom: PhantomData<P>,
}

impl<P: FloatPointCompatible + 'static> OutlineBuilder<P> {
    pub fn new(radius: P::Scalar, join: &LineJoin<P::Scalar>) -> OutlineBuilder<P> {
        let extend = radius > P::Scalar::from_float(0.0);
        let builder: Box<dyn OutlineBuild<P>> = {
            match join {
                LineJoin::Miter(ratio) => Box::new(Builder {
                    extend,
                    radius,
                    join_builder: MiterJoinBuilder::new(*ratio, radius),
                    _phantom: Default::default(),
                }),
                LineJoin::Round(ratio) => Box::new(Builder {
                    extend,
                    radius,
                    join_builder: RoundJoinBuilder::new(*ratio, radius),
                    _phantom: Default::default(),
                }),
                LineJoin::Bevel => Box::new(Builder {
                    extend,
                    radius,
                    join_builder: BevelJoinBuilder {},
                    _phantom: Default::default(),
                }),
            }
        };

        Self { builder }
    }

    #[inline]
    pub fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        self.builder.build(path, adapter, segments);
    }

    /// Like `build`, but tags each segment from a parallel `tags`
    /// array AND takes per-edge analytical tangent vectors at each
    /// endpoint.
    ///
    /// All four parallel arrays must have length `path.len()`.
    /// `tags[i]` tags edge `i` (from `path[i]` to `path[(i+1) % n]`).
    /// `tangent_start[i]` is the unit tangent at `path[i]` for the
    /// curve heading into edge `i`; `tangent_end[i]` is the unit
    /// tangent at `path[(i+1) % n]` for the curve at edge `i`'s end.
    /// For straight-line input segments, both tangents equal the
    /// line direction; for tagged-arc input the analytical tangent
    /// is the arc derivative.
    ///
    /// **Dedup-share contract.** At a CAD-tangent boundary between
    /// adjacent edges — where the caller passes the **same** tangent
    /// vector on both sides of the shared vertex — the offset
    /// endpoints round-trip to bit-identical ints and `feed_join`
    /// emits nothing, suppressing the spurious round-join that
    /// chord-based discretization would otherwise insert. The
    /// equality must be **bit-identical floats**, not just
    /// analytically equal — compute the tangent once per shared
    /// vertex and use the same value on both sides.
    #[inline]
    pub fn build_and_tag(
        &self,
        path: &[P],
        tags: &[u32],
        tangent_start: &[P],
        tangent_end: &[P],
        adapter: &FloatPointAdapter<P>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        self.builder.build_and_tag(
            path,
            tags,
            tangent_start,
            tangent_end,
            adapter,
            segments,
        );
    }

    #[inline]
    pub fn capacity(&self, points_count: usize) -> usize {
        self.builder.capacity(points_count)
    }

    #[inline]
    pub fn additional_offset(&self, radius: P::Scalar) -> P::Scalar {
        self.builder.additional_offset(radius)
    }
}

impl<J: JoinBuilder<P>, P: FloatPointCompatible> OutlineBuild<P> for Builder<J, P> {
    #[inline]
    fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        if path.len() < 2 {
            return;
        }

        self.build(path, adapter, segments);
    }

    fn build_and_tag(
        &self,
        path: &[P],
        tags: &[u32],
        tangent_start: &[P],
        tangent_end: &[P],
        adapter: &FloatPointAdapter<P>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        if path.len() < 2
            || tags.len() != path.len()
            || tangent_start.len() != path.len()
            || tangent_end.len() != path.len()
        {
            return;
        }
        self.build_and_tag(
            path,
            tags,
            tangent_start,
            tangent_end,
            adapter,
            segments,
        );
    }

    #[inline]
    fn capacity(&self, points_count: usize) -> usize {
        self.join_builder.capacity() * points_count
    }

    #[inline]
    fn additional_offset(&self, radius: P::Scalar) -> P::Scalar {
        self.join_builder.additional_offset(radius)
    }
}

impl<J: JoinBuilder<P>, P: FloatPointCompatible> Builder<J, P> {
    #[inline]
    fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        self.build_impl(path, None, None, adapter, segments);
    }

    /// Tagged + tangent-aware build. See
    /// [`OutlineBuilder::build_and_tag`] for the dedup-share contract.
    #[inline]
    fn build_and_tag(
        &self,
        path: &[P],
        tags: &[u32],
        tangent_start: &[P],
        tangent_end: &[P],
        adapter: &FloatPointAdapter<P>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        debug_assert_eq!(path.len(), tags.len());
        debug_assert_eq!(path.len(), tangent_start.len());
        debug_assert_eq!(path.len(), tangent_end.len());
        self.build_impl(
            path,
            Some(tags),
            Some((tangent_start, tangent_end)),
            adapter,
            segments,
        );
    }

    /// Shared body for the two public entry points. Both args are
    /// optional. When `tangents` is `None`, the per-section
    /// perpendicular falls back to chord direction (`(b - a) /
    /// |b - a|`) — same numbers the upstream chord-only builder
    /// would have produced. When `tags` is `None`, `apply_tag` is a
    /// no-op so an `#[inline]` monomorphization collapses back to
    /// the untagged hot loop.
    #[inline]
    fn build_impl(
        &self,
        path: &[P],
        tags: Option<&[u32]>,
        tangents: Option<(&[P], &[P])>,
        adapter: &FloatPointAdapter<P>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        let n = path.len();
        if n < 2 {
            return;
        }

        // Each UniqueSegment carries `edge_index` (first edge of the
        // run, used for `tangent_start` lookup) and `last_edge_index`
        // (last edge of the run, used for `tangent_end` lookup at
        // the run's `b` endpoint when collinear merging spans
        // multiple input edges).
        let iter = path
            .iter()
            .enumerate()
            .map(|(i, p)| (adapter.float_to_int(p), i));
        let mut uniq = match UniqueSegmentsIter::new(iter, n) {
            Some(u) => u,
            None => return,
        };
        let us0 = match uniq.next() {
            Some(u) => u,
            None => return,
        };

        let edge_tag = |edge_index: usize| -> u32 { tags.map_or(0, |t| t[edge_index % n]) };
        let has_tags = tags.is_some();
        let apply_tag = |segments: &mut Vec<Segment<ShapeCountBoolean>>, mark: usize, tag: u32| {
            if has_tags {
                let pair = tag_pair(tag);
                for seg in &mut segments[mark..] {
                    seg.tag = pair;
                }
            }
        };
        // Hot-path dispatch: tangent-aware sections cost two
        // perpendiculars (one per endpoint), chord-only sections
        // cost one (dir_a == dir_b). Keeping a separate chord
        // constructor avoids paying for two `ortho_and_scale` calls
        // on every section in the union/boolean pipeline.
        let make_section = |us: &UniqueSegment| -> OffsetSection<P> {
            match tangents {
                Some((ts, te)) => OffsetSection::with_tangents(
                    self.radius,
                    us,
                    ts[us.edge_index],
                    te[us.last_edge_index],
                    adapter,
                ),
                None => OffsetSection::from_chord(self.radius, us, adapter),
            }
        };

        let s0 = make_section(&us0);
        let mut sk = s0.clone();
        let mut prev_tag = edge_tag(us0.edge_index);

        let mark = segments.len();
        segments.push_some(sk.top_segment());
        apply_tag(segments, mark, prev_tag);

        for usi in uniq {
            let si = make_section(&usi);
            let cur_tag = edge_tag(usi.edge_index);

            let mark = segments.len();
            segments.push_some(si.top_segment());
            apply_tag(segments, mark, cur_tag);

            // Join: same tag → edge tag, different → JOIN sentinel.
            let jt = if prev_tag == cur_tag {
                prev_tag
            } else {
                JOIN_TAG_SENTINEL
            };
            let mark = segments.len();
            self.feed_join(&sk, &si, adapter, segments);
            apply_tag(segments, mark, jt);

            sk = si;
            prev_tag = cur_tag;
        }

        let first_tag = edge_tag(us0.edge_index);
        let jt = if prev_tag == first_tag {
            prev_tag
        } else {
            JOIN_TAG_SENTINEL
        };
        let mark = segments.len();
        self.feed_join(&sk, &s0, adapter, segments);
        apply_tag(segments, mark, jt);
    }

    #[inline]
    fn feed_join(
        &self,
        s0: &OffsetSection<P>,
        s1: &OffsetSection<P>,
        adapter: &FloatPointAdapter<P>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        // Tangent-boundary fast path: when the caller's analytical
        // tangents at the shared vertex are bit-identical (the
        // `with_tangents` contract), `s0.b_top` and `s1.a_top`
        // round-trip identically through `float_to_int`. The
        // mathematical offset has no corner here — emit nothing.
        // This covers BOTH outer-tangent (no fillet needed) and
        // inner-tangent (no clip-extend segments needed) in one
        // guard; the upstream code only handled the outer case
        // via `if s0.b_top != s1.a_top` inside the outer branch
        // and would push two duplicate segments in the inner branch
        // at a tangent concave transition.
        if s0.b_top == s1.a_top {
            return;
        }

        let vi = s1.b - s1.a;
        let vp = s0.b - s0.a;

        let cross = vi.cross_product(vp);
        debug_assert!(cross != 0, "not possible! UniqueSegmentsIter guarantee it");
        let outer_corner = (cross > 0) == self.extend;
        if outer_corner {
            self.join_builder.add_join(s0, s1, adapter, segments);
        } else {
            // inner (concave) corner — extend the offset bottoms to
            // clip at the input vertex
            segments.push_some(s0.b_segment());
            segments.push_some(s1.a_segment());
        }
    }
}

impl<P: FloatPointCompatible> OffsetSection<P> {
    /// Chord-only constructor: both endpoints' tangents are the
    /// chord direction, so the perpendicular is computed once and
    /// reused. Hot path for chord-only `build` (union, boolean,
    /// upstream consumers).
    #[inline]
    fn from_chord(
        radius: P::Scalar,
        s: &UniqueSegment,
        adapter: &FloatPointAdapter<P>,
    ) -> Self {
        let a = adapter.int_to_float(&s.a);
        let b = adapter.int_to_float(&s.b);
        let dir = FloatPointMath::normalize(&FloatPointMath::sub(&b, &a));
        let t = Math::ortho_and_scale(&dir, radius);
        let a_top = adapter.float_to_int(&FloatPointMath::add(&a, &t));
        let b_top = adapter.float_to_int(&FloatPointMath::add(&b, &t));
        Self {
            a: s.a,
            b: s.b,
            a_top,
            b_top,
            dir_a: dir,
            dir_b: dir,
        }
    }

    /// Tangent-aware constructor: `dir_a` / `dir_b` are the
    /// caller-supplied unit tangents at each endpoint. **Caller
    /// MUST pass unit-length vectors** — `ortho_and_scale` does not
    /// normalize, so a non-unit input would silently scale the
    /// offset. The `debug_assert!`s catch this in dev builds; the
    /// release path skips the sqrt+div.
    ///
    /// **Dedup-share contract.** At a boundary between two sections
    /// that the caller intends to be tangent, the previous section's
    /// `dir_b` and the next section's `dir_a` must be bit-identical
    /// floats. Caller computes the tangent once per shared vertex
    /// and passes the same value to both sides; offset endpoints
    /// round-trip to identical ints and `feed_join`'s
    /// `s0.b_top == s1.a_top` guard emits nothing — no spurious
    /// JOIN_TAG run.
    #[inline]
    fn with_tangents(
        radius: P::Scalar,
        s: &UniqueSegment,
        dir_a: P,
        dir_b: P,
        adapter: &FloatPointAdapter<P>,
    ) -> Self {
        debug_assert!(
            unit_norm_check(&dir_a),
            "with_tangents: dir_a is not unit length",
        );
        debug_assert!(
            unit_norm_check(&dir_b),
            "with_tangents: dir_b is not unit length",
        );
        let a = adapter.int_to_float(&s.a);
        let b = adapter.int_to_float(&s.b);
        let ta = Math::ortho_and_scale(&dir_a, radius);
        let tb = Math::ortho_and_scale(&dir_b, radius);
        let a_top = adapter.float_to_int(&FloatPointMath::add(&a, &ta));
        let b_top = adapter.float_to_int(&FloatPointMath::add(&b, &tb));
        Self {
            a: s.a,
            b: s.b,
            a_top,
            b_top,
            dir_a,
            dir_b,
        }
    }
}

/// Debug-only: confirm `v` is unit length within a generous slack
/// (~1e-9 on `f32`-ish scalars). Production path skips entirely.
#[inline]
fn unit_norm_check<P: FloatPointCompatible>(v: &P) -> bool {
    let dot = FloatPointMath::dot_product(v, v);
    let one = P::Scalar::from_float(1.0);
    let slack = P::Scalar::from_float(1e-6);
    let diff = if dot >= one { dot - one } else { one - dot };
    diff <= slack
}

trait VecPushSome<T> {
    fn push_some(&mut self, value: Option<T>);
}

impl<T> VecPushSome<T> for Vec<T> {
    #[inline]
    fn push_some(&mut self, value: Option<T>) {
        if let Some(v) = value {
            self.push(v);
        }
    }
}
