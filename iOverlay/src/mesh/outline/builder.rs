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
pub const JOIN_TAG_SENTINEL: u16 = u16::MAX;
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::marker::PhantomData;
use i_float::adapter::FloatPointAdapter;
use i_float::float::compatible::FloatPointCompatible;
use i_float::float::number::FloatNumber;
use i_float::float::vector::FloatPointMath;

trait OutlineBuild<P: FloatPointCompatible<T>, T: FloatNumber> {
    fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    );

    fn build_and_tag(
        &self,
        path: &[P],
        tags: &[u16],
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    );

    fn capacity(&self, points_count: usize) -> usize;
    fn additional_offset(&self, radius: T) -> T;
}

pub struct OutlineBuilder<P: FloatPointCompatible<T>, T: FloatNumber> {
    builder: Box<dyn OutlineBuild<P, T>>,
}

struct Builder<J: JoinBuilder<P, T>, P: FloatPointCompatible<T>, T: FloatNumber> {
    extend: bool,
    radius: T,
    join_builder: J,
    _phantom: PhantomData<P>,
}

impl<P: FloatPointCompatible<T> + 'static, T: FloatNumber + 'static> OutlineBuilder<P, T> {
    pub fn new(radius: T, join: &LineJoin<T>) -> OutlineBuilder<P, T> {
        let extend = radius > T::from_float(0.0);
        let builder: Box<dyn OutlineBuild<P, T>> = {
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
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        self.builder.build(path, adapter, segments);
    }

    /// Like `build`, but tags each segment from a parallel `tags` array.
    #[inline]
    pub fn build_and_tag(
        &self,
        path: &[P],
        tags: &[u16],
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        self.builder.build_and_tag(path, tags, adapter, segments);
    }

    #[inline]
    pub fn capacity(&self, points_count: usize) -> usize {
        self.builder.capacity(points_count)
    }

    #[inline]
    pub fn additional_offset(&self, radius: T) -> T {
        self.builder.additional_offset(radius)
    }
}

impl<J: JoinBuilder<P, T>, P: FloatPointCompatible<T>, T: FloatNumber> OutlineBuild<P, T>
    for Builder<J, P, T>
{
    #[inline]
    fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P, T>,
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
        tags: &[u16],
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        if path.len() < 2 || tags.len() != path.len() {
            return;
        }
        self.build_and_tag(path, tags, adapter, segments);
    }

    #[inline]
    fn capacity(&self, points_count: usize) -> usize {
        self.join_builder.capacity() * points_count
    }

    #[inline]
    fn additional_offset(&self, radius: T) -> T {
        self.join_builder.additional_offset(radius)
    }
}

impl<J: JoinBuilder<P, T>, P: FloatPointCompatible<T>, T: FloatNumber> Builder<J, P, T> {
    #[inline]
    fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        self.build_impl(path, None, adapter, segments);
    }

    /// Like `build`, but sets `segment.tag` from the parallel `tags`
    /// array. `tags.len()` must equal `path.len()`. Edge `i` gets
    /// `tags[i]`; join segments between different-tag edges get the
    /// `JOIN_TAG_SENTINEL`.
    #[inline]
    fn build_and_tag(
        &self,
        path: &[P],
        tags: &[u16],
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        debug_assert_eq!(path.len(), tags.len());
        self.build_impl(path, Some(tags), adapter, segments);
    }

    /// Shared body for `build` and `build_and_tag`. The two call sites
    /// differ only in whether they apply per-edge tags; this function
    /// runs the offset/join pipeline once and short-circuits the tag
    /// writes when `tags` is `None`. `#[inline]` lets the monomorphized
    /// untagged path collapse back to a straight offset pipeline —
    /// no branches on the hot loop.
    #[inline]
    fn build_impl(
        &self,
        path: &[P],
        tags: Option<&[u16]>,
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        let n = path.len();
        if n < 2 {
            return;
        }

        // Each UniqueSegment carries edge_index — the index of the
        // input edge it came from. Tag lookup is tags[edge_index],
        // a direct array access: no map, no collisions.
        let iter = path.iter().enumerate().map(|(i, p)| (adapter.float_to_int(p), i));
        let mut uniq = match UniqueSegmentsIter::new(iter) {
            Some(u) => u,
            None => return,
        };
        let us0 = match uniq.next() {
            Some(u) => u,
            None => return,
        };

        let edge_tag = |edge_index: usize| -> u16 {
            tags.map_or(0, |t| t[edge_index % n])
        };
        // `apply_tag` owns the &mut Vec through its argument, so the
        // closure itself only captures the boolean Option::is_some.
        let has_tags = tags.is_some();
        let apply_tag =
            |segments: &mut Vec<Segment<ShapeCountBoolean>>, mark: usize, tag: u16| {
                if has_tags {
                    let pair = tag_pair(tag);
                    for seg in &mut segments[mark..] {
                        seg.tag = pair;
                    }
                }
            };

        let s0 = OffsetSection::new(self.radius, &us0, adapter);
        let mut sk = s0.clone();
        let mut prev_tag = edge_tag(us0.edge_index);

        let mark = segments.len();
        segments.push_some(sk.top_segment());
        apply_tag(segments, mark, prev_tag);

        for usi in uniq {
            let si = OffsetSection::new(self.radius, &usi, adapter);
            let cur_tag = edge_tag(usi.edge_index);

            let mark = segments.len();
            segments.push_some(si.top_segment());
            apply_tag(segments, mark, cur_tag);

            // Join: same tag → edge tag, different → JOIN sentinel.
            let jt = if prev_tag == cur_tag { prev_tag } else { JOIN_TAG_SENTINEL };
            let mark = segments.len();
            self.feed_join(&sk, &si, adapter, segments);
            apply_tag(segments, mark, jt);

            sk = si;
            prev_tag = cur_tag;
        }

        let first_tag = edge_tag(us0.edge_index);
        let jt = if prev_tag == first_tag { prev_tag } else { JOIN_TAG_SENTINEL };
        let mark = segments.len();
        self.feed_join(&sk, &s0, adapter, segments);
        apply_tag(segments, mark, jt);
    }

    #[inline]
    fn feed_join(
        &self,
        s0: &OffsetSection<P, T>,
        s1: &OffsetSection<P, T>,
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        let vi = s1.b - s1.a;
        let vp = s0.b - s0.a;

        let cross = vi.cross_product(vp);
        debug_assert!(cross != 0, "not possible! UniqueSegmentsIter guarantee it");
        let outer_corner = (cross > 0) == self.extend;
        if outer_corner {
            if s0.b_top != s1.a_top {
                self.join_builder.add_join(s0, s1, adapter, segments);
            }
        } else {
            // no join
            segments.push_some(s0.b_segment());
            segments.push_some(s1.a_segment());
        }
    }
}

impl<P: FloatPointCompatible<T>, T: FloatNumber> OffsetSection<P, T> {
    #[inline]
    fn new(radius: T, s: &UniqueSegment, adapter: &FloatPointAdapter<P, T>) -> Self
    where
        P: FloatPointCompatible<T>,
        T: FloatNumber,
    {
        let a = adapter.int_to_float(&s.a);
        let b = adapter.int_to_float(&s.b);
        let ab = FloatPointMath::sub(&b, &a);
        let dir = FloatPointMath::normalize(&ab);
        let t = Math::ortho_and_scale(&dir, radius);

        let at = FloatPointMath::add(&a, &t);
        let bt = FloatPointMath::add(&b, &t);
        let a_top = adapter.float_to_int(&at);
        let b_top = adapter.float_to_int(&bt);

        Self {
            a: s.a,
            b: s.b,
            a_top,
            b_top,
            dir,
            _phantom: Default::default(),
        }
    }
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
