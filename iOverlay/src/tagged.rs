//! # Tagged edges for iOverlay
//!
//! All of the "carry a `u32` tag through a boolean / offset run"
//! machinery lives in this one file so a maintainer reviewing the
//! feature can see it at a glance. The design is documented in
//! `upstream.md` at the crate root; this file is the implementation.
//!
//! The core idea: iOverlay's split solver only cuts segments at
//! intersection points, and its merge step only combines
//! already-collinear segments. Both preserve the infinite line each
//! segment lies on. That means, when `preserve_output_collinear`
//! is set, every output edge is collinear with some pre-overlay
//! input edge — so a `HashMap<LineIdentity, u32>` populated
//! *before* the overlay is sufficient to recover per-edge tags
//! *after* extraction. No tag field rides through the split/merge
//! core; no boolean-core type gets generalized.
//!
//! What lives here:
//!
//!   * [`LineKey`] — exact, direction-invariant integer identity of
//!     an infinite line.
//!   * [`TagLookup`] — type alias for `BTreeMap<LineKey, u32>`.
//!     Callers insert entries with standard map syntax
//!     (`lookup.entry(LineKey::from_points(a, b)).or_insert(tag)`)
//!     and recover per-edge tags via [`annotate_shapes`] /
//!     [`annotate_contour`].
//!   * [`SYNTHESIZED_ROUND_JOIN_TAG`] — sentinel for chords emitted
//!     by `RoundJoinBuilder` at sharp convex corners.
//!   * [`build_outline_tagged`] — the per-edge walk used by
//!     `OutlineBuilder::build_tagged_into`. Lives here, not in
//!     `mesh/outline/builder.rs`, so the feature is one file.

use crate::core::overlay::Overlay;
use crate::mesh::outline::builder::Builder;
use crate::mesh::outline::builder_join::JoinBuilder;
use crate::mesh::outline::section::OffsetSection;
use crate::mesh::outline::uniq_iter::UniqueSegment;
use crate::segm::boolean::ShapeCountBoolean;
use crate::segm::segment::Segment;
use alloc::collections::BTreeMap;
use alloc::vec::Vec;
use i_float::adapter::FloatPointAdapter;
use i_float::float::compatible::FloatPointCompatible;
use i_float::float::number::FloatNumber;
use i_float::int::point::IntPoint;
use i_shape::int::shape::IntShapes;

// --------------------------------------------------------------------
// Public API
// --------------------------------------------------------------------

/// Tag stamped on segments synthesized by `RoundJoinBuilder` at sharp
/// convex corners. User-supplied tags must avoid colliding with this.
pub const SYNTHESIZED_ROUND_JOIN_TAG: u32 = u32::MAX;

/// Exact, direction-less identity of the infinite line through two
/// distinct `IntPoint`s. Two collinear segments — regardless of
/// orientation, length, or position along the line — collapse to the
/// same `LineKey`.
///
/// Construction is fully integer: reduce the direction vector by
/// `gcd`, canonicalize its sign, and pair it with the
/// translation-invariant constant `ax*dy - ay*dx`. Since `IntPoint`
/// holds `i32`s, the `i64` intermediates below cannot overflow.
#[derive(Hash, PartialEq, Eq, Clone, Copy, Debug, PartialOrd, Ord)]
pub struct LineKey {
    dx: i64,
    dy: i64,
    c: i64,
}

impl LineKey {
    /// Panics in debug if `a == b`; callers must skip degenerate edges.
    #[inline]
    pub fn from_points(a: IntPoint, b: IntPoint) -> Self {
        debug_assert!(a != b, "LineKey undefined for degenerate segment");

        let ax = a.x as i64;
        let ay = a.y as i64;
        let bx = b.x as i64;
        let by = b.y as i64;

        let mut dx = bx - ax;
        let mut dy = by - ay;

        let g = gcd_u64(dx.unsigned_abs(), dy.unsigned_abs()) as i64;
        if g > 1 {
            dx /= g;
            dy /= g;
        }

        // Canonical sign: prefer dx > 0; on the vertical line, dy > 0.
        if dx < 0 || (dx == 0 && dy < 0) {
            dx = -dx;
            dy = -dy;
        }

        let c = ax * dy - ay * dx;

        Self { dx, dy, c }
    }
}

#[inline]
fn gcd_u64(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }
    a
}

/// Map from infinite-line identity to user tag. Populated by
/// [`build_outline_tagged`] for the offset path, or hand-populated
/// by arbitrary tagged boolean-op callers via standard `BTreeMap`
/// entry syntax:
///
/// ```ignore
/// if a != b {
///     lookup.entry(LineKey::from_points(a, b)).or_insert(tag);
/// }
/// ```
///
/// Recover per-edge tags with [`annotate_shapes`] /
/// [`annotate_contour`]. `or_insert` gives first-wins semantics;
/// callers must skip tag-0 and degenerate edges themselves.
pub type TagLookup = BTreeMap<LineKey, u32>;

/// Per-edge tag lookup for one closed contour. Edge `i` goes from
/// `contour[i]` to `contour[(i + 1) % n]`. Missing entries and
/// degenerate edges resolve to tag `0`.
pub fn annotate_contour(lookup: &TagLookup, contour: &[IntPoint]) -> Vec<u32> {
    let n = contour.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let a = contour[i];
        let b = contour[(i + 1) % n];
        let tag = if a == b {
            0
        } else {
            lookup.get(&LineKey::from_points(a, b)).copied().unwrap_or(0)
        };
        out.push(tag);
    }
    out
}

/// Annotate every contour of every shape. Shape / contour / edge
/// structure of the result mirrors `shapes`.
pub fn annotate_shapes(lookup: &TagLookup, shapes: &IntShapes) -> Vec<Vec<Vec<u32>>> {
    shapes
        .iter()
        .map(|shape| {
            shape
                .iter()
                .map(|contour| annotate_contour(lookup, contour))
                .collect()
        })
        .collect()
}

// --------------------------------------------------------------------
// Outline walking (used by `OutlineBuilder::build_tagged_into`)
// --------------------------------------------------------------------

/// Tagged counterpart of the untagged `Builder::build` walk. Called
/// once per `OutlineBuilder::build_tagged_into` invocation via the
/// `OutlineBuild` dyn trait.
///
/// Unlike the untagged path, we do NOT run inputs through
/// `UniqueSegmentsIter`. That iterator folds collinear consecutive
/// edges into one, which would desync the parallel `tags` indexing.
/// Instead we walk `path` / `tags` directly, skip degenerate edges
/// (`a == b`), and use [`feed_join_tagged`] to bridge corners.
/// [`feed_join_tagged`] silently tolerates `cross == 0` (the
/// collinear continuation case), which upstream's `feed_join`
/// debug-asserts against because `UniqueSegmentsIter` never hands it
/// such a pair.
pub(crate) fn build_outline_tagged<J, P, T>(
    builder: &Builder<J, P, T>,
    path: &[P],
    tags: &[u32],
    adapter: &FloatPointAdapter<P, T>,
    overlay: &mut Overlay,
    lookup: &mut TagLookup,
) where
    J: JoinBuilder<P, T>,
    P: FloatPointCompatible<T>,
    T: FloatNumber,
{
    let n = path.len();
    debug_assert_eq!(n, tags.len(), "tags.len() must equal path.len()");
    if n < 2 || tags.len() != n {
        return;
    }

    let segments = &mut overlay.segments;
    segments.reserve(builder.join_builder.capacity() * n);

    let mut first: Option<OffsetSection<P, T>> = None;
    let mut prev: Option<OffsetSection<P, T>> = None;
    let mut prev_tag: u32 = 0;
    let mut n_emitted: usize = 0;

    for i in 0..n {
        let a_int = adapter.float_to_int(&path[i]);
        let b_int = adapter.float_to_int(&path[(i + 1) % n]);
        if a_int == b_int {
            continue;
        }

        let sec = OffsetSection::new(
            builder.radius,
            &UniqueSegment { a: a_int, b: b_int },
            adapter,
        );
        let cur_tag = tags[i];

        if let Some(ts) = sec.top_segment() {
            if cur_tag != 0 && sec.a_top != sec.b_top {
                lookup
                    .entry(LineKey::from_points(sec.a_top, sec.b_top))
                    .or_insert(cur_tag);
            }
            segments.push(ts);
        }

        if let Some(ref p) = prev {
            feed_join_tagged(builder, p, &sec, prev_tag, adapter, segments, lookup);
        } else {
            first = Some(sec.clone());
        }

        prev = Some(sec);
        prev_tag = cur_tag;
        n_emitted += 1;
    }

    if n_emitted >= 2
        && let (Some(p), Some(f)) = (prev, first)
    {
        feed_join_tagged(builder, &p, &f, prev_tag, adapter, segments, lookup);
    }
}

/// Tagged counterpart of `Builder::feed_join`. Silently returns on
/// `cross == 0` (collinear continuation), and stamps every emitted
/// connector segment with `connector_tag` — except a genuine
/// round-arc, which stamps [`SYNTHESIZED_ROUND_JOIN_TAG`]. The round-
/// vs-bevel discrimination is delegated to
/// `JoinBuilder::emits_round_arc`, which defaults to `false` and is
/// overridden only on `RoundJoinBuilder`.
#[inline]
fn feed_join_tagged<J, P, T>(
    builder: &Builder<J, P, T>,
    s0: &OffsetSection<P, T>,
    s1: &OffsetSection<P, T>,
    connector_tag: u32,
    adapter: &FloatPointAdapter<P, T>,
    segments: &mut Vec<Segment<ShapeCountBoolean>>,
    lookup: &mut TagLookup,
) where
    J: JoinBuilder<P, T>,
    P: FloatPointCompatible<T>,
    T: FloatNumber,
{
    let vi = s1.b - s1.a;
    let vp = s0.b - s0.a;

    let cross = vi.cross_product(vp);
    if cross == 0 {
        return;
    }

    let tag = if builder.join_builder.emits_round_arc(s0, s1) {
        SYNTHESIZED_ROUND_JOIN_TAG
    } else {
        connector_tag
    };

    let start = segments.len();
    let outer_corner = (cross > 0) == builder.extend;
    if outer_corner {
        if s0.b_top != s1.a_top {
            builder.join_builder.add_join(s0, s1, adapter, segments);
        }
    } else {
        if let Some(seg) = s0.b_segment() {
            segments.push(seg);
        }
        if let Some(seg) = s1.a_segment() {
            segments.push(seg);
        }
    }
    if tag != 0 {
        for seg in &segments[start..] {
            let a = seg.x_segment.a;
            let b = seg.x_segment.b;
            if a != b {
                lookup.entry(LineKey::from_points(a, b)).or_insert(tag);
            }
        }
    }
}

// --------------------------------------------------------------------
// Tests
// --------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    fn p(x: i32, y: i32) -> IntPoint {
        IntPoint::new(x, y)
    }

    #[test]
    fn line_key_direction_invariant() {
        let k0 = LineKey::from_points(p(0, 0), p(10, 0));
        let k1 = LineKey::from_points(p(10, 0), p(0, 0));
        assert_eq!(k0, k1);
    }

    #[test]
    fn line_key_fragment_invariant() {
        let k_full = LineKey::from_points(p(0, 0), p(10, 0));
        let k_l = LineKey::from_points(p(0, 0), p(3, 0));
        let k_m = LineKey::from_points(p(3, 0), p(7, 0));
        let k_r = LineKey::from_points(p(7, 0), p(10, 0));
        assert_eq!(k_full, k_l);
        assert_eq!(k_full, k_m);
        assert_eq!(k_full, k_r);
    }

    #[test]
    fn line_key_distinct_parallel() {
        let k0 = LineKey::from_points(p(0, 0), p(10, 0));
        let k1 = LineKey::from_points(p(0, 1), p(10, 1));
        assert_ne!(k0, k1);
    }

    #[test]
    fn line_key_distinct_non_collinear() {
        let k0 = LineKey::from_points(p(0, 0), p(10, 0));
        let k1 = LineKey::from_points(p(0, 0), p(0, 10));
        assert_ne!(k0, k1);
    }

    #[test]
    fn line_key_reduced_direction() {
        let k0 = LineKey::from_points(p(0, 0), p(2, 4));
        let k1 = LineKey::from_points(p(0, 0), p(1, 2));
        assert_eq!(k0.dx, k1.dx);
        assert_eq!(k0.dy, k1.dy);
    }

    fn set(map: &mut TagLookup, a: IntPoint, b: IntPoint, tag: u32) {
        if tag != 0 && a != b {
            map.entry(LineKey::from_points(a, b)).or_insert(tag);
        }
    }

    #[test]
    fn tag_lookup_first_non_zero_wins() {
        let mut map = TagLookup::new();
        set(&mut map, p(0, 0), p(10, 0), 0); // filtered at call site
        set(&mut map, p(0, 0), p(5, 0), 42); // first non-zero wins
        set(&mut map, p(3, 0), p(7, 0), 99); // same line, preserved
        assert_eq!(
            map.get(&LineKey::from_points(p(0, 0), p(10, 0))).copied(),
            Some(42)
        );
    }

    #[test]
    fn annotate_contour_basic() {
        let mut map = TagLookup::new();
        set(&mut map, p(0, 0), p(10, 0), 1);
        set(&mut map, p(10, 0), p(10, 10), 2);
        set(&mut map, p(10, 10), p(0, 10), 3);
        set(&mut map, p(0, 10), p(0, 0), 4);

        let contour = vec![p(0, 0), p(10, 0), p(10, 10), p(0, 10)];
        let tags = annotate_contour(&map, &contour);
        assert_eq!(tags, vec![1, 2, 3, 4]);
    }
}
