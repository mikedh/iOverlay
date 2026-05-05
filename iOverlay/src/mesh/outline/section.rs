use crate::segm::boolean::ShapeCountBoolean;
use crate::segm::segment::Segment;
use i_float::float::compatible::FloatPointCompatible;
use i_float::int::point::IntPoint;

/// One offset "section" — a single straight chord of the input
/// polyline, plus its perpendicular offset endpoints.
///
/// **Analytical-tangent extension** (rmesh-fork): the original
/// upstream `OffsetSection` carried a single chord-direction `dir`
/// used both as the perpendicular axis for `a_top`/`b_top` and as
/// the corner-tangent supplied to `JoinBuilder`. That conflates two
/// distinct concepts: at a curved input edge (e.g., one chord of a
/// pre-discretised arc), the analytical tangent at the chord's
/// **start** differs from the analytical tangent at the chord's
/// **end** — they're equal only for a straight Line.
///
/// We split into `dir_a` (tangent at `a`) and `dir_b` (tangent at
/// `b`). When the caller does not supply analytical tangents the
/// chord-based default constructor sets `dir_a == dir_b == chord_dir`,
/// preserving upstream behavior bit-for-bit. When the caller does
/// supply tangents, `a_top` is computed from `dir_a` and `b_top`
/// from `dir_b`. At a CAD-tangent boundary between two adjacent
/// sections, both sides compute their offset endpoints from the
/// same tangent vector → bit-identical → `feed_join`'s
/// `s0.b_top != s1.a_top` check skips the join automatically.
#[derive(Debug, Clone)]
pub(super) struct OffsetSection<P: FloatPointCompatible> {
    pub(super) a: IntPoint,
    pub(super) b: IntPoint,
    pub(super) a_top: IntPoint,
    pub(super) b_top: IntPoint,
    /// Tangent direction at the `a` endpoint. Equal to the chord
    /// direction `(b - a).normalize()` for the chord-based default
    /// constructor; equal to the caller-supplied analytical tangent
    /// when the tangent-aware constructor is used.
    pub(super) dir_a: P,
    /// Tangent direction at the `b` endpoint. Same convention as
    /// `dir_a`. `JoinBuilder` impls consume `s0.dir_b` (incoming
    /// tangent at the corner) and `s1.dir_a` (outgoing tangent at
    /// the corner) when computing miter/round geometry.
    pub(super) dir_b: P,
}

impl<P: FloatPointCompatible> OffsetSection<P> {
    #[inline]
    pub(super) fn top_segment(&self) -> Option<Segment<ShapeCountBoolean>> {
        if self.a_top != self.b_top {
            Some(Segment::subject(self.a_top, self.b_top))
        } else {
            None
        }
    }

    #[inline]
    pub(super) fn a_segment(&self) -> Option<Segment<ShapeCountBoolean>> {
        if self.a_top != self.a {
            Some(Segment::subject(self.a, self.a_top))
        } else {
            None
        }
    }

    #[inline]
    pub(super) fn b_segment(&self) -> Option<Segment<ShapeCountBoolean>> {
        if self.b_top != self.b {
            Some(Segment::subject(self.b_top, self.b))
        } else {
            None
        }
    }
}
