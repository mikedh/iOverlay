use crate::core::extract::VisitState;
use crate::core::overlay_rule::OverlayRule;
use crate::geom::id_point::IdPoint;
use crate::segm::segment::SegmentFill;
use alloc::vec::Vec;

#[derive(Debug, Clone, Copy)]
pub struct OverlayLink {
    pub a: IdPoint,
    pub b: IdPoint,
    pub fill: SegmentFill,
    /// User-supplied edge tag, propagated from the input `Segment`.
    pub tag: u16,
}

impl OverlayLink {
    #[inline(always)]
    #[allow(dead_code)]
    pub(crate) fn new(a: IdPoint, b: IdPoint, fill: SegmentFill) -> OverlayLink {
        OverlayLink { a, b, fill, tag: 0 }
    }

    #[inline(always)]
    pub(crate) fn new_tagged(a: IdPoint, b: IdPoint, fill: SegmentFill, tag: u16) -> OverlayLink {
        OverlayLink { a, b, fill, tag }
    }

    #[inline(always)]
    pub(crate) fn other(&self, node_id: usize) -> IdPoint {
        if self.a.id == node_id { self.b } else { self.a }
    }

    #[inline(always)]
    pub(crate) fn is_direct(&self) -> bool {
        self.a.point < self.b.point
    }
}

pub(crate) trait OverlayLinkFilter {
    fn filter_by_overlay_into(&self, overlay_rule: OverlayRule, buffer: &mut Vec<VisitState>);
}
