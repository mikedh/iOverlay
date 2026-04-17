use crate::core::extract::VisitState;
use crate::core::overlay_rule::OverlayRule;
use crate::geom::id_point::IdPoint;
use crate::segm::segment::{SegmentFill, TagPair};
use alloc::vec::Vec;

#[derive(Debug, Clone, Copy)]
pub(crate) struct OverlayLink {
    pub(crate) a: IdPoint,
    pub(crate) b: IdPoint,
    pub(crate) fill: SegmentFill,
    /// Per-edge source-tag pair propagated from the input `Segment`.
    pub(crate) tag: TagPair,
}

impl OverlayLink {
    #[inline(always)]
    pub(crate) fn new_tagged(
        a: IdPoint,
        b: IdPoint,
        fill: SegmentFill,
        tag: TagPair,
    ) -> OverlayLink {
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
