use crate::geom::x_segment::XSegment;
use crate::segm::winding::WindingCount;
use core::cmp::Ordering;
use i_float::int::point::IntPoint;

pub type SegmentFill = u8;

pub const NONE: SegmentFill = 0;

pub const SUBJ_TOP: SegmentFill = 0b0001;
pub const SUBJ_BOTTOM: SegmentFill = 0b0010;
pub const CLIP_TOP: SegmentFill = 0b0100;
pub const CLIP_BOTTOM: SegmentFill = 0b1000;

pub const SUBJ_BOTH: SegmentFill = SUBJ_TOP | SUBJ_BOTTOM;
pub const CLIP_BOTH: SegmentFill = CLIP_TOP | CLIP_BOTTOM;
pub const BOTH_TOP: SegmentFill = SUBJ_TOP | CLIP_TOP;
pub const BOTH_BOTTOM: SegmentFill = SUBJ_BOTTOM | CLIP_BOTTOM;

pub const ALL: SegmentFill = SUBJ_BOTH | CLIP_BOTH;

/// Per-segment source-tag slot. Holds up to two distinct non-zero
/// tags — `0` is the untagged sentinel; both slots zero means the
/// segment carries no tag. When the boolean engine merges two
/// segments whose `x_segment` coincides, tags from both inputs are
/// unioned into the pair so downstream consumers can pick whichever
/// matches their ring-local context (see merge semantics below).
///
/// A third distinct tag overflows silently (first-wins). In practice
/// the merge points are pairwise coincidences; three distinct tagged
/// sources producing the *same* `x_segment` with a surviving winding
/// count is geometrically pathological.
///
/// Width: `u32`. Lifts the 65k segment ceiling on tagged callers
/// (rmesh's `Path2D::offset` / boolean ops). The tag never participates
/// in the integer-grid arithmetic — it's pure provenance metadata that
/// survives the split / merge passes — so widening it is a no-op for
/// the boolean engine itself.
pub type TagPair = [u32; 2];

/// The untagged sentinel pair.
pub const UNTAGGED: TagPair = [0, 0];

/// Build a pair holding a single tag (or `UNTAGGED` if `t == 0`).
#[inline(always)]
pub const fn tag_pair(t: u32) -> TagPair {
    [t, 0]
}

/// `true` iff `t` (non-zero) appears in either slot of `pair`.
#[inline]
pub fn tag_pair_contains(pair: TagPair, t: u32) -> bool {
    t != 0 && (pair[0] == t || pair[1] == t)
}

/// Union `src` into `dst`. Already-present tags are skipped; new tags
/// fill the first empty slot. A third distinct tag is dropped — the
/// existing two win (first-wins on overflow).
#[inline]
pub fn tag_pair_union(dst: &mut TagPair, src: TagPair) {
    for &t in &src {
        if t == 0 || tag_pair_contains(*dst, t) {
            continue;
        }
        if dst[0] == 0 {
            dst[0] = t;
        } else if dst[1] == 0 {
            dst[1] = t;
        }
        // else: third tag silently dropped — see `TagPair` docs.
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Segment<C> {
    pub(crate) x_segment: XSegment,
    pub(crate) count: C,
    /// Per-edge source-tag pair. See [`TagPair`].
    pub tag: TagPair,
}

impl<C: WindingCount> Segment<C> {
    /// Backward-compatible untagged constructor. Equivalent to
    /// [`Self::create_and_validate_tagged`] with `tag = 0` — the
    /// untagged sentinel. Lets every upstream (non-fork) call site
    /// stay byte-identical to upstream `iOverlay`, shrinking the fork
    /// diff on `merge.rs`, `build.rs`, `predicate.rs`, `subject.rs`,
    /// and `string/overlay.rs`.
    #[inline(always)]
    pub fn create_and_validate(a: IntPoint, b: IntPoint, count: C) -> Self {
        Self::with_tag_pair(a, b, count, UNTAGGED)
    }

    /// Tagged constructor taking a single `u32` — the common case for
    /// external callers (rmesh's `discretize`, `boolean`, `project`).
    #[inline(always)]
    pub fn create_and_validate_tagged(a: IntPoint, b: IntPoint, count: C, tag: u32) -> Self {
        Self::with_tag_pair(a, b, count, tag_pair(tag))
    }

    /// Internal constructor taking a full `TagPair`. Used by the split
    /// solver to propagate the parent segment's pair onto its children
    /// unchanged (including any second tag inherited from a prior merge).
    #[inline(always)]
    pub(crate) fn with_tag_pair(a: IntPoint, b: IntPoint, count: C, tag: TagPair) -> Self {
        if a < b {
            Self {
                x_segment: XSegment { a, b },
                count,
                tag,
            }
        } else {
            Self {
                x_segment: XSegment { a: b, b: a },
                count: count.invert(),
                tag,
            }
        }
    }
}

impl<C> PartialEq<Self> for Segment<C> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.x_segment == other.x_segment
    }
}

impl<C> Eq for Segment<C> {}

impl<C> PartialOrd for Segment<C> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<C> Ord for Segment<C> {
    #[inline(always)]
    fn cmp(&self, other: &Self) -> Ordering {
        self.x_segment.cmp(&other.x_segment)
    }
}
