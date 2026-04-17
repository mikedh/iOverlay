use crate::segm::segment::{Segment, tag_pair_union};
use crate::segm::winding::WindingCount;
use alloc::vec::Vec;

pub(crate) trait ShapeSegmentsMerge {
    fn merge_if_needed(&mut self) -> bool;
}

impl<C: WindingCount> ShapeSegmentsMerge for Vec<Segment<C>> {
    fn merge_if_needed(&mut self) -> bool {
        if self.len() < 2 {
            return false;
        }

        let mut prev = &self[0].x_segment;
        for i in 1..self.len() {
            let this = &self[i].x_segment;
            if prev.eq(this) {
                let new_len = merge(self, i);
                self.truncate(new_len);
                return true;
            }
            prev = this;
        }

        false
    }
}

/// Collapse adjacent segments that share an `x_segment` into a single
/// survivor. Windings combine via `count.apply`; if the combined
/// winding cancels to zero the segment is dropped. Source tags are
/// unioned into a `TagPair` so both (up to two) provenance tags
/// survive — downstream consumers pick whichever matches their
/// ring-local context, avoiding the speckle that a first-wins
/// tiebreak would otherwise introduce on coincident offset boundaries.
fn merge<C: WindingCount>(segments: &mut [Segment<C>], after: usize) -> usize {
    let mut i = after;
    let mut j = i - 1;
    let mut prev = segments[j];

    while i < segments.len() {
        if prev.x_segment.eq(&segments[i].x_segment) {
            prev.count.apply(segments[i].count);
            tag_pair_union(&mut prev.tag, segments[i].tag);
        } else {
            if prev.count.is_not_empty() {
                segments[j] = prev;
                j += 1;
            }
            prev = segments[i];
        }
        i += 1;
    }

    if prev.count.is_not_empty() {
        segments[j] = prev;
        j += 1;
    }

    j
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::segm::boolean::ShapeCountBoolean;
    use crate::segm::segment::{UNTAGGED, tag_pair, tag_pair_contains};
    use alloc::vec;
    use i_float::int::point::IntPoint;

    #[test]
    fn empty_and_single() {
        let mut empty: Vec<Segment<ShapeCountBoolean>> = Vec::new();
        empty.merge_if_needed();
        assert!(empty.is_empty());

        let seg = Segment::create_and_validate(
            IntPoint::new(1, 2),
            IntPoint::new(3, 4),
            ShapeCountBoolean::new(1, 1),
        );
        let mut single = vec![seg];
        single.merge_if_needed();
        assert_eq!(single, vec![seg]);
    }

    #[test]
    fn distinct_x_segments_are_untouched() {
        let s1 = Segment::create_and_validate(
            IntPoint::new(1, 1),
            IntPoint::new(2, 2),
            ShapeCountBoolean::new(1, 1),
        );
        let s2 = Segment::create_and_validate(
            IntPoint::new(3, 3),
            IntPoint::new(4, 4),
            ShapeCountBoolean::new(2, 2),
        );
        let s3 = Segment::create_and_validate(
            IntPoint::new(5, 5),
            IntPoint::new(6, 6),
            ShapeCountBoolean::new(3, 3),
        );
        let mut segments = vec![s1, s2, s3];
        segments.merge_if_needed();
        assert_eq!(segments, vec![s1, s2, s3]);
    }

    #[test]
    fn coincident_segments_combine_winding() {
        let a = IntPoint::new(1, 2);
        let b = IntPoint::new(3, 4);

        // Two same-direction segments sum to `(1, 1)`.
        let s1 = Segment::create_and_validate(a, b, ShapeCountBoolean::new(1, 0));
        let s2 = Segment::create_and_validate(a, b, ShapeCountBoolean::new(0, 1));
        let mut segments = vec![s1, s2];
        segments.merge_if_needed();
        assert_eq!(segments.len(), 1);
        assert_eq!(segments[0].count, ShapeCountBoolean::new(1, 1));

        // Three same-direction segments sum to `(3, 3)`.
        let s1 = Segment::create_and_validate(a, b, ShapeCountBoolean::new(1, 0));
        let s2 = Segment::create_and_validate(a, b, ShapeCountBoolean::new(0, 1));
        let s3 = Segment::create_and_validate(a, b, ShapeCountBoolean::new(2, 2));
        let mut segments = vec![s1, s2, s3];
        segments.merge_if_needed();
        assert_eq!(segments.len(), 1);
        assert_eq!(segments[0].count, ShapeCountBoolean::new(3, 3));

        // Inverted-order coincident segments are canonicalised by the
        // constructor before merge sees them.
        let reversed = Segment::create_and_validate(b, a, ShapeCountBoolean::new(1, 0));
        let forward = Segment::create_and_validate(a, b, ShapeCountBoolean::new(0, 1));
        let mut segments = vec![reversed, forward];
        segments.merge_if_needed();
        assert_eq!(segments.len(), 1);
        // `reversed` inverts its count; forward stays positive → (1, 1) - (1, 0) = (0, 1)? Actually the
        // invert takes `(1, 0)` → `(-1, 0)`, and summed with `(0, 1)` gives `(-1, 1)`. That's still non-empty.
        assert!(segments[0].count.is_not_empty());
    }

    #[test]
    fn tags_union_on_conflict() {
        let a = IntPoint::new(0, 0);
        let b = IntPoint::new(10, 10);
        let count = ShapeCountBoolean::new(1, 0);

        // Distinct non-zero tags meet: both survive in the pair.
        let s1 = Segment::create_and_validate_tagged(a, b, count, 3);
        let s2 = Segment::create_and_validate_tagged(a, b, count, 5);
        let mut segments = vec![s1, s2];
        segments.merge_if_needed();
        assert_eq!(segments.len(), 1);
        assert!(tag_pair_contains(segments[0].tag, 3));
        assert!(tag_pair_contains(segments[0].tag, 5));

        // Equal tags collapse to a single-slot pair.
        let s1 = Segment::create_and_validate_tagged(a, b, count, 7);
        let s2 = Segment::create_and_validate_tagged(a, b, count, 7);
        let mut segments = vec![s1, s2];
        segments.merge_if_needed();
        assert_eq!(segments[0].tag, tag_pair(7));

        // Untagged + tagged: tag wins; untagged slot stays empty.
        let s1 = Segment::create_and_validate(a, b, count);
        let s2 = Segment::create_and_validate_tagged(a, b, count, 9);
        let mut segments = vec![s1, s2];
        segments.merge_if_needed();
        assert_eq!(segments[0].tag, tag_pair(9));

        // Cancelling merge drops the segment regardless of tags.
        let s1 = Segment::create_and_validate_tagged(a, b, ShapeCountBoolean::new(1, 0), 3);
        let s2 = Segment::create_and_validate_tagged(a, b, ShapeCountBoolean::new(-1, 0), 5);
        let mut segments = vec![s1, s2];
        segments.merge_if_needed();
        assert!(segments.is_empty());
    }

    #[test]
    fn third_tag_overflows_first_wins() {
        let a = IntPoint::new(0, 0);
        let b = IntPoint::new(10, 10);
        let count = ShapeCountBoolean::new(1, 0);

        // Three distinct tags: pair fills with the first two; the third is dropped.
        let mut segments = vec![
            Segment::create_and_validate_tagged(a, b, count, 3),
            Segment::create_and_validate_tagged(a, b, count, 5),
            Segment::create_and_validate_tagged(a, b, count, 7),
        ];
        segments.merge_if_needed();
        assert_eq!(segments.len(), 1);
        assert!(tag_pair_contains(segments[0].tag, 3));
        assert!(tag_pair_contains(segments[0].tag, 5));
        assert!(!tag_pair_contains(segments[0].tag, 7));
        assert_ne!(segments[0].tag, UNTAGGED);
    }
}
