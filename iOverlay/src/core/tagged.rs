//! Per-edge tag extraction for overlay results.
//!
//! Tags originate from `Segment.tag`, survive split/merge/sort, and
//! propagate through links during graph construction. Tags are emitted
//! inline during `extract_shapes_with_tags` — no post-hoc recovery needed.

#[cfg(test)]
mod tests {
    use crate::core::extract::BooleanExtractionBuffer;
    use crate::core::fill_rule::FillRule;
    use crate::core::overlay::{IntOverlayOptions, Overlay};
    use crate::core::overlay_rule::OverlayRule;
    use crate::segm::boolean::ShapeCountBoolean;
    use crate::segm::segment::{Segment, tag_pair_contains};
    use i_float::int::point::IntPoint;

    /// Tags pushed on segments survive split/overlay and appear in
    /// output. Two fully disjoint rectangles (no shared edges or
    /// sub-segments) — each output shape must carry the tag of the
    /// source rectangle its edges came from.
    #[test]
    fn tags_survive_overlay() {
        let mut overlay = Overlay::new_custom(16, IntOverlayOptions {
            preserve_output_collinear: true,
            ..Default::default()
        }, Default::default());
        for (rect, tag) in [
            ([IntPoint::new(0,0), IntPoint::new(50,0), IntPoint::new(50,100), IntPoint::new(0,100)], 1u16),
            ([IntPoint::new(200,0), IntPoint::new(300,0), IntPoint::new(300,100), IntPoint::new(200,100)], 2u16),
        ] {
            for i in 0..4 {
                let a = rect[i];
                let b = rect[(i+1)%4];
                if a != b {
                    overlay.segments.push(Segment::create_and_validate_tagged(a, b, ShapeCountBoolean::SUBJ_DIRECT, tag));
                }
            }
        }
        let mut buf = BooleanExtractionBuffer::default();
        let graph = overlay.build_graph_view(FillRule::EvenOdd).unwrap();
        let (shapes, tags) = graph.extract_shapes_with_tags(OverlayRule::Subject, &mut buf);
        assert_eq!(shapes.len(), 2, "two disjoint rectangles must give two shapes");
        assert_eq!(tags.len(), 2);
        // Each shape's outer contour carries exactly the tag of its
        // source rectangle. Edges report a `TagPair`; probe either slot.
        let shape_has_tag = |shape_idx: usize, needle: u16| -> bool {
            tags[shape_idx].iter().any(|c| c.iter().any(|&pair| tag_pair_contains(pair, needle)))
        };
        let found_1 = shape_has_tag(0, 1) || shape_has_tag(1, 1);
        let found_2 = shape_has_tag(0, 2) || shape_has_tag(1, 2);
        assert!(found_1, "tag 1 missing from output");
        assert!(found_2, "tag 2 missing from output");
    }
}
