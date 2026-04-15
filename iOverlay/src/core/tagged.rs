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
    use crate::segm::segment::Segment;
    use i_float::int::point::IntPoint;

    /// Tags pushed on segments survive split/overlay and appear in output.
    #[test]
    fn tags_survive_overlay() {
        let mut overlay = Overlay::new_custom(16, IntOverlayOptions {
            preserve_output_collinear: true,
            ..Default::default()
        }, Default::default());
        // Two adjacent rectangles, tags 1 and 2.
        for (rect, tag) in [
            ([IntPoint::new(0,0), IntPoint::new(50,0), IntPoint::new(50,100), IntPoint::new(0,100)], 1u32),
            ([IntPoint::new(50,0), IntPoint::new(100,0), IntPoint::new(100,100), IntPoint::new(50,100)], 2u32),
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
        let (_shapes, tags) = graph.extract_shapes_with_tags(OverlayRule::Subject, &mut buf);
        assert_eq!(_shapes.len(), 1);
        let t = &tags[0][0];
        assert!(t.iter().any(|&v| v == 1), "tag 1 missing from output");
        assert!(t.iter().any(|&v| v == 2), "tag 2 missing from output");
    }
}
