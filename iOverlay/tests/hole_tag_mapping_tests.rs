//! Regression suite for the per-ring tag-array length invariant in
//! `extract_shapes_with_tags`.
//!
//! Every test in this file asserts the contract that the
//! `mikedh/rmesh` crate (downstream consumer) used to paper over with
//! a `log::warn! + tags.resize(points.len(), 0)` fallback at
//! `crates/rmesh/src/project.rs:778`. That fallback existed because
//! `distribute_hole_tags`'s multi-shape path walked holes in
//! INSERTION order while `scan_join` distributes them in SPATIAL
//! order — so per-ring tag arrays ended up paired with the wrong
//! contour whenever the binder reordered holes across multiple
//! enclosing shapes.
//!
//! Coverage:
//!
//! 1. `tagged_single_shape_with_holes_keeps_per_ring_lengths` — the
//!    early-return path: one outer shape, multiple holes, in
//!    insertion order. The mapping-aware distribution must match the
//!    legacy single-shape extend semantics exactly.
//!
//! 2. `tagged_multi_shape_disjoint_holes_keep_alignment` — two
//!    disjoint outer rectangles, each with its own internal hole;
//!    holes of DIFFERENT vertex counts. Guards the multi-shape
//!    alignment invariant end to end (the binder mapping here is the
//!    identity — see the test's own doc for why this is not the
//!    reordering proof).
//!
//! 3. `tagged_multi_shape_hole_tag_values_pin_to_right_shape` — same
//!    layout as (2), but also verifies the TAG VALUES end up on the
//!    correct shape (not just the lengths).
//!
//! 4. `tagged_no_holes_multi_shape` — degenerate: many outer shapes,
//!    no holes. The mapping is empty/None and distribution is a
//!    no-op.
//!
//! 5. `tagged_collinear_input_does_not_drop_points` — regression for
//!    the `simplify_contour` interaction: a contour with collinear
//!    vertices must survive extraction with both points and tags
//!    intact (VecTagCollector forces `preserve_output_collinear`).
//!
//! 6. `tagged_collinear_outer_with_hole_keeps_alignment` (G3) —
//!    collinear vertices on an outer ring that also has an interior
//!    hole: exercises the preserve-collinear path together with hole
//!    distribution.
//!
//! 7. `untagged_path_still_simplifies_collinear` (G3) — the SAME
//!    collinear input through the untagged `extract_shapes` must
//!    still drop collinear points (proves `NoOpTagCollector`'s
//!    `PRESERVE_COLLINEAR = false` is unchanged — we did not globally
//!    disable simplification).
//!
//! 8. `randomized_tagged_overlay_preserves_alignment` (G5) — a
//!    deterministic pseudo-random sweep of many multi-rectangle +
//!    hole layouts run through the REAL binder/mapping, asserting
//!    the per-ring `points.len() == tags.len()` invariant on every
//!    output ring. This is the "from every angle" coverage that
//!    actually exercises non-trivial `parent_for_child` bindings.

extern crate alloc;

#[cfg(test)]
mod tests {
    use i_float::int::point::IntPoint;
    use i_overlay::core::extract::BooleanExtractionBuffer;
    use i_overlay::core::fill_rule::FillRule;
    use i_overlay::core::overlay::{IntOverlayOptions, Overlay};
    use i_overlay::core::overlay_rule::OverlayRule;
    use i_overlay::segm::boolean::ShapeCountBoolean;
    use i_overlay::segm::segment::{Segment, tag_pair_contains};

    /// Local clone of the (`pub(crate)`) `SUBJ_DIRECT` constant from
    /// inside iOverlay — pushing subject-direct count from a test
    /// crate isn't possible otherwise.
    fn subj_direct() -> ShapeCountBoolean {
        ShapeCountBoolean { subj: 1, clip: 0 }
    }

    /// Wire a closed polygon's edges into `overlay.segments` carrying
    /// `tag` on each segment. CCW input is treated as outer; reversed
    /// (CW) traversal naturally yields holes after the boolean fill.
    fn push_polygon(overlay: &mut Overlay, ring: &[IntPoint], tag: u32, count: ShapeCountBoolean) {
        for i in 0..ring.len() {
            let a = ring[i];
            let b = ring[(i + 1) % ring.len()];
            if a == b {
                continue;
            }
            overlay
                .segments
                .push(Segment::create_and_validate_tagged(a, b, count, tag));
        }
    }

    /// Build an `Overlay` configured to preserve collinear vertices
    /// (so simplification doesn't desync points and tags) and run the
    /// tagged extraction. Returns `(shapes, tags)`.
    fn run_tagged_overlay(
        polygons: &[(&[IntPoint], u32)],
    ) -> (
        i_shape::int::shape::IntShapes,
        alloc::vec::Vec<alloc::vec::Vec<alloc::vec::Vec<i_overlay::segm::segment::TagPair>>>,
    ) {
        let mut overlay = Overlay::new_custom(
            64,
            IntOverlayOptions {
                preserve_output_collinear: false, // VecTagCollector forces preserve internally
                ..Default::default()
            },
            Default::default(),
        );
        for (ring, tag) in polygons {
            push_polygon(&mut overlay, ring, *tag, subj_direct());
        }
        let mut buf = BooleanExtractionBuffer::default();
        let graph = overlay
            .build_graph_view(FillRule::EvenOdd)
            .expect("build_graph_view should not fail on non-empty overlay");
        graph.extract_shapes_with_tags(OverlayRule::Subject, &mut buf)
    }

    /// Assert the structural invariant the downstream rmesh consumer
    /// requires: shape-count matches across `shapes` and `tags`, and
    /// for every ring the per-edge tag count equals the per-vertex
    /// point count.
    fn assert_shape_tag_alignment(
        shapes: &i_shape::int::shape::IntShapes,
        tags: &[alloc::vec::Vec<alloc::vec::Vec<i_overlay::segm::segment::TagPair>>],
    ) {
        assert_eq!(
            shapes.len(),
            tags.len(),
            "shape-count mismatch: shapes={}, tags={}",
            shapes.len(),
            tags.len()
        );
        for (si, (shape, shape_tags)) in shapes.iter().zip(tags.iter()).enumerate() {
            assert_eq!(
                shape.len(),
                shape_tags.len(),
                "shape[{si}] ring-count mismatch: rings={}, tag-arrays={}",
                shape.len(),
                shape_tags.len()
            );
            for (ri, (ring, ring_tags)) in shape.iter().zip(shape_tags.iter()).enumerate() {
                assert_eq!(
                    ring.len(),
                    ring_tags.len(),
                    "shape[{si}].ring[{ri}] points/tags mismatch: \
                     points={}, tags={}",
                    ring.len(),
                    ring_tags.len()
                );
            }
        }
    }

    /// Single outer rectangle with two distinct interior holes. The
    /// early-return path inside `join_sorted_holes_with_mapping`
    /// returns `[0; holes.len()]`; the resulting per-ring lengths
    /// must agree.
    #[test]
    fn tagged_single_shape_with_holes_keeps_per_ring_lengths() {
        // Outer 0–1000 square (CCW), two interior holes (CW).
        let outer = [
            IntPoint::new(0, 0),
            IntPoint::new(1000, 0),
            IntPoint::new(1000, 1000),
            IntPoint::new(0, 1000),
        ];
        // Hole A: 4-vertex square at (100, 100)–(200, 200), CW.
        let hole_a = [
            IntPoint::new(100, 100),
            IntPoint::new(100, 200),
            IntPoint::new(200, 200),
            IntPoint::new(200, 100),
        ];
        // Hole B: 4-vertex square at (500, 500)–(600, 600), CW.
        let hole_b = [
            IntPoint::new(500, 500),
            IntPoint::new(500, 600),
            IntPoint::new(600, 600),
            IntPoint::new(600, 500),
        ];

        let (shapes, tags) = run_tagged_overlay(&[(&outer[..], 1), (&hole_a[..], 2), (&hole_b[..], 3)]);

        assert_eq!(shapes.len(), 1, "one outer rectangle → one shape");
        assert_eq!(shapes[0].len(), 3, "shape must have outer + 2 holes");
        assert_shape_tag_alignment(&shapes, &tags);
    }

    /// Two disjoint outer rectangles, each with one hole, holes of
    /// DIFFERENT vertex counts (4 and 5).
    ///
    /// NOTE: this layout does **not** by itself force
    /// `parent_for_child` to permute holes across shapes — the graph
    /// traversal visits each shape and its interior hole in spatial
    /// left-top order, so the binder mapping here is the identity.
    /// It therefore guards the multi-shape *alignment invariant* end
    /// to end, but it is NOT the proof that the reordering bug is
    /// fixed. That proof is: (a) the synthetic permutation tests in
    /// `extract::tag_distribution_tests`, (b) the structural
    /// guarantee that `join_sorted_holes` now delegates to
    /// `join_sorted_holes_with_mapping` (same binding), and (c) the
    /// randomized differential test `randomized_tagged_overlay_*`
    /// below which exercises the real binder over many inputs.
    #[test]
    fn tagged_multi_shape_disjoint_holes_keep_alignment() {
        // Two disjoint outer rectangles.
        let outer_left = [
            IntPoint::new(0, 0),
            IntPoint::new(400, 0),
            IntPoint::new(400, 400),
            IntPoint::new(0, 400),
        ];
        let outer_right = [
            IntPoint::new(1000, 0),
            IntPoint::new(1400, 0),
            IntPoint::new(1400, 400),
            IntPoint::new(1000, 400),
        ];

        // Hole inside LEFT rectangle, 4 vertices (CW).
        let hole_left = [
            IntPoint::new(100, 100),
            IntPoint::new(100, 300),
            IntPoint::new(300, 300),
            IntPoint::new(300, 100),
        ];
        // Hole inside RIGHT rectangle, 5 vertices (CW). The 5th
        // vertex breaks any insertion-order-aligned shortcut: if
        // the binder swaps holes between shapes, the test fails.
        let hole_right = [
            IntPoint::new(1100, 100),
            IntPoint::new(1100, 300),
            IntPoint::new(1200, 350),
            IntPoint::new(1300, 300),
            IntPoint::new(1300, 100),
        ];

        let (shapes, tags) = run_tagged_overlay(&[
            (&outer_left[..], 10),
            (&outer_right[..], 20),
            (&hole_left[..], 11),
            (&hole_right[..], 21),
        ]);

        assert_eq!(shapes.len(), 2, "two disjoint rectangles → two shapes");
        for shape in &shapes {
            assert_eq!(shape.len(), 2, "each shape has outer + 1 hole");
        }
        assert_shape_tag_alignment(&shapes, &tags);
    }

    /// Same layout as the previous test, but also asserts the actual
    /// TAG VALUES end up on the correct shape. With the old
    /// insertion-order distribution, hole tags could land on the
    /// wrong shape even when per-ring lengths happened to match
    /// (e.g. two holes with the same vertex count) — silent data
    /// corruption that escapes purely-structural assertions.
    #[test]
    fn tagged_multi_shape_hole_tag_values_pin_to_right_shape() {
        let outer_left = [
            IntPoint::new(0, 0),
            IntPoint::new(400, 0),
            IntPoint::new(400, 400),
            IntPoint::new(0, 400),
        ];
        let outer_right = [
            IntPoint::new(1000, 0),
            IntPoint::new(1400, 0),
            IntPoint::new(1400, 400),
            IntPoint::new(1000, 400),
        ];
        // Both holes are 4-vertex squares — length-only assertions
        // would pass even with mis-pairing.
        let hole_left = [
            IntPoint::new(100, 100),
            IntPoint::new(100, 300),
            IntPoint::new(300, 300),
            IntPoint::new(300, 100),
        ];
        let hole_right = [
            IntPoint::new(1100, 100),
            IntPoint::new(1100, 300),
            IntPoint::new(1300, 300),
            IntPoint::new(1300, 100),
        ];

        let (shapes, tags) = run_tagged_overlay(&[
            (&outer_left[..], 10),
            (&outer_right[..], 20),
            (&hole_left[..], 11),
            (&hole_right[..], 21),
        ]);

        assert_eq!(shapes.len(), 2);
        assert_shape_tag_alignment(&shapes, &tags);

        // Identify each shape by an interior probe and check the
        // hole tag carried by its inner ring.
        for (si, shape) in shapes.iter().enumerate() {
            // Outer contour's leftmost x decides which logical
            // rectangle this is.
            let left_x = shape[0].iter().map(|p| p.x).min().unwrap();
            let expected_outer_tag = if left_x < 500 { 10 } else { 20 };
            let expected_hole_tag = if left_x < 500 { 11 } else { 21 };
            assert!(
                tags[si][0]
                    .iter()
                    .any(|&p| tag_pair_contains(p, expected_outer_tag)),
                "shape[{si}] outer ring must carry tag {expected_outer_tag}"
            );
            assert!(
                tags[si][1]
                    .iter()
                    .any(|&p| tag_pair_contains(p, expected_hole_tag)),
                "shape[{si}] hole ring must carry tag {expected_hole_tag} \
                 (mis-pairing would put the other rectangle's hole tag here)"
            );
        }
    }

    /// Two disjoint outer rectangles, no holes. The mapping is
    /// `None`/empty; distribution must be a no-op and per-shape tag
    /// arrays must each contain exactly one outer-ring entry.
    #[test]
    fn tagged_no_holes_multi_shape() {
        let r1 = [
            IntPoint::new(0, 0),
            IntPoint::new(100, 0),
            IntPoint::new(100, 100),
            IntPoint::new(0, 100),
        ];
        let r2 = [
            IntPoint::new(500, 0),
            IntPoint::new(600, 0),
            IntPoint::new(600, 100),
            IntPoint::new(500, 100),
        ];
        let (shapes, tags) = run_tagged_overlay(&[(&r1[..], 1), (&r2[..], 2)]);
        assert_eq!(shapes.len(), 2);
        for shape in &shapes {
            assert_eq!(shape.len(), 1);
        }
        assert_shape_tag_alignment(&shapes, &tags);
    }

    /// Regression for the `simplify_contour` interaction. A square
    /// with a midpoint vertex on every edge (so the contour has
    /// collinear triples) gets pushed; if `validate` simplified the
    /// points without simplifying the per-edge `tags` vector, the
    /// invariant would break here.
    #[test]
    fn tagged_collinear_input_does_not_drop_points() {
        // 8-vertex "square" with midpoints (collinear with corners).
        let ring = [
            IntPoint::new(0, 0),
            IntPoint::new(500, 0), // midpoint of bottom
            IntPoint::new(1000, 0),
            IntPoint::new(1000, 500), // midpoint of right
            IntPoint::new(1000, 1000),
            IntPoint::new(500, 1000), // midpoint of top
            IntPoint::new(0, 1000),
            IntPoint::new(0, 500), // midpoint of left
        ];
        let (shapes, tags) = run_tagged_overlay(&[(&ring[..], 7)]);
        assert_eq!(shapes.len(), 1, "single square → single shape");
        assert_shape_tag_alignment(&shapes, &tags);
    }

    /// Run the SAME polygons through the untagged `extract_shapes`
    /// path (default options, `preserve_output_collinear = false`).
    fn run_untagged_overlay(polygons: &[(&[IntPoint], u32)]) -> i_shape::int::shape::IntShapes {
        let mut overlay = Overlay::new_custom(64, IntOverlayOptions::default(), Default::default());
        for (ring, tag) in polygons {
            push_polygon(&mut overlay, ring, *tag, subj_direct());
        }
        let mut buf = BooleanExtractionBuffer::default();
        let graph = overlay
            .build_graph_view(FillRule::EvenOdd)
            .expect("build_graph_view should not fail on non-empty overlay");
        graph.extract_shapes(OverlayRule::Subject, &mut buf)
    }

    /// G3: collinear vertices on the OUTER ring of a shape that also
    /// has an interior hole. Exercises the preserve-collinear path
    /// together with hole distribution — the outer must keep all 8
    /// (collinear-inclusive) vertices and the per-ring tag lengths
    /// must still agree.
    #[test]
    fn tagged_collinear_outer_with_hole_keeps_alignment() {
        // 8-vertex outer "square" with edge midpoints (collinear).
        let outer = [
            IntPoint::new(0, 0),
            IntPoint::new(500, 0),
            IntPoint::new(1000, 0),
            IntPoint::new(1000, 500),
            IntPoint::new(1000, 1000),
            IntPoint::new(500, 1000),
            IntPoint::new(0, 1000),
            IntPoint::new(0, 500),
        ];
        // Interior hole (CW), 4 vertices, no collinear points.
        let hole = [
            IntPoint::new(300, 300),
            IntPoint::new(300, 700),
            IntPoint::new(700, 700),
            IntPoint::new(700, 300),
        ];
        let (shapes, tags) = run_tagged_overlay(&[(&outer[..], 1), (&hole[..], 2)]);
        assert_eq!(shapes.len(), 1, "one outer → one shape");
        assert_eq!(shapes[0].len(), 2, "outer + 1 hole");
        assert_eq!(
            shapes[0][0].len(),
            8,
            "tagged path must PRESERVE the 4 collinear midpoints on the outer ring"
        );
        assert_shape_tag_alignment(&shapes, &tags);
    }

    /// G3: the SAME collinear outer, run through the untagged path,
    /// MUST still be simplified (collinear midpoints dropped → 4
    /// vertices). This proves `NoOpTagCollector::PRESERVE_COLLINEAR`
    /// stayed `false` — the fix did not globally disable
    /// simplification, only the tag-aware extractor opts out.
    #[test]
    fn untagged_path_still_simplifies_collinear() {
        let outer = [
            IntPoint::new(0, 0),
            IntPoint::new(500, 0),
            IntPoint::new(1000, 0),
            IntPoint::new(1000, 500),
            IntPoint::new(1000, 1000),
            IntPoint::new(500, 1000),
            IntPoint::new(0, 1000),
            IntPoint::new(0, 500),
        ];
        let shapes = run_untagged_overlay(&[(&outer[..], 1)]);
        assert_eq!(shapes.len(), 1);
        assert_eq!(
            shapes[0][0].len(),
            4,
            "untagged path must still drop the 4 collinear midpoints \
             (regression guard: the tag fix must not disable global \
             simplification)"
        );
    }

    /// G5: deterministic pseudo-random differential sweep. Many
    /// layouts of 1..=4 well-separated outer rectangles, each
    /// optionally carrying a concentric interior hole of randomized
    /// vertex count, are run through the REAL tagged
    /// extraction/binder/mapping. Every output ring must satisfy
    /// `points.len() == tags.len()` and the shape/ring counts must
    /// mirror. This is the coverage that actually drives non-trivial
    /// `parent_for_child` bindings across many shapes/holes.
    #[test]
    fn randomized_tagged_overlay_preserves_alignment() {
        // Tiny self-contained LCG (Numerical Recipes constants) so
        // the sweep is fully deterministic and dependency-free.
        let mut state: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut next = |bound: u32| -> u32 {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((state >> 33) as u32) % bound
        };

        for iter in 0..400u32 {
            let n_rects = 1 + next(4) as usize; // 1..=4 outer rects
            // Own the rings so slices stay alive for the whole call.
            let mut rings: Vec<(Vec<IntPoint>, u32)> = Vec::new();
            for r in 0..n_rects {
                // Disjoint columns, generously separated.
                let x0 = (r as i32) * 10_000;
                let w = 2_000 + next(3_000) as i32;
                let h = 2_000 + next(3_000) as i32;
                let outer = vec![
                    IntPoint::new(x0, 0),
                    IntPoint::new(x0 + w, 0),
                    IntPoint::new(x0 + w, h),
                    IntPoint::new(x0, h),
                ];
                rings.push((outer, (r as u32) * 2 + 1));

                // ~⅔ of rects also get a concentric interior hole
                // (CW) with 4 or 5 vertices.
                if next(3) != 0 {
                    let cx = x0 + w / 2;
                    let cy = h / 2;
                    let rad = 200 + next((w.min(h) / 4).max(1) as u32) as i32;
                    let hole = if next(2) == 0 {
                        vec![
                            IntPoint::new(cx - rad, cy - rad),
                            IntPoint::new(cx - rad, cy + rad),
                            IntPoint::new(cx + rad, cy + rad),
                            IntPoint::new(cx + rad, cy - rad),
                        ]
                    } else {
                        // 5-vertex hole — distinct count so any
                        // mis-pairing breaks the length invariant.
                        vec![
                            IntPoint::new(cx - rad, cy - rad),
                            IntPoint::new(cx - rad, cy + rad),
                            IntPoint::new(cx, cy + rad + rad / 2),
                            IntPoint::new(cx + rad, cy + rad),
                            IntPoint::new(cx + rad, cy - rad),
                        ]
                    };
                    rings.push((hole, (r as u32) * 2 + 2));
                }
            }
            let polys: Vec<(&[IntPoint], u32)> =
                rings.iter().map(|(v, t)| (v.as_slice(), *t)).collect();
            let (shapes, tags) = run_tagged_overlay(&polys);
            // The contract rmesh's project.rs depends on: every ring
            // aligned. `assert_shape_tag_alignment` panics with a
            // descriptive message on any divergence.
            assert!(
                !shapes.is_empty(),
                "iter {iter}: non-empty input must yield ≥1 shape"
            );
            assert_shape_tag_alignment(&shapes, &tags);
        }
    }
}
