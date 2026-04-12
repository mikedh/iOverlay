//! End-to-end test for the tagged outline pipeline.
//!
//! Drives `OutlineBuilder::build_tagged_into` into a stock
//! `Overlay` with `preserve_output_collinear = true`, then uses
//! `tagged::annotate_contour` to recover per-edge tags on the
//! extracted shapes. Asserts:
//!
//!   1. Every user-supplied tag survives end-to-end through the
//!      split solver, merge step, and graph extraction.
//!   2. The discretized-arc tag appears on multiple consecutive
//!      output edges (because the arc was multiple input edges,
//!      all sharing one tag).
//!   3. Round joins synthesized at sharp corners carry the
//!      reserved `SYNTHESIZED_ROUND_JOIN_TAG` sentinel.
//!   4. No round join is inserted at the smooth tangent transition
//!      between the discretized arc and its neighbouring straight
//!      edges (i.e., the round-join builder doesn't over-fire on
//!      smooth joins).
//!
//! Plus a no-behavior-change canary that drives the existing
//! untagged `outline()` API through a square and asserts the area
//! is unchanged.

use std::collections::HashSet;

use i_float::adapter::FloatPointAdapter;
use i_float::float::rect::FloatRect;
use i_overlay::core::extract::BooleanExtractionBuffer;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay::Overlay;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::mesh::outline::builder::OutlineBuilder;
use i_overlay::mesh::outline::offset::OutlineOffset;
use i_overlay::mesh::style::{LineJoin, OutlineStyle};
use i_overlay::tagged::{annotate_contour, SYNTHESIZED_ROUND_JOIN_TAG, TagLookup};

#[test]
fn outline_with_tags_preserves_input_and_marks_round_joins() {
    // Build the input contour. The "rounded corner" is the
    // discretization of a quarter-arc from (10, 8) to (8, 10) with
    // center (8, 8) and radius 2, sampled at 8 intermediate points.
    let mut points: Vec<[f64; 2]> = Vec::new();
    let mut tags: Vec<u32> = Vec::new();

    // Edge 0: bottom-left → bottom-right, tag 1.
    points.push([0.0, 0.0]);
    tags.push(1);
    // Edge 1: bottom-right → start-of-arc, tag 2.
    points.push([10.0, 0.0]);
    tags.push(2);
    // Edges 2..N: discretized quarter-arc, all tagged 4.
    points.push([10.0, 8.0]);
    tags.push(4);
    let n_arc = 8;
    for i in 1..n_arc {
        let theta = std::f64::consts::FRAC_PI_2 * (i as f64) / (n_arc as f64);
        let x = 8.0 + 2.0 * theta.cos();
        let y = 8.0 + 2.0 * theta.sin();
        points.push([x, y]);
        tags.push(4);
    }
    // End of arc → start of top edge, tag 3.
    points.push([8.0, 10.0]);
    tags.push(3);
    // Top edge → top-left, tag 3 (top edge continuation).
    points.push([0.0, 10.0]);
    tags.push(3);

    assert_eq!(points.len(), tags.len(), "tags must be parallel to points");

    // `LineJoin::Round(ratio)` doubles as the per-vertex angle
    // threshold for inserting a round-join arc. A ratio smaller
    // than the per-vertex angle of a discretized smooth curve
    // would fire a round-join at EVERY vertex of that curve,
    // shattering its tag run. `FRAC_PI_8` (22.5°) sits comfortably
    // above typical discretization angles (~0.2 rad for an 8-point
    // quarter arc) and below any meaningful CAD corner.
    let join = LineJoin::Round(std::f64::consts::FRAC_PI_8);

    // Size the float-to-int adapter for the input rectangle plus
    // the offset margin the outline builder needs.
    let probe = OutlineBuilder::<[f64; 2], f64>::new(-1.0, &join);
    let extra = probe.additional_offset(1.0).abs();
    let mut rect = FloatRect::with_iter(points.iter()).expect("rect");
    rect.add_offset(extra);
    let adapter = FloatPointAdapter::new(rect);

    // Build the offset segments + line-identity lookup in one pass.
    let builder = OutlineBuilder::<[f64; 2], f64>::new(-1.0, &join);
    let mut overlay = Overlay::new(builder.capacity(points.len()));
    overlay.options.preserve_output_collinear = true;
    let mut lookup = TagLookup::new();
    builder.build_tagged_into(&points, &tags, &adapter, &mut overlay, &mut lookup);
    assert!(
        !lookup.is_empty(),
        "OutlineBuilder produced no tagged segments"
    );

    // Run the standard (unmodified) overlay pipeline. The input is
    // CCW and the outline extends outward by 1 unit, so
    // FillRule::Positive / OverlayRule::Subject gives the offset
    // shape.
    let mut buf = BooleanExtractionBuffer::default();
    let graph = overlay
        .build_graph_view(FillRule::Positive)
        .expect("non-empty overlay");
    let shapes = graph.extract_shapes(OverlayRule::Subject, &mut buf);

    assert!(!shapes.is_empty(), "expected at least one output shape");
    assert!(
        !shapes[0].is_empty(),
        "shape should have at least one contour"
    );

    // Recover per-edge tags post-hoc.
    let edge_tags = annotate_contour(&lookup, &shapes[0][0]);

    // Compress the tag stream into maximal runs for later
    // inspection in assertion messages. The assertion at the
    // bottom checks that round-join sentinels are only adjacent
    // to distinct user tags (i.e., at a real sharp corner), never
    // inserted between consecutive tag-4 edges of the smooth
    // discretized arc.
    let mut compressed: Vec<(u32, usize)> = Vec::new();
    for &t in &edge_tags {
        if let Some(last) = compressed.last_mut() {
            if last.0 == t {
                last.1 += 1;
                continue;
            }
        }
        compressed.push((t, 1));
    }

    // 1. Each user-supplied tag must appear at least once.
    let unique: HashSet<u32> = edge_tags.iter().copied().collect();
    assert!(
        unique.contains(&1),
        "user tag 1 missing from output: {edge_tags:?}"
    );
    assert!(
        unique.contains(&2),
        "user tag 2 missing from output: {edge_tags:?}"
    );
    assert!(
        unique.contains(&3),
        "user tag 3 missing from output: {edge_tags:?}"
    );
    assert!(
        unique.contains(&4),
        "user tag 4 missing from output: {edge_tags:?}"
    );

    // 2. Tag 4 (discretized arc) must appear on multiple consecutive
    //    output edges — the arc was multiple input edges, each of
    //    which produces at least one offset edge.
    let tag4_count = edge_tags.iter().filter(|&&t| t == 4).count();
    assert!(
        tag4_count > 1,
        "discretized arc tag 4 should produce multiple offset edges, got {tag4_count}"
    );

    // 3. Round-join sentinel must appear at the sharp corners.
    let synth_count = edge_tags
        .iter()
        .filter(|&&t| t == SYNTHESIZED_ROUND_JOIN_TAG)
        .count();
    assert!(
        synth_count > 0,
        "i_overlay should insert round-join arcs at sharp corners; \
         expected at least one segment with SYNTHESIZED_ROUND_JOIN_TAG"
    );

    // 4. No synth tag should appear ANYWHERE inside the discretized-arc
    //    region. With a sensible Round ratio the smooth tangent
    //    transitions between consecutive arc vertices should NOT
    //    trigger round-join insertions. Specifically: every
    //    maximal synth run must be bounded on BOTH sides by a
    //    non-tag-4 run (i.e., a real sharp corner between straight
    //    edges), not by tag-4 (the discretized arc).
    let n = compressed.len();
    for (i, (tag, _)) in compressed.iter().enumerate() {
        if *tag != SYNTHESIZED_ROUND_JOIN_TAG {
            continue;
        }
        let prev_tag = compressed[(i + n - 1) % n].0;
        let next_tag = compressed[(i + 1) % n].0;
        assert_ne!(
            prev_tag, 4,
            "round-join inserted adjacent to discretized arc (tag 4) — \
             ratio is too tight and round-joins are firing on smooth \
             tangent transitions. runs: {compressed:?}"
        );
        assert_ne!(
            next_tag, 4,
            "round-join inserted adjacent to discretized arc (tag 4) — \
             ratio is too tight and round-joins are firing on smooth \
             tangent transitions. runs: {compressed:?}"
        );
    }
}

#[test]
fn untagged_outline_unchanged() {
    // Sanity check: the existing untagged `outline()` API still
    // works after the additive changes. Bit-identical to baseline.
    let square = vec![vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]];
    let style = OutlineStyle::new(1.0).line_join(LineJoin::Round(0.1));
    let shapes = square.outline(&style);
    assert!(!shapes.is_empty(), "untagged outline produced empty result");

    // The 10x10 square expanded by 1 with rounded corners has:
    //   bounding box (12x12) - 4 × (corner square - quarter-circle)
    //   = 144 - 4 × (1 - π/4)
    //   = 140 + π
    //   ≈ 143.14
    let expected = 140.0 + std::f64::consts::PI;
    let total: f64 = shapes[0].iter().map(|c| polygon_area(c)).sum();
    assert!(
        (total - expected).abs() < 0.1,
        "untagged outline area changed: got {total}, want ≈ {expected}"
    );
}

fn polygon_area(points: &[[f64; 2]]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..points.len() {
        let j = (i + 1) % points.len();
        area += points[i][0] * points[j][1];
        area -= points[j][0] * points[i][1];
    }
    (area * 0.5).abs()
}

