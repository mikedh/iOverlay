use super::overlay_rule::OverlayRule;
use crate::bind::segment::{ContourIndex, IdSegment};
use crate::bind::solver::{JoinHoles, LeftBottomSegment};
use crate::core::graph::{OverlayGraph, OverlayNode};
use crate::core::link::OverlayLink;
use crate::core::link::OverlayLinkFilter;
use crate::core::nearest_vector::NearestVector;
use crate::core::overlay::ContourDirection;
use crate::geom::v_segment::VSegment;
use crate::i_shape::flat::buffer::FlatContoursBuffer;
use crate::segm::segment::TagPair;
use alloc::vec;
use alloc::vec::Vec;
use i_float::int::point::IntPoint;
use i_float::triangle::Triangle;
use i_shape::int::path::ContourExtension;
use i_shape::int::shape::{IntContour, IntShapes};
use i_shape::int::simple::Simplify;
use i_shape::util::reserve::Reserve;

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub(crate) enum VisitState {
    #[default]
    Unvisited = 0,
    Skipped = 1,
    HoleVisited = 2,
    HullVisited = 3,
}

#[derive(Default)]
pub struct BooleanExtractionBuffer {
    pub(crate) points: Vec<IntPoint>,
    pub(crate) tags: Vec<TagPair>,
    pub(crate) visited: Vec<VisitState>,
    pub(crate) contour_visited: Option<Vec<VisitState>>,
}

impl OverlayGraph<'_> {
    /// Extracts shapes from the overlay graph based on the specified overlay rule. This method is used to retrieve the final geometric shapes after boolean operations have been applied. It's suitable for most use cases where the minimum area of shapes is not a concern.
    /// - `overlay_rule`: The boolean operation rule to apply when extracting shapes from the graph, such as union or intersection.
    /// - `buffer`: Reusable buffer, optimisation purpose only.
    /// - Returns: A vector of `IntShape`, representing the geometric result of the applied overlay rule.
    /// # Shape Representation
    /// The output is a `IntShapes`, where:
    /// - The outer `Vec<IntShape>` represents a set of shapes.
    /// - Each shape `Vec<IntContour>` represents a collection of contours, where the first contour is the outer boundary, and all subsequent contours are holes in this boundary.
    /// - Each path `Vec<IntPoint>` is a sequence of points, forming a closed path.
    ///
    /// Note: Outer boundary paths have a counterclockwise order, and holes have a clockwise order.
    #[inline]
    pub fn extract_shapes(
        &self,
        overlay_rule: OverlayRule,
        buffer: &mut BooleanExtractionBuffer,
    ) -> IntShapes {
        self.links
            .filter_by_overlay_into(overlay_rule, &mut buffer.visited);
        if self.options.ogc {
            self.extract_ogc(overlay_rule, buffer)
        } else {
            self.extract_no_tags(overlay_rule, buffer)
        }
    }

    /// Extract shapes with per-edge tags emitted inline during graph
    /// traversal. Each output edge carries the `TagPair` of the graph
    /// link it was traversed from — up to two source tags survive every
    /// merge, letting downstream consumers pick whichever matches their
    /// ring-local context (see [`TagPair`]).
    #[inline]
    pub fn extract_shapes_with_tags(
        &self,
        overlay_rule: OverlayRule,
        buffer: &mut BooleanExtractionBuffer,
    ) -> (IntShapes, Vec<Vec<Vec<TagPair>>>) {
        self.links
            .filter_by_overlay_into(overlay_rule, &mut buffer.visited);
        debug_assert!(!self.options.ogc, "OGC extraction doesn't support inline tags yet");
        self.extract_with_tags(overlay_rule, buffer)
    }

    /// Extracts the flat contours from the overlay graph based on the specified overlay rule.
    ///
    /// This method performs a Boolean operation (e.g., union or intersection) and stores the result
    /// directly into a flat buffer of contours, without nesting them into shapes (i.e., no hole-joining or grouping).
    ///
    /// It is optimized for performance and suitable when raw contour data is sufficient,
    /// such as during intermediate processing, visualization, or tesselation.
    ///
    /// - `overlay_rule`: The boolean operation rule to apply (e.g., union, intersection, xor).
    /// - `buffer`: Reusable working buffer to avoid reallocations.
    /// - `output`: A flat buffer to which the resulting valid contours will be written.
    #[inline]
    pub fn extract_contours_into(
        &self,
        overlay_rule: OverlayRule,
        buffer: &mut BooleanExtractionBuffer,
        output: &mut FlatContoursBuffer,
    ) {
        self.links
            .filter_by_overlay_into(overlay_rule, &mut buffer.visited);
        self.extract_contours(overlay_rule, buffer, output);
    }

    /// Fast extraction path — no tag collection, no tag allocation.
    fn extract_no_tags(
        &self,
        overlay_rule: OverlayRule,
        buffer: &mut BooleanExtractionBuffer,
    ) -> IntShapes {
        let (shapes, ()) = self.extract_impl::<NoOpTagCollector>(overlay_rule, buffer);
        shapes
    }

    pub(crate) fn extract_with_tags(
        &self,
        overlay_rule: OverlayRule,
        buffer: &mut BooleanExtractionBuffer,
    ) -> (IntShapes, Vec<Vec<Vec<TagPair>>>) {
        self.extract_impl::<VecTagCollector>(overlay_rule, buffer)
    }

    /// Shared body for [`Self::extract_no_tags`] and
    /// [`Self::extract_with_tags`]. The two paths are byte-identical
    /// except for tag work, which is hidden behind [`TagCollector`].
    /// `NoOpTagCollector` compiles every tag-related branch out, so
    /// this stays zero-cost on the fast path.
    fn extract_impl<TC: TagCollector>(
        &self,
        overlay_rule: OverlayRule,
        buffer: &mut BooleanExtractionBuffer,
    ) -> (IntShapes, TC::ShapeTags) {
        let clockwise = self.options.output_direction == ContourDirection::Clockwise;

        let mut shapes = Vec::new();
        let mut shape_tags = TC::ShapeTags::default();
        let mut holes = Vec::new();
        let mut hole_tags = TC::HoleTags::default();
        let mut anchors = Vec::new();

        buffer.points.reserve_capacity(buffer.visited.len());

        let mut link_index = 0;
        let mut anchors_already_sorted = true;
        while link_index < buffer.visited.len() {
            if buffer.visited.is_visited(link_index) {
                link_index += 1;
                continue;
            }

            let left_top_link = unsafe {
                GraphUtil::find_left_top_link(self.links, self.nodes, link_index, &buffer.visited)
            };

            let link = unsafe { self.links.get_unchecked(left_top_link) };
            let is_hole = overlay_rule.is_fill_top(link.fill);
            let visited_state = [VisitState::HullVisited, VisitState::HoleVisited][is_hole as usize];

            let direction = is_hole == clockwise;
            let start_data = StartPathData::new(direction, link, left_top_link);

            TC::traverse(
                self,
                &start_data,
                direction,
                visited_state,
                &mut buffer.visited,
                &mut buffer.points,
                &mut buffer.tags,
            );

            // Tag-aware extractors force `preserve_output_collinear =
            // true` regardless of the caller's option, because
            // `simplify_contour` (inside `validate`) trims collinear
            // vertices from `points` but cannot do the matching trim
            // on `tags` (which is sized one-per-traversed-link). The
            // length divergence was the root cause of the
            // `AccumulatorMerger::merge: reconciling ring with
            // mismatched points (N) vs tags (M)` warning storm in
            // mikedh/rmesh's `crates/rmesh/src/project.rs:778` —
            // every `examples/tool.py` invocation against
            // `airflow_redirector_axial.glb` triggered it dozens of
            // times. Untagged callers keep the original behavior.
            let preserve_collinear = self.options.preserve_output_collinear || TC::PRESERVE_COLLINEAR;
            let (is_valid, is_modified) = buffer
                .points
                .validate(self.options.min_output_area, preserve_collinear);

            if !is_valid {
                link_index += 1;
                continue;
            }

            // Strict invariant: per-link tag collection MUST keep
            // `tags.len() == points.len()` after `validate`. Untagged
            // callers don't allocate `tags`, so the check is skipped
            // via the const-bool. Any failure here is a contract bug,
            // not a data condition — panic so the upstream caller
            // can't paper over it with a `pad to match` recovery.
            //
            // Promoted from `debug_assert!` to `assert!` because
            // `maturin develop` historically built rmesh in release
            // mode (no debug_assert!), letting this invariant slip
            // silently — see the `AccumulatorMerger::merge` warning
            // storm fixed by this commit.
            assert!(
                !TC::PRESERVE_COLLINEAR || buffer.points.len() == buffer.tags.len(),
                "extract_with_tags: points/tags length mismatch — \
                 points={}, tags={}. `TC::PRESERVE_COLLINEAR=true` \
                 must keep them in lockstep; if `validate` is mutating \
                 points, simplify must also mutate tags (or be skipped).",
                buffer.points.len(),
                buffer.tags.len(),
            );

            let contour = buffer.points.as_slice().to_vec();

            if is_hole {
                let mut v_segment = if clockwise {
                    VSegment { a: contour[1], b: contour[2] }
                } else {
                    VSegment { a: contour[0], b: contour[contour.len() - 1] }
                };
                if is_modified {
                    let most_left = contour.left_bottom_segment();
                    if most_left != v_segment {
                        v_segment = most_left;
                        anchors_already_sorted = false;
                    }
                };

                debug_assert_eq!(v_segment, contour.left_bottom_segment());
                let id_data = ContourIndex::new_hole(holes.len());
                anchors.push(IdSegment::with_segment(id_data, v_segment));
                let pts_before_push = contour.len();
                holes.push(contour);
                TC::push_hole(&mut hole_tags, &buffer.tags);
                // Post-push invariant: this hole's points and tags
                // arrays must agree. `debug_assert!` — redundant with
                // the always-on per-contour length `assert!` above;
                // kept only as a finer-grained debug locator.
                debug_assert!(
                    !TC::PRESERVE_COLLINEAR || pts_before_push == TC::last_hole_len(&hole_tags),
                    "VecTagCollector: post-push_hole length mismatch — \
                     contour.len()={}, hole_tags.last().len()={}, \
                     buffer.tags.len()={}",
                    pts_before_push,
                    TC::last_hole_len(&hole_tags),
                    buffer.tags.len(),
                );
            } else {
                let pts_before_push = contour.len();
                shapes.push(vec![contour]);
                TC::push_shape(&mut shape_tags, &buffer.tags);
                debug_assert!(
                    !TC::PRESERVE_COLLINEAR || pts_before_push == TC::last_shape_outer_len(&shape_tags),
                    "VecTagCollector: post-push_shape length mismatch — \
                     contour.len()={}, shape_tags.last()[0].len()={}, \
                     buffer.tags.len()={}",
                    pts_before_push,
                    TC::last_shape_outer_len(&shape_tags),
                    buffer.tags.len(),
                );
            }
        }

        if !anchors_already_sorted {
            anchors.sort_unstable_by(|s0, s1| s0.v_segment.a.cmp(&s1.v_segment.a));
        }

        // Pre-flight invariant: holes and hole_tags must be in
        // lockstep at this point (lengths AND per-entry edge counts).
        // Caught early so a future change inside the main extraction
        // loop can't silently desync them.
        TC::assert_holes_lockstep(&hole_tags, &holes);

        // `join_sorted_holes_with_mapping` returns the hole→shape
        // assignment the binder actually used. We need this to
        // distribute `hole_tags` to the SAME shapes, otherwise the
        // multi-shape path silently corrupts per-ring tag arrays —
        // the root cause of the `AccumulatorMerger::merge: reconciling
        // ring with mismatched points (N) vs tags (M)` warning
        // storm in mikedh/rmesh that masqueraded as an i_overlay
        // bug for an entire debugging session before being traced
        // here. With `len==1` shape sets the early-return inside
        // `join_sorted_holes_with_mapping` returns `[0; holes.len()]`
        // so the same code path covers both single and multi-shape
        // cases.
        // Distribute tags using the EXACT hole→shape mapping the
        // binder applied (replaces the old insertion-order shortcut
        // that caused per-ring tag mis-pairing on multi-shape
        // outputs — root cause of the `AccumulatorMerger::merge` ring
        // warning storm in mikedh/rmesh).
        let hole_to_shape = shapes.join_sorted_holes_with_mapping(holes, anchors, clockwise);
        TC::distribute_hole_tags_mapped(&mut shape_tags, hole_tags, hole_to_shape.as_deref());

        // Strict invariant for the tagged extraction path: shape_tags
        // must mirror `shapes` exactly — same shape count, and within
        // each shape, same per-contour ring count AND same per-ring
        // edge count (`points.len() == tags.len()`). Any divergence is
        // a contract bug here, in `distribute_hole_tags_mapped`, or in
        // `join_sorted_holes_with_mapping` itself.
        TC::assert_consistent(&shape_tags, &shapes);

        (shapes, shape_tags)
    }

    pub(crate) fn find_contour(
        &self,
        start_data: &StartPathData,
        clockwise: bool,
        visited_state: VisitState,
        visited: &mut [VisitState],
        points: &mut Vec<IntPoint>,
        tags: &mut Vec<TagPair>,
    ) {
        let mut link_id = start_data.link_id;
        let mut node_id = start_data.node_id;
        let last_node_id = start_data.last_node_id;

        visited.visit_edge(link_id, visited_state);
        points.clear();
        tags.clear();
        points.push(start_data.begin);
        tags.push(start_data.start_tag);

        // Find a closed tour
        while node_id != last_node_id {
            link_id = GraphUtil::next_link(self.links, self.nodes, link_id, node_id, clockwise, visited);

            let link = unsafe {
                self.links.get_unchecked(link_id)
            };
            node_id = points.push_node_and_get_other(link, node_id);
            tags.push(link.tag);

            visited.visit_edge(link_id, visited_state);
        }
    }

    /// Fast contour traversal — points only, no tag collection.
    pub(crate) fn find_contour_no_tags(
        &self,
        start_data: &StartPathData,
        clockwise: bool,
        visited_state: VisitState,
        visited: &mut [VisitState],
        points: &mut Vec<IntPoint>,
    ) {
        let mut link_id = start_data.link_id;
        let mut node_id = start_data.node_id;
        let last_node_id = start_data.last_node_id;

        visited.visit_edge(link_id, visited_state);
        points.clear();
        points.push(start_data.begin);

        while node_id != last_node_id {
            link_id = GraphUtil::next_link(self.links, self.nodes, link_id, node_id, clockwise, visited);

            let link = unsafe {
                self.links.get_unchecked(link_id)
            };
            node_id = points.push_node_and_get_other(link, node_id);

            visited.visit_edge(link_id, visited_state);
        }
    }

    fn extract_contours(
        &self,
        overlay_rule: OverlayRule,
        buffer: &mut BooleanExtractionBuffer,
        output: &mut FlatContoursBuffer,
    ) {
        let clockwise = self.options.output_direction == ContourDirection::Clockwise;
        let len = buffer.visited.len();
        buffer.points.reserve_capacity(len);
        output.clear_and_reserve(len, 4);

        let mut link_index = 0;
        while link_index < len {
            if buffer.visited.is_visited(link_index) {
                link_index += 1;
                continue;
            }

            let left_top_link = unsafe {
                GraphUtil::find_left_top_link(self.links, self.nodes, link_index, &buffer.visited)
            };

            let link = unsafe {
                self.links.get_unchecked(left_top_link)
            };
            let is_hole = overlay_rule.is_fill_top(link.fill);
            let visited_state = [VisitState::HullVisited, VisitState::HoleVisited][is_hole as usize];

            let direction = is_hole == clockwise;
            let start_data = StartPathData::new(direction, link, left_top_link);

            self.find_contour_no_tags(
                &start_data,
                direction,
                visited_state,
                &mut buffer.visited,
                &mut buffer.points,
            );
            let (is_valid, _) = buffer.points.validate(
                self.options.min_output_area,
                self.options.preserve_output_collinear,
            );

            if !is_valid {
                link_index += 1;
                continue;
            }

            output.add_contour(buffer.points.as_slice());
        }
    }
}

/// Strategy used by [`OverlayGraph::extract_impl`] to handle per-edge
/// tag collection. Two impls exist:
///
/// - [`NoOpTagCollector`]: zero-cost strategy for callers that only need
///   the shape geometry. All methods are compiled down to nothing; the
///   associated types are `()` so no allocations happen.
/// - [`VecTagCollector`]: collects one `Vec<TagPair>` of link tags per
///   contour, nested inside a `Vec<Vec<Vec<TagPair>>>` (shape → contour →
///   edge-tag). Used by rmesh to propagate source-face tags through
///   booleans.
///
/// The trait is generic (not dyn-safe) so the fast path stays branchless.
pub(crate) trait TagCollector {
    /// Accumulator the impl builds up while walking shapes.
    type ShapeTags: Default;
    /// Accumulator the impl builds up while walking holes, before
    /// `distribute_hole_tags_mapped` folds them back into shapes.
    type HoleTags: Default;

    /// When `true`, the per-contour simplification pass that strips
    /// collinear points MUST be skipped, otherwise the `tags` vector
    /// (built one-to-one with the unsimplified `points` vector during
    /// graph traversal) ends up longer than the simplified `points`
    /// and every downstream consumer sees a corrupted ring.
    ///
    /// `NoOpTagCollector` keeps the default `false` so the untagged
    /// fast path is bit-identical to the pre-tag implementation;
    /// `VecTagCollector` flips this to `true` so the points-vs-tags
    /// length invariant is preserved by construction. Callers (e.g.
    /// rmesh's `TaggedContour::simplify`) can re-simplify in a
    /// tag-aware pass after extraction.
    const PRESERVE_COLLINEAR: bool = false;

    /// Walk one closed contour starting from `start_data`, recording
    /// points into `points` and tags into `tags` as appropriate for
    /// the impl. `NoOpTagCollector` dispatches to `find_contour_no_tags`
    /// (skips tag collection entirely); `VecTagCollector` dispatches to
    /// `find_contour`.
    fn traverse(
        graph: &OverlayGraph<'_>,
        start_data: &StartPathData,
        clockwise: bool,
        visited_state: VisitState,
        visited: &mut [VisitState],
        points: &mut Vec<IntPoint>,
        tags: &mut Vec<TagPair>,
    );

    /// Push the tags just written to `buffer_tags` onto the hole
    /// accumulator. `NoOpTagCollector` ignores the buffer.
    fn push_hole(hole_tags: &mut Self::HoleTags, buffer_tags: &[TagPair]);

    /// Push the tags just written to `buffer_tags` onto the shape
    /// accumulator, wrapping them as a single-contour shape (outer
    /// boundary). Holes are spliced in later via
    /// `distribute_hole_tags_mapped`.
    fn push_shape(shape_tags: &mut Self::ShapeTags, buffer_tags: &[TagPair]);

    /// Distribute `hole_tags` to their owning shapes using the EXACT
    /// `hole_index → shape_index` mapping that `join_sorted_holes_with_mapping`
    /// applied to the holes themselves. `mapping[k] == s` means
    /// `hole_tags[k]` belongs to `shape_tags[s]`, mirroring how
    /// `holes[k]` was pushed to `shapes[s]`.
    ///
    /// `None` mapping means no holes — nothing to distribute. Replaced
    /// an earlier insertion-order distribution that silently corrupted
    /// per-ring tag arrays on multi-shape outputs (the bug this whole
    /// change fixes; the buggy variant now lives only as a reference
    /// implementation in the `tag_distribution_tests` module).
    fn distribute_hole_tags_mapped(
        shape_tags: &mut Self::ShapeTags,
        hole_tags: Self::HoleTags,
        mapping: Option<&[usize]>,
    );

    /// Post-extraction structural assertion: `shape_tags` mirrors
    /// `shapes` exactly. `NoOpTagCollector` ignores the inputs; the
    /// tag-collecting impls verify shape count, per-shape ring count,
    /// and per-ring `tags.len() == points.len()`. Promoted from
    /// "warn-and-pad" downstream so divergence panics at the source.
    fn assert_consistent(shape_tags: &Self::ShapeTags, shapes: &IntShapes);

    /// Diagnostic peek: length of the most recently pushed hole's
    /// tag array. Used by per-push asserts; `NoOpTagCollector`
    /// returns `0` (never compared because `PRESERVE_COLLINEAR` is
    /// false there).
    fn last_hole_len(hole_tags: &Self::HoleTags) -> usize;

    /// Diagnostic peek: length of the outer-ring tag array of the
    /// most recently pushed shape.
    fn last_shape_outer_len(shape_tags: &Self::ShapeTags) -> usize;

    /// Diagnostic: assert `hole_tags` and `holes` align in length AND
    /// per-entry edge count, just before `join_sorted_holes` runs.
    fn assert_holes_lockstep(hole_tags: &Self::HoleTags, holes: &[IntContour]);
}

/// Zero-cost tag collector used by the untagged extract path.
pub(crate) struct NoOpTagCollector;

impl TagCollector for NoOpTagCollector {
    type ShapeTags = ();
    type HoleTags = ();

    #[inline(always)]
    fn traverse(
        graph: &OverlayGraph<'_>,
        start_data: &StartPathData,
        clockwise: bool,
        visited_state: VisitState,
        visited: &mut [VisitState],
        points: &mut Vec<IntPoint>,
        _tags: &mut Vec<TagPair>,
    ) {
        graph.find_contour_no_tags(start_data, clockwise, visited_state, visited, points);
    }

    #[inline(always)]
    fn push_hole(_: &mut (), _: &[TagPair]) {}

    #[inline(always)]
    fn push_shape(_: &mut (), _: &[TagPair]) {}

    #[inline(always)]
    fn distribute_hole_tags_mapped(_: &mut (), _: (), _: Option<&[usize]>) {}

    #[inline(always)]
    fn assert_consistent(_: &(), _: &IntShapes) {}

    #[inline(always)]
    fn last_hole_len(_: &()) -> usize {
        0
    }

    #[inline(always)]
    fn last_shape_outer_len(_: &()) -> usize {
        0
    }

    #[inline(always)]
    fn assert_holes_lockstep(_: &(), _: &[IntContour]) {}
}

/// Collects per-edge tags as `Vec<Vec<Vec<TagPair>>>` (shape → contour →
/// edge-tag). Matches the shape of `IntShapes` 1-to-1.
pub(crate) struct VecTagCollector;

impl TagCollector for VecTagCollector {
    type ShapeTags = Vec<Vec<Vec<TagPair>>>;
    type HoleTags = Vec<Vec<TagPair>>;

    // Tag-aware extraction MUST keep collinear points; see the trait
    // doc for the points-vs-tags length invariant.
    const PRESERVE_COLLINEAR: bool = true;

    #[inline]
    fn traverse(
        graph: &OverlayGraph<'_>,
        start_data: &StartPathData,
        clockwise: bool,
        visited_state: VisitState,
        visited: &mut [VisitState],
        points: &mut Vec<IntPoint>,
        tags: &mut Vec<TagPair>,
    ) {
        graph.find_contour(start_data, clockwise, visited_state, visited, points, tags);
    }

    #[inline]
    fn push_hole(hole_tags: &mut Self::HoleTags, buffer_tags: &[TagPair]) {
        hole_tags.push(buffer_tags.to_vec());
    }

    #[inline]
    fn push_shape(shape_tags: &mut Self::ShapeTags, buffer_tags: &[TagPair]) {
        shape_tags.push(vec![buffer_tags.to_vec()]);
    }

    fn distribute_hole_tags_mapped(
        shape_tags: &mut Self::ShapeTags,
        hole_tags: Self::HoleTags,
        mapping: Option<&[usize]>,
    ) {
        let Some(mapping) = mapping else {
            return;
        };
        assert_eq!(
            mapping.len(),
            hole_tags.len(),
            "VecTagCollector::distribute_hole_tags_mapped: mapping len ({}) \
             must equal hole_tags len ({})",
            mapping.len(),
            hole_tags.len(),
        );
        for (hole_index, tag_array) in hole_tags.into_iter().enumerate() {
            let shape_index = mapping[hole_index];
            assert!(
                shape_index < shape_tags.len(),
                "VecTagCollector::distribute_hole_tags_mapped: hole {hole_index} \
                 maps to shape {shape_index} but only {} shape tag arrays exist",
                shape_tags.len(),
            );
            shape_tags[shape_index].push(tag_array);
        }
    }

    fn last_hole_len(hole_tags: &Self::HoleTags) -> usize {
        hole_tags.last().map_or(0, |t| t.len())
    }

    fn last_shape_outer_len(shape_tags: &Self::ShapeTags) -> usize {
        shape_tags.last().and_then(|s| s.first()).map_or(0, |t| t.len())
    }

    // O(Σ hole edges) full scan — `debug_assert_eq!` so it costs
    // nothing in release builds for the geometry-kernel hot path.
    // The always-on guarantee is the O(1) per-contour `assert!` in
    // `extract_impl`; this is a finer-grained debug locator only.
    fn assert_holes_lockstep(hole_tags: &Self::HoleTags, holes: &[IntContour]) {
        debug_assert_eq!(
            hole_tags.len(),
            holes.len(),
            "VecTagCollector: holes vs hole_tags top-level count mismatch \
             — holes={}, hole_tags={}",
            holes.len(),
            hole_tags.len(),
        );
        for (i, (hole, tags)) in holes.iter().zip(hole_tags.iter()).enumerate() {
            debug_assert_eq!(
                hole.len(),
                tags.len(),
                "VecTagCollector: hole[{i}] points={}, tags={} \
                 — divergence is between push_hole and join_sorted_holes",
                hole.len(),
                tags.len(),
            );
        }
    }

    // O(Σ ring edges) full scan — `debug_assert_eq!` for the same
    // reason as `assert_holes_lockstep`: the always-on guarantee is
    // the per-contour `assert!`; this is a debug-only locator.
    fn assert_consistent(shape_tags: &Self::ShapeTags, shapes: &IntShapes) {
        debug_assert_eq!(
            shape_tags.len(),
            shapes.len(),
            "VecTagCollector: shape_tags vs shapes shape-count mismatch \
             — shape_tags={}, shapes={}",
            shape_tags.len(),
            shapes.len(),
        );
        for (si, (shape, tags)) in shapes.iter().zip(shape_tags.iter()).enumerate() {
            debug_assert_eq!(
                shape.len(),
                tags.len(),
                "VecTagCollector: shape[{si}] ring-count mismatch — \
                 shape rings={}, tags arrays={}",
                shape.len(),
                tags.len(),
            );
            for (ri, (ring_pts, ring_tags)) in shape.iter().zip(tags.iter()).enumerate() {
                debug_assert_eq!(
                    ring_pts.len(),
                    ring_tags.len(),
                    "VecTagCollector: shape[{si}].ring[{ri}] edge count \
                     mismatch — points={}, tags={}",
                    ring_pts.len(),
                    ring_tags.len(),
                );
            }
        }
    }
}

pub(crate) struct StartPathData {
    pub(crate) begin: IntPoint,
    pub(crate) node_id: usize,
    pub(crate) link_id: usize,
    pub(crate) last_node_id: usize,
    pub(crate) start_tag: TagPair,
}

impl StartPathData {
    #[inline(always)]
    pub(crate) fn new(direction: bool, link: &OverlayLink, link_id: usize) -> Self {
        if direction {
            Self {
                begin: link.b.point,
                node_id: link.a.id,
                link_id,
                last_node_id: link.b.id,
                start_tag: link.tag,
            }
        } else {
            Self {
                begin: link.a.point,
                node_id: link.b.id,
                link_id,
                last_node_id: link.a.id,
                start_tag: link.tag,
            }
        }
    }
}

pub(crate) trait GraphContour {
    fn validate(&mut self, min_output_area: u64, preserve_output_collinear: bool) -> (bool, bool);
    fn push_node_and_get_other(&mut self, link: &OverlayLink, node_id: usize) -> usize;
}

impl GraphContour for IntContour {
    #[inline]
    fn validate(&mut self, min_output_area: u64, preserve_output_collinear: bool) -> (bool, bool) {
        let is_modified = if !preserve_output_collinear {
            self.simplify_contour()
        } else {
            false
        };

        if self.len() < 3 {
            return (false, is_modified);
        }

        if min_output_area == 0 {
            return (true, is_modified);
        }
        let area = self.unsafe_area();
        let abs_area = area.unsigned_abs() >> 1;
        let is_valid = abs_area >= min_output_area;

        (is_valid, is_modified)
    }

    #[inline]
    fn push_node_and_get_other(&mut self, link: &OverlayLink, node_id: usize) -> usize {
        if link.a.id == node_id {
            self.push(link.a.point);
            link.b.id
        } else {
            self.push(link.b.point);
            link.a.id
        }
    }
}

impl VisitState {
    #[inline(always)]
    pub(crate) fn new(skipped: bool) -> Self {
        let raw = skipped as u8; // 0 or 1
        debug_assert!(raw <= VisitState::Skipped as u8);
        // SAFETY: repr(u8) and raw is in range 0..=1
        unsafe { core::mem::transmute(raw) }
    }
}

pub(crate) trait Visit {
    fn is_visited(&self, index: usize) -> bool;
    fn is_not_visited(&self, index: usize) -> bool;
    fn visit_edge(&mut self, index: usize, state: VisitState);
}

// Safety: every call site creates `visited` slices with one entry per link/node,
// and they only pass indices directly obtained from those slices. That keeps
// `index < self.len()` true for the lifetime of the traversal.
impl Visit for [VisitState] {
    #[inline(always)]
    fn is_visited(&self, index: usize) -> bool {
        unsafe {
            // SAFETY: callers only pass indices derived from the visited slice itself, so index < len.
            *self.get_unchecked(index) != VisitState::Unvisited
        }
    }

    #[inline(always)]
    fn is_not_visited(&self, index: usize) -> bool {
        unsafe {
            // SAFETY: callers only pass indices derived from the visited slice itself, so index < len.
            *self.get_unchecked(index) == VisitState::Unvisited
        }
    }
    #[inline(always)]
    fn visit_edge(&mut self, index: usize, state: VisitState) {
        unsafe {
            // SAFETY: callers only pass indices derived from the visited slice itself, so index < len.
            *self.get_unchecked_mut(index) = state;
        }
    }
}

pub(crate) struct GraphUtil;

impl GraphUtil {
    /// # Safety
    /// * `link_index < links.len()`
    /// * `links[top.a.id]` must exist for every link
    /// * For bridge nodes, both `bridge[k] < links.len()`
    /// * `visited` is at least `links.len()` long (or whatever invariant applies)
    #[inline]
    pub(crate) unsafe fn find_left_top_link(
        links: &[OverlayLink],
        nodes: &[OverlayNode],
        link_index: usize,
        visited: &[VisitState],
    ) -> usize {
        let top = unsafe {
            // SAFETY: link_index is always < links.len(); callers either iterate that range or
            // pull the value from visited, which mirrors links.
            links.get_unchecked(link_index)
        };
        let node = unsafe {
            // SAFETY: GraphBuilder assigns `a.id` from the index in the `nodes` vector,
            // so every `id` is guaranteed to lie in 0..nodes.len().
            nodes.get_unchecked(top.a.id)
        };

        debug_assert!(top.is_direct());

        match node {
            OverlayNode::Bridge(bridge) => Self::find_left_top_link_on_bridge(links, bridge),
            OverlayNode::Cross(indices) => {
                Self::find_left_top_link_on_indices(links, top, link_index, indices, visited)
            }
        }
    }

    #[inline(always)]
    fn find_left_top_link_on_indices(
        links: &[OverlayLink],
        link: &OverlayLink,
        link_index: usize,
        indices: &[usize],
        visited: &[VisitState],
    ) -> usize {
        let mut top_index = link_index;
        let mut top = link;

        // find most top link

        for &i in indices.iter() {
            if i == link_index {
                continue;
            }
            let link = unsafe {
                // SAFETY: indices holds link ids emitted by GraphBuilder, so each i < links.len().
                links.get_unchecked(i)
            };
            if !link.is_direct() || Triangle::is_clockwise_point(top.a.point, top.b.point, link.b.point) {
                continue;
            }

            if visited.is_visited(i) {
                continue;
            }

            top_index = i;
            top = link;
        }

        top_index
    }

    #[inline(always)]
    fn find_left_top_link_on_bridge(links: &[OverlayLink], bridge: &[usize; 2]) -> usize {
        // SAFETY: every bridge index comes straight from GraphBuilder::build_nodes_and_connect_links,
        // which only records values in 0..links.len(), so the unchecked lookups stay in-bounds.
        let (l0, l1) = unsafe { (links.get_unchecked(bridge[0]), links.get_unchecked(bridge[1])) };
        if Triangle::is_clockwise_point(l0.a.point, l0.b.point, l1.b.point) {
            bridge[0]
        } else {
            bridge[1]
        }
    }

    #[inline(always)]
    pub(crate) fn next_link(
        links: &[OverlayLink],
        nodes: &[OverlayNode],
        link_id: usize,
        node_id: usize,
        clockwise: bool,
        visited: &[VisitState],
    ) -> usize {
        let node = unsafe {
            // SAFETY: all node ids flowing through traversal originate from GraphBuilder,
            // hence are within `0..nodes.len()`.
            nodes.get_unchecked(node_id)
        };
        match node {
            OverlayNode::Bridge(bridge) => {
                if bridge[0] == link_id {
                    bridge[1]
                } else {
                    bridge[0]
                }
            }
            OverlayNode::Cross(indices) => {
                GraphUtil::find_nearest_link_to(links, link_id, node_id, clockwise, indices, visited)
            }
        }
    }

    // Assumes: `indices` comes from an OverlayNode::Cross built by GraphBuilder,
    // so every element is a valid index into `links`, and at least one of them is
    // still unvisited when we enter. The unchecked accesses rely on that invariant.
    #[inline]
    fn find_nearest_link_to(
        links: &[OverlayLink],
        target_index: usize,
        node_id: usize,
        clockwise: bool,
        indices: &[usize],
        visited: &[VisitState],
    ) -> usize {
        let mut is_first = true;
        let mut first_index = 0;
        let mut second_index = usize::MAX;
        let mut pos = 0;
        for (i, &link_index) in indices.iter().enumerate() {
            if visited.is_not_visited(link_index) {
                if is_first {
                    first_index = link_index;
                    is_first = false;
                } else {
                    second_index = link_index;
                    pos = i;
                    break;
                }
            }
        }

        if second_index == usize::MAX {
            return first_index;
        }

        let target = unsafe {
            // SAFETY: target_index is either the incoming link_id or an entry from indices, both validated.
            links.get_unchecked(target_index)
        };
        let (c, a) = if target.a.id == node_id {
            (target.a.point, target.b.point)
        } else {
            (target.b.point, target.a.point)
        };

        // more the one vectors
        let b = unsafe {
            // SAFETY: first_index originates from indices, so it is within links.
            links.get_unchecked(first_index)
        }
        .other(node_id)
        .point;
        let mut vector_solver = NearestVector::new(c, a, b, first_index, clockwise);

        // add second vector
        vector_solver.add(
            unsafe {
                // SAFETY: second_index comes from indices just like first_index.
                links.get_unchecked(second_index)
            }
            .other(node_id)
            .point,
            second_index,
        );

        // check the rest vectors
        for &link_index in indices.iter().skip(pos + 1) {
            if visited.is_not_visited(link_index) {
                let p = unsafe {
                    // SAFETY: every link_index here is sourced from indices, so it addresses links.
                    links.get_unchecked(link_index)
                }
                .other(node_id)
                .point;
                vector_solver.add(p, link_index);
            }
        }

        vector_solver.best_id
    }
}

#[cfg(test)]
mod tag_distribution_tests {
    //! Direct unit tests for the hole-tag distribution code paths,
    //! constructed at the trait-impl level so they don't depend on
    //! graph-traversal ordering.
    //!
    //! These tests would have caught the multi-shape mis-pairing bug
    //! that escaped `tests/hole_tag_mapping_tests.rs` — the
    //! end-to-end tests can't easily force `parent_for_child` to
    //! reorder holes across shapes because the graph traversal
    //! naturally hits shapes and their interior holes in spatial
    //! left-top order.
    //!
    //! The old, buggy insertion-order distribution is no longer part
    //! of the production trait surface; its exact logic is preserved
    //! here as the `old_distribute_hole_tags` reference fn purely so
    //! these tests can demonstrate that the new
    //! `VecTagCollector::distribute_hole_tags_mapped` fixes it on
    //! hand-crafted inputs that exercise the reordering path.
    use super::{TagCollector, VecTagCollector};
    use crate::segm::segment::TagPair;
    use alloc::vec;
    use alloc::vec::Vec;
    use i_float::int::point::IntPoint;
    use i_shape::int::shape::{IntShape, IntShapes};

    /// Build a placeholder `IntShape` (one ring of `vertex_count`
    /// distinct points) at the given origin. The exact geometry
    /// doesn't matter for these tests — only `ring.len()` is
    /// asserted against tag-array length.
    fn square_shape(origin: IntPoint, side: i32, vertex_count: usize) -> IntShape {
        assert!(vertex_count >= 3, "need at least 3 verts to be a polygon");
        let mut ring = Vec::with_capacity(vertex_count);
        for i in 0..vertex_count {
            let t = i as i32;
            let n = vertex_count as i32;
            ring.push(IntPoint::new(
                origin.x + (t * side) / n,
                origin.y + (t * side) / n,
            ));
        }
        vec![ring]
    }

    /// Build a synthetic hole-tag array of `len` zero pairs — only
    /// the LENGTH matters for these tests.
    fn tag_array(len: usize) -> Vec<TagPair> {
        vec![[0u32, 0u32]; len]
    }

    /// Verbatim copy of the *old, buggy* `VecTagCollector::distribute_hole_tags`
    /// that was removed from the production trait. Preserved here only
    /// as a reference oracle so the tests below can prove the new
    /// mapping-aware path fixes the insertion-order mis-pairing.
    /// Do not reintroduce this into `src/`.
    fn old_distribute_hole_tags(
        shape_tags: &mut [Vec<Vec<TagPair>>],
        hole_tags: Vec<Vec<TagPair>>,
        shapes: &IntShapes,
        shape_count_before: usize,
    ) {
        if shape_count_before == 1 && !hole_tags.is_empty() {
            shape_tags[0].extend(hole_tags);
            return;
        }
        let mut hole_idx = 0;
        for (si, shape) in shapes.iter().enumerate() {
            let existing_tag_count = shape_tags.get(si).map_or(0, |t| t.len());
            while existing_tag_count + usize::from(hole_idx < hole_tags.len()) <= shape.len()
                && hole_idx < hole_tags.len()
                && shape_tags.get(si).map_or(0, |t| t.len()) < shape.len()
            {
                if si < shape_tags.len() {
                    shape_tags[si].push(hole_tags[hole_idx].clone());
                }
                hole_idx += 1;
            }
        }
    }

    /// Two shapes, two holes, where insertion order == spatial order.
    /// Both `distribute_hole_tags` (old) and `distribute_hole_tags_mapped`
    /// (new) must produce the same result.
    #[test]
    fn distribute_mapped_matches_old_when_insertion_equals_spatial() {
        let shape0 = square_shape(IntPoint::new(0, 0), 100, 4);
        let shape1 = square_shape(IntPoint::new(200, 0), 100, 4);
        let mut shapes: IntShapes = vec![shape0, shape1];
        // Pretend the binder added one hole to each shape.
        shapes[0].push(vec![IntPoint::new(10, 10); 4]);
        shapes[1].push(vec![IntPoint::new(210, 10); 5]);

        // Insertion-order hole tags, lengths matching the hole rings
        // pushed above.
        let hole_tags: Vec<Vec<TagPair>> = vec![tag_array(4), tag_array(5)];

        // Initial shape_tags: one outer entry per shape.
        let mut shape_tags_old: Vec<Vec<Vec<TagPair>>> = vec![
            vec![tag_array(shapes[0][0].len())],
            vec![tag_array(shapes[1][0].len())],
        ];
        let mut shape_tags_new = shape_tags_old.clone();

        // shape_count_before for the OLD signature.
        let shape_count_before = 2;
        old_distribute_hole_tags(
            &mut shape_tags_old,
            hole_tags.clone(),
            &shapes,
            shape_count_before,
        );
        VecTagCollector::distribute_hole_tags_mapped(&mut shape_tags_new, hole_tags, Some(&[0, 1]));

        assert_eq!(
            shape_tags_old, shape_tags_new,
            "in-order insertion must agree between old and new APIs"
        );

        // Both must keep per-ring lengths consistent.
        for (si, shape) in shapes.iter().enumerate() {
            for (ri, ring) in shape.iter().enumerate() {
                assert_eq!(
                    shape_tags_new[si][ri].len(),
                    ring.len(),
                    "shape[{si}].ring[{ri}] length mismatch in NEW"
                );
            }
        }
    }

    /// **The critical test.** Insertion order REVERSED relative to
    /// spatial assignment: hole 0 spatially belongs to shape 1, hole
    /// 1 spatially belongs to shape 0. With different vertex counts,
    /// the OLD `distribute_hole_tags` mis-pairs tags so that the
    /// per-ring length invariant breaks. The NEW
    /// `distribute_hole_tags_mapped` uses the explicit mapping and
    /// preserves the invariant.
    #[test]
    fn distribute_old_corrupts_when_parent_for_child_is_reordered_new_does_not() {
        let shape0 = square_shape(IntPoint::new(0, 0), 100, 4);
        let shape1 = square_shape(IntPoint::new(200, 0), 100, 4);
        let mut shapes: IntShapes = vec![shape0, shape1];

        // Spatial layout chosen by the binder: hole 1 (5 verts) →
        // shape 0; hole 0 (4 verts) → shape 1.
        // The binder pushes each hole to its enclosing shape, so
        // `shape[0]` gets hole 1 and `shape[1]` gets hole 0.
        shapes[0].push(vec![IntPoint::new(50, 50); 5]); // hole 1's 5 verts
        shapes[1].push(vec![IntPoint::new(250, 50); 4]); // hole 0's 4 verts

        // Insertion order of hole tags: [hole_0=4 tags, hole_1=5 tags].
        let hole_tags: Vec<Vec<TagPair>> = vec![tag_array(4), tag_array(5)];

        let mut shape_tags_old: Vec<Vec<Vec<TagPair>>> = vec![vec![tag_array(4)], vec![tag_array(4)]];
        let mut shape_tags_new = shape_tags_old.clone();

        old_distribute_hole_tags(
            &mut shape_tags_old,
            hole_tags.clone(),
            &shapes,
            /* shape_count_before */ 2,
        );
        // Correct mapping: hole 0 → shape 1, hole 1 → shape 0.
        VecTagCollector::distribute_hole_tags_mapped(&mut shape_tags_new, hole_tags, Some(&[1, 0]));

        // OLD path: walks shapes in order, pushes hole_tags in
        // insertion order → shape 0 gets hole 0 (4 tags) but its
        // ring has 5 points. MISMATCH.
        assert_eq!(
            shape_tags_old[0][1].len(),
            4,
            "OLD pushed hole_tags[0] (4 elements) onto shape 0"
        );
        assert_eq!(shapes[0][1].len(), 5, "but shape 0's hole ring has 5 points");
        assert_ne!(
            shape_tags_old[0][1].len(),
            shapes[0][1].len(),
            "OLD distribution MUST corrupt shape[0].ring[1] length \
             when parent_for_child reorders — if this passes the test \
             is no longer exercising the bug",
        );

        // NEW path: hole 1 (5 tags) lands on shape 0 (5-point ring),
        // hole 0 (4 tags) lands on shape 1 (4-point ring). Per-ring
        // lengths agree.
        for (si, shape) in shapes.iter().enumerate() {
            for (ri, ring) in shape.iter().enumerate() {
                assert_eq!(
                    shape_tags_new[si][ri].len(),
                    ring.len(),
                    "NEW: shape[{si}].ring[{ri}] points={} tags={} — \
                     mapping-aware distribution must keep lockstep",
                    ring.len(),
                    shape_tags_new[si][ri].len()
                );
            }
        }
    }

    /// `distribute_hole_tags_mapped` with `None` mapping is a no-op
    /// (the no-holes case).
    #[test]
    fn distribute_mapped_none_is_noop() {
        let mut shape_tags: Vec<Vec<Vec<TagPair>>> = vec![vec![tag_array(4)]];
        let original = shape_tags.clone();
        VecTagCollector::distribute_hole_tags_mapped(&mut shape_tags, vec![], None);
        assert_eq!(shape_tags, original);
    }

    /// Mapping length must equal `hole_tags` length; mismatch panics.
    #[test]
    #[should_panic(expected = "mapping len")]
    fn distribute_mapped_panics_on_mapping_hole_tag_length_mismatch() {
        let mut shape_tags: Vec<Vec<Vec<TagPair>>> = vec![vec![tag_array(4)]];
        let hole_tags: Vec<Vec<TagPair>> = vec![tag_array(3), tag_array(3)];
        VecTagCollector::distribute_hole_tags_mapped(&mut shape_tags, hole_tags, Some(&[0]));
    }

    /// Mapping that references an out-of-bounds shape panics with a
    /// clear message — the binder must never lie about how many
    /// shapes exist.
    #[test]
    #[should_panic(expected = "but only")]
    fn distribute_mapped_panics_on_out_of_range_shape_index() {
        let mut shape_tags: Vec<Vec<Vec<TagPair>>> = vec![vec![tag_array(4)]];
        let hole_tags: Vec<Vec<TagPair>> = vec![tag_array(3)];
        VecTagCollector::distribute_hole_tags_mapped(&mut shape_tags, hole_tags, Some(&[1]));
    }

    /// Three shapes, three holes, with `parent_for_child` permuting
    /// them across shapes [2, 0, 1] and DIFFERENT vertex counts —
    /// the most adversarial layout. Verifies that all per-ring
    /// length invariants hold for the NEW path AND that the OLD
    /// path measurably corrupts at least one ring length.
    #[test]
    fn distribute_three_shape_full_permutation_round_trip() {
        let mut shapes: IntShapes = vec![
            square_shape(IntPoint::new(0, 0), 100, 4),
            square_shape(IntPoint::new(200, 0), 100, 4),
            square_shape(IntPoint::new(400, 0), 100, 4),
        ];
        // Spatial assignment: hole[0]→shape[2] (6 verts),
        // hole[1]→shape[0] (5 verts), hole[2]→shape[1] (7 verts).
        // Hole ring pushed into each shape's contour list:
        shapes[2].push(vec![IntPoint::new(450, 50); 6]); // hole 0
        shapes[0].push(vec![IntPoint::new(50, 50); 5]); // hole 1
        shapes[1].push(vec![IntPoint::new(250, 50); 7]); // hole 2

        // Insertion-order tag arrays.
        let hole_tags: Vec<Vec<TagPair>> = vec![
            tag_array(6), // hole 0 → shape 2
            tag_array(5), // hole 1 → shape 0
            tag_array(7), // hole 2 → shape 1
        ];

        let mut shape_tags_old: Vec<Vec<Vec<TagPair>>> =
            vec![vec![tag_array(4)], vec![tag_array(4)], vec![tag_array(4)]];
        let mut shape_tags_new = shape_tags_old.clone();

        old_distribute_hole_tags(
            &mut shape_tags_old,
            hole_tags.clone(),
            &shapes,
            /* shape_count_before */ 3,
        );
        VecTagCollector::distribute_hole_tags_mapped(&mut shape_tags_new, hole_tags, Some(&[2, 0, 1]));

        // OLD path corrupts at least one ring (shape 0 gets hole_tags[0]
        // which has 6 entries onto a 5-point ring → 6 vs 5 mismatch).
        let mut found_corruption = false;
        for (si, shape) in shapes.iter().enumerate() {
            for (ri, ring) in shape.iter().enumerate() {
                if shape_tags_old[si][ri].len() != ring.len() {
                    found_corruption = true;
                    break;
                }
            }
        }
        assert!(
            found_corruption,
            "OLD distribution MUST corrupt at least one ring on a full \
             3-shape permutation with mismatched vertex counts"
        );

        // NEW path: every ring's length matches its tag array.
        for (si, shape) in shapes.iter().enumerate() {
            for (ri, ring) in shape.iter().enumerate() {
                assert_eq!(
                    shape_tags_new[si][ri].len(),
                    ring.len(),
                    "NEW: shape[{si}].ring[{ri}] points={} tags={}",
                    ring.len(),
                    shape_tags_new[si][ri].len()
                );
            }
        }
    }

    /// One shape with many holes — exercises the early-return inside
    /// `join_sorted_holes_with_mapping` indirectly (here we just
    /// pass the resulting `[0; N]` mapping). Lengths must hold for
    /// arbitrary N.
    #[test]
    fn distribute_mapped_single_shape_with_many_holes() {
        let mut shapes: IntShapes = vec![square_shape(IntPoint::new(0, 0), 100, 4)];
        let n_holes = 20;
        let hole_lengths: Vec<usize> = (3..3 + n_holes).collect();
        for &len in &hole_lengths {
            shapes[0].push(vec![IntPoint::new(50, 50); len]);
        }
        let hole_tags: Vec<Vec<TagPair>> = hole_lengths.iter().map(|&l| tag_array(l)).collect();
        let mapping: Vec<usize> = vec![0; n_holes];

        let mut shape_tags: Vec<Vec<Vec<TagPair>>> = vec![vec![tag_array(4)]];
        VecTagCollector::distribute_hole_tags_mapped(&mut shape_tags, hole_tags, Some(&mapping));

        assert_eq!(shape_tags[0].len(), shapes[0].len());
        for (ri, ring) in shapes[0].iter().enumerate() {
            assert_eq!(shape_tags[0][ri].len(), ring.len());
        }
    }

    /// No-holes case with multiple shapes — the mapping is empty;
    /// shape_tags must stay exactly as initialized (one outer per
    /// shape).
    #[test]
    fn distribute_mapped_no_holes_with_multiple_shapes_is_noop() {
        let shapes: IntShapes = vec![
            square_shape(IntPoint::new(0, 0), 100, 4),
            square_shape(IntPoint::new(200, 0), 100, 4),
            square_shape(IntPoint::new(400, 0), 100, 4),
        ];
        let mut shape_tags: Vec<Vec<Vec<TagPair>>> =
            vec![vec![tag_array(4)], vec![tag_array(4)], vec![tag_array(4)]];
        let snapshot = shape_tags.clone();
        VecTagCollector::distribute_hole_tags_mapped(&mut shape_tags, vec![], Some(&[]));
        assert_eq!(shape_tags, snapshot, "no holes → no changes");
        // And per-ring lengths still align with the (untouched) shapes.
        for (si, shape) in shapes.iter().enumerate() {
            for (ri, ring) in shape.iter().enumerate() {
                assert_eq!(shape_tags[si][ri].len(), ring.len());
            }
        }
    }

    // --- G4: the demoted structural guards must still fire in debug
    // builds (`cargo test` is debug, so `debug_assert_eq!` is live).
    // These prove the guards are real, not dead code, and pin their
    // panic messages.

    #[test]
    #[should_panic(expected = "shape-count mismatch")]
    fn assert_consistent_panics_on_shape_count_mismatch() {
        let shapes: IntShapes = vec![
            square_shape(IntPoint::new(0, 0), 100, 4),
            square_shape(IntPoint::new(200, 0), 100, 4),
        ];
        // Only one tag array for two shapes.
        let shape_tags: Vec<Vec<Vec<TagPair>>> = vec![vec![tag_array(4)]];
        VecTagCollector::assert_consistent(&shape_tags, &shapes);
    }

    #[test]
    #[should_panic(expected = "edge count")]
    fn assert_consistent_panics_on_per_ring_edge_mismatch() {
        let shapes: IntShapes = vec![square_shape(IntPoint::new(0, 0), 100, 4)];
        // Ring has 4 points but its tag array has 3 — the exact
        // corruption the whole change exists to prevent.
        let shape_tags: Vec<Vec<Vec<TagPair>>> = vec![vec![tag_array(3)]];
        VecTagCollector::assert_consistent(&shape_tags, &shapes);
    }

    #[test]
    #[should_panic(expected = "hole[0]")]
    fn assert_holes_lockstep_panics_on_per_hole_length_mismatch() {
        // One hole ring of 5 points, but its tag array has 4.
        let holes = vec![vec![IntPoint::new(0, 0); 5]];
        let hole_tags: Vec<Vec<TagPair>> = vec![tag_array(4)];
        VecTagCollector::assert_holes_lockstep(&hole_tags, &holes);
    }
}

#[cfg(test)]
mod tests {
    use crate::core::fill_rule::FillRule;
    use crate::core::overlay::{ContourDirection, Overlay};
    use crate::core::overlay_rule::OverlayRule;
    use i_shape::int_shape;

    #[test]
    fn test_0() {
        #[rustfmt::skip]
        let subj = int_shape![
            [[0, 0], [4, 0], [4, 4], [0, 4]],
            [[1, 1], [1, 3], [3, 3], [3, 1]],
        ];

        let mut buffer = Default::default();
        let mut overlay = Overlay::with_contours(&subj, &[]);

        let shapes_0 = overlay
            .build_graph_view(FillRule::NonZero)
            .unwrap()
            .extract_shapes(OverlayRule::Subject, &mut buffer);

        debug_assert!(shapes_0.len() == 1);

        overlay.options.output_direction = ContourDirection::Clockwise;
        let shapes_1 = overlay
            .build_graph_view(FillRule::NonZero)
            .unwrap()
            .extract_shapes(OverlayRule::Subject, &mut buffer);

        debug_assert!(shapes_1.len() == 1);
    }
}
