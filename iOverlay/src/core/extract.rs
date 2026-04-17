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
            let (is_valid, is_modified) = buffer.points.validate(
                self.options.min_output_area,
                self.options.preserve_output_collinear,
            );

            if !is_valid {
                link_index += 1;
                continue;
            }

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
                holes.push(contour);
                TC::push_hole(&mut hole_tags, &buffer.tags);
            } else {
                shapes.push(vec![contour]);
                TC::push_shape(&mut shape_tags, &buffer.tags);
            }
        }

        if !anchors_already_sorted {
            anchors.sort_unstable_by(|s0, s1| s0.v_segment.a.cmp(&s1.v_segment.a));
        }

        let shape_count_before = shapes.len();
        shapes.join_sorted_holes(holes, anchors, clockwise);

        TC::distribute_hole_tags(&mut shape_tags, hole_tags, &shapes, shape_count_before);

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
    /// [`distribute_hole_tags`] folds them back into shapes.
    type HoleTags: Default;

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
    /// boundary). Holes are spliced in later via `distribute_hole_tags`.
    fn push_shape(shape_tags: &mut Self::ShapeTags, buffer_tags: &[TagPair]);

    /// After `shapes.join_sorted_holes` has folded the hole contours
    /// into the correct shapes, mirror that distribution on the tag
    /// accumulator so that `shape_tags[i]` lines up 1-to-1 with
    /// `shapes[i]`. `NoOpTagCollector` is a no-op.
    fn distribute_hole_tags(
        shape_tags: &mut Self::ShapeTags,
        hole_tags: Self::HoleTags,
        shapes: &IntShapes,
        shape_count_before: usize,
    );
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
    fn distribute_hole_tags(_: &mut (), _: (), _: &IntShapes, _: usize) {}
}

/// Collects per-edge tags as `Vec<Vec<Vec<TagPair>>>` (shape → contour →
/// edge-tag). Matches the shape of `IntShapes` 1-to-1.
pub(crate) struct VecTagCollector;

impl TagCollector for VecTagCollector {
    type ShapeTags = Vec<Vec<Vec<TagPair>>>;
    type HoleTags = Vec<Vec<TagPair>>;

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

    fn distribute_hole_tags(
        shape_tags: &mut Self::ShapeTags,
        hole_tags: Self::HoleTags,
        shapes: &IntShapes,
        shape_count_before: usize,
    ) {
        // Single-shape output (the common buffer/offset case): every
        // hole belongs to shapes[0].
        if shape_count_before == 1 && !hole_tags.is_empty() {
            shape_tags[0].extend(hole_tags);
            return;
        }

        // Multi-shape: walk shapes in order, attaching hole_tags to
        // whichever shape has gained contours since join_sorted_holes.
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
