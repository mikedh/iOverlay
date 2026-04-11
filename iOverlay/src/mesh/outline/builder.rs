use crate::core::overlay::Overlay;
use crate::mesh::outline::builder_join::JoinBuilder;
use crate::mesh::outline::builder_join::{BevelJoinBuilder, MiterJoinBuilder, RoundJoinBuilder};
use crate::mesh::outline::section::OffsetSection;
use crate::mesh::outline::uniq_iter::UniqueSegmentsIter;
use crate::mesh::style::LineJoin;
use crate::segm::boolean::ShapeCountBoolean;
use crate::segm::segment::Segment;
use crate::tagged::TagLookup;
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::marker::PhantomData;
use i_float::adapter::FloatPointAdapter;
use i_float::float::compatible::FloatPointCompatible;
use i_float::float::number::FloatNumber;

pub(crate) trait OutlineBuild<P: FloatPointCompatible<T>, T: FloatNumber> {
    fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    );

    /// Tagged variant of `build`. Delegates to
    /// [`crate::tagged::build_outline_tagged`], passing the concrete
    /// `J: JoinBuilder` the caller picked in `OutlineBuilder::new`.
    fn build_tagged(
        &self,
        path: &[P],
        tags: &[u32],
        adapter: &FloatPointAdapter<P, T>,
        overlay: &mut Overlay,
        lookup: &mut TagLookup,
    );

    fn capacity(&self, points_count: usize) -> usize;
    fn additional_offset(&self, radius: T) -> T;
}

pub struct OutlineBuilder<P: FloatPointCompatible<T>, T: FloatNumber> {
    builder: Box<dyn OutlineBuild<P, T>>,
}

pub(crate) struct Builder<J: JoinBuilder<P, T>, P: FloatPointCompatible<T>, T: FloatNumber> {
    pub(crate) extend: bool,
    pub(crate) radius: T,
    pub(crate) join_builder: J,
    pub(crate) _phantom: PhantomData<P>,
}

impl<P: FloatPointCompatible<T> + 'static, T: FloatNumber + 'static> OutlineBuilder<P, T> {
    pub fn new(radius: T, join: &LineJoin<T>) -> OutlineBuilder<P, T> {
        let extend = radius > T::from_float(0.0);
        let builder: Box<dyn OutlineBuild<P, T>> = {
            match join {
                LineJoin::Miter(ratio) => Box::new(Builder {
                    extend,
                    radius,
                    join_builder: MiterJoinBuilder::new(*ratio, radius),
                    _phantom: Default::default(),
                }),
                LineJoin::Round(ratio) => Box::new(Builder {
                    extend,
                    radius,
                    join_builder: RoundJoinBuilder::new(*ratio, radius),
                    _phantom: Default::default(),
                }),
                LineJoin::Bevel => Box::new(Builder {
                    extend,
                    radius,
                    join_builder: BevelJoinBuilder {},
                    _phantom: Default::default(),
                }),
            }
        };

        Self { builder }
    }

    #[inline]
    pub(crate) fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        self.builder.build(path, adapter, segments);
    }

    /// Tagged outline: `tags.len()` must equal `path.len()`. Edge `i`
    /// — from `path[i]` to `path[(i + 1) % n]` — carries `tags[i]`.
    /// Offset segments emitted for that edge inherit the same tag;
    /// round-join arc chords at sharp convex corners carry
    /// [`crate::tagged::SYNTHESIZED_ROUND_JOIN_TAG`]. Pass the
    /// resulting `overlay` to a standard run with
    /// `options.preserve_output_collinear = true`, then annotate each
    /// output contour via [`TagLookup::annotate_contour`].
    ///
    /// All walking / join logic lives in [`crate::tagged`]; this is
    /// just the dyn-trait entry point.
    #[inline]
    pub fn build_tagged_into(
        &self,
        path: &[P],
        tags: &[u32],
        adapter: &FloatPointAdapter<P, T>,
        overlay: &mut Overlay,
        lookup: &mut TagLookup,
    ) {
        self.builder.build_tagged(path, tags, adapter, overlay, lookup);
    }

    #[inline]
    pub fn capacity(&self, points_count: usize) -> usize {
        self.builder.capacity(points_count)
    }

    #[inline]
    pub fn additional_offset(&self, radius: T) -> T {
        self.builder.additional_offset(radius)
    }
}

impl<J: JoinBuilder<P, T>, P: FloatPointCompatible<T>, T: FloatNumber> OutlineBuild<P, T>
    for Builder<J, P, T>
{
    #[inline]
    fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        if path.len() < 2 {
            return;
        }

        self.build(path, adapter, segments);
    }

    #[inline]
    fn build_tagged(
        &self,
        path: &[P],
        tags: &[u32],
        adapter: &FloatPointAdapter<P, T>,
        overlay: &mut Overlay,
        lookup: &mut TagLookup,
    ) {
        crate::tagged::build_outline_tagged(self, path, tags, adapter, overlay, lookup);
    }

    #[inline]
    fn capacity(&self, points_count: usize) -> usize {
        self.join_builder.capacity() * points_count
    }

    #[inline]
    fn additional_offset(&self, radius: T) -> T {
        self.join_builder.additional_offset(radius)
    }
}

impl<J: JoinBuilder<P, T>, P: FloatPointCompatible<T>, T: FloatNumber> Builder<J, P, T> {
    fn build(
        &self,
        path: &[P],
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        let iter = path.iter().map(|p| adapter.float_to_int(p));
        let mut uniq_segments = if let Some(iter) = UniqueSegmentsIter::new(iter) {
            iter
        } else {
            return;
        };

        let us0 = if let Some(us) = uniq_segments.next() {
            us
        } else {
            return;
        };

        let s0 = OffsetSection::new(self.radius, &us0, adapter);
        let mut sk = s0.clone();

        segments.push_some(sk.top_segment());

        for usi in uniq_segments {
            let si = OffsetSection::new(self.radius, &usi, adapter);
            segments.push_some(si.top_segment());
            self.feed_join(&sk, &si, adapter, segments);
            sk = si;
        }
        self.feed_join(&sk, &s0, adapter, segments);
    }

    #[inline]
    fn feed_join(
        &self,
        s0: &OffsetSection<P, T>,
        s1: &OffsetSection<P, T>,
        adapter: &FloatPointAdapter<P, T>,
        segments: &mut Vec<Segment<ShapeCountBoolean>>,
    ) {
        let vi = s1.b - s1.a;
        let vp = s0.b - s0.a;

        let cross = vi.cross_product(vp);
        debug_assert!(cross != 0, "not possible! UniqueSegmentsIter guarantee it");
        let outer_corner = (cross > 0) == self.extend;
        if outer_corner {
            if s0.b_top != s1.a_top {
                self.join_builder.add_join(s0, s1, adapter, segments);
            }
        } else {
            // no join
            segments.push_some(s0.b_segment());
            segments.push_some(s1.a_segment());
        }
    }
}

trait VecPushSome<T> {
    fn push_some(&mut self, value: Option<T>);
}

impl<T> VecPushSome<T> for Vec<T> {
    #[inline]
    fn push_some(&mut self, value: Option<T>) {
        if let Some(v) = value {
            self.push(v);
        }
    }
}
