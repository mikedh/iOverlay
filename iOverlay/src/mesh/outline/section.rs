use crate::mesh::math::Math;
use crate::mesh::outline::uniq_iter::UniqueSegment;
use crate::segm::boolean::ShapeCountBoolean;
use crate::segm::segment::Segment;
use core::marker::PhantomData;
use i_float::adapter::FloatPointAdapter;
use i_float::float::compatible::FloatPointCompatible;
use i_float::float::number::FloatNumber;
use i_float::float::vector::FloatPointMath;
use i_float::int::point::IntPoint;

#[derive(Debug, Clone)]
pub(crate) struct OffsetSection<P: FloatPointCompatible<T>, T: FloatNumber> {
    pub(crate) a: IntPoint,
    pub(crate) b: IntPoint,
    pub(crate) a_top: IntPoint,
    pub(crate) b_top: IntPoint,
    pub(crate) dir: P,
    pub(crate) _phantom: PhantomData<T>,
}

impl<P: FloatPointCompatible<T>, T: FloatNumber> OffsetSection<P, T> {
    #[inline]
    pub(crate) fn new(
        radius: T,
        s: &UniqueSegment,
        adapter: &FloatPointAdapter<P, T>,
    ) -> Self {
        let a = adapter.int_to_float(&s.a);
        let b = adapter.int_to_float(&s.b);
        let ab = FloatPointMath::sub(&b, &a);
        let dir = FloatPointMath::normalize(&ab);
        let t = Math::ortho_and_scale(&dir, radius);

        let at = FloatPointMath::add(&a, &t);
        let bt = FloatPointMath::add(&b, &t);
        let a_top = adapter.float_to_int(&at);
        let b_top = adapter.float_to_int(&bt);

        Self {
            a: s.a,
            b: s.b,
            a_top,
            b_top,
            dir,
            _phantom: Default::default(),
        }
    }

    #[inline]
    pub(crate) fn top_segment(&self) -> Option<Segment<ShapeCountBoolean>> {
        if self.a_top != self.b_top {
            Some(Segment::subject(self.a_top, self.b_top))
        } else {
            None
        }
    }

    #[inline]
    pub(crate) fn a_segment(&self) -> Option<Segment<ShapeCountBoolean>> {
        if self.a_top != self.a {
            Some(Segment::subject(self.a, self.a_top))
        } else {
            None
        }
    }

    #[inline]
    pub(crate) fn b_segment(&self) -> Option<Segment<ShapeCountBoolean>> {
        if self.b_top != self.b {
            Some(Segment::subject(self.b_top, self.b))
        } else {
            None
        }
    }
}
