use crate::geom::x_segment::XSegment;
use crate::segm::boolean::ShapeCountBoolean;
use crate::segm::segment::{Segment, UNTAGGED};
use i_float::int::point::IntPoint;

impl Segment<ShapeCountBoolean> {
    #[inline]
    pub(crate) fn subject(p0: IntPoint, p1: IntPoint) -> Self {
        if p0 < p1 {
            Self {
                x_segment: XSegment { a: p0, b: p1 },
                count: ShapeCountBoolean { subj: 1, clip: 0 },
                tag: UNTAGGED,
            }
        } else {
            Self {
                x_segment: XSegment { a: p1, b: p0 },
                count: ShapeCountBoolean { subj: -1, clip: 0 },
                tag: UNTAGGED,
            }
        }
    }
}
