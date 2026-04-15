use i_float::int::point::IntPoint;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct IdPoint {
    pub id: usize,
    pub point: IntPoint,
}

impl IdPoint {
    pub(crate) fn new(id: usize, point: IntPoint) -> Self {
        Self { id, point }
    }
}
