use core::iter::Chain;
use i_float::int::point::IntPoint;

pub(super) struct UniqueSegment {
    pub(super) a: IntPoint,
    pub(super) b: IntPoint,
    /// Index of the FIRST input edge contributing to this unique
    /// segment. Equal to the `tangent_start` lookup index for the
    /// segment's `a` endpoint.
    pub(super) edge_index: usize,
    /// Index of the LAST input edge contributing to this unique
    /// segment. Equals `edge_index` when no collinear merging
    /// occurred. Equal to the `tangent_end` lookup index for the
    /// segment's `b` endpoint.
    pub(super) last_edge_index: usize,
}

pub(super) struct UniqueSegmentsIter<I>
where
    I: Iterator<Item = (IntPoint, usize)>,
{
    iter: Chain<I, core::array::IntoIter<(IntPoint, usize), 2>>,
    p0: IntPoint,
    i0: usize,
    p1: IntPoint,
    i1: usize,
    /// Total input path length, in vertices. Used to convert
    /// `i1` (path index of `p1`) into the edge index of the last
    /// edge contained in the current run via `(i1 + n - 1) % n`.
    path_len: usize,
}

impl<I> UniqueSegmentsIter<I>
where
    I: Iterator<Item = (IntPoint, usize)>,
{
    #[inline]
    pub(super) fn new(iter: I, path_len: usize) -> Option<Self> {
        let mut iter = iter;

        let (mut p0, mut i0) = iter.next()?;
        let (mut p1, mut i1) = iter.find(|(p, _)| p0.ne(p))?;
        // Once we've consumed at least two distinct points, the
        // path the caller is iterating over must contain at least
        // those two — so `path_len > 0`. `last_edge_index`
        // computation below depends on this (it does `% path_len`).
        debug_assert!(
            path_len > 0,
            "UniqueSegmentsIter::new: non-empty iterator implies path_len > 0",
        );

        let (q0, qi0) = (p0, i0);

        for (p2, idx2) in &mut iter {
            if include_point(p0, p1, p2) {
                p0 = p1;
                i0 = i1;
                p1 = p2;
                i1 = idx2;
                break;
            }
            p1 = p2;
            i1 = idx2;
        }

        let (q1, qi1) = (p0, i0);

        let chain_iter = iter.chain([(q0, qi0), (q1, qi1)]);

        Some(Self {
            iter: chain_iter,
            p0,
            i0,
            p1,
            i1,
            path_len,
        })
    }
}

impl<I> Iterator for UniqueSegmentsIter<I>
where
    I: Iterator<Item = (IntPoint, usize)>,
{
    type Item = UniqueSegment;
    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        // Edge index of the last edge fully contained in the current
        // run is `(i1 + path_len - 1) % path_len` — i.e., the edge
        // ending at `p1`. This is the canonical lookup index for the
        // analytical tangent at the segment's `b` endpoint.
        let last_edge = |i1: usize| (i1 + self.path_len - 1) % self.path_len;

        for (p2, idx2) in &mut self.iter {
            if !include_point(self.p0, self.p1, p2) {
                self.p1 = p2;
                self.i1 = idx2;
                continue;
            }
            let s = UniqueSegment {
                a: self.p0,
                b: self.p1,
                edge_index: self.i0,
                last_edge_index: last_edge(self.i1),
            };

            self.p0 = self.p1;
            self.i0 = self.i1;
            self.p1 = p2;
            self.i1 = idx2;

            return Some(s);
        }

        let add_last = self.p1 != self.p0;
        if add_last {
            let s = UniqueSegment {
                a: self.p0,
                b: self.p1,
                edge_index: self.i0,
                last_edge_index: last_edge(self.i1),
            };
            self.p1 = self.p0;
            Some(s)
        } else {
            None
        }
    }
}

#[inline]
fn include_point(p0: IntPoint, p1: IntPoint, p2: IntPoint) -> bool {
    let a = p1.subtract(p0);
    let b = p1.subtract(p2);

    if a.cross_product(b) != 0 {
        // not collinear
        return true;
    }

    // collinear – keep only if we keep going opposite direction
    a.dot_product(b) > 0
}
#[cfg(test)]
mod tests {
    use crate::mesh::outline::uniq_iter::{UniqueSegment, UniqueSegmentsIter};
    use alloc::vec::Vec;
    use i_float::int::point::IntPoint;
    use i_shape::int_path;

    fn indexed(path: &[IntPoint]) -> impl Iterator<Item = (IntPoint, usize)> + '_ {
        path.iter().copied().enumerate().map(|(i, p)| (p, i))
    }

    #[test]
    fn test_empty() {
        let uniq_iter = UniqueSegmentsIter::new(core::iter::empty::<(IntPoint, usize)>(), 0);
        assert!(uniq_iter.is_none());
    }

    #[test]
    fn test_single_point() {
        let path = int_path![[0, 0]];
        let uniq_iter = UniqueSegmentsIter::new(indexed(&path), path.len());
        assert!(uniq_iter.is_none());
    }

    #[test]
    fn test_all_points_equal() {
        let path = int_path![[0, 0], [0, 0], [0, 0]];
        let uniq_iter = UniqueSegmentsIter::new(indexed(&path), path.len());
        assert!(uniq_iter.is_none());
    }

    #[test]
    fn test_line_0() {
        let path = int_path![[0, 0], [10, 0]];
        validate_case_all_rotations(&path, 2);
    }

    #[test]
    fn test_line_1() {
        let path = int_path![[0, 0], [5, 0], [10, 0]];
        validate_case_all_rotations(&path, 2);
    }

    #[test]
    fn test_line_2() {
        let path = int_path![[0, 0], [5, 0], [10, 0], [5, 0]];
        validate_case_all_rotations(&path, 2);
    }

    #[test]
    fn test_square_0() {
        let path = int_path![[0, 10], [0, 0], [10, 0], [10, 10]];
        validate_case_all_rotations(&path, 4);
    }

    #[test]
    fn test_square_1() {
        #[rustfmt::skip]
        let path = int_path![[0, 10], [0, 5], [0, 0], [5, 0], [10, 0], [10, 5], [10, 10], [5, 10]];
        validate_case_all_rotations(&path, 4);
    }

    #[test]
    fn test_square_2() {
        #[rustfmt::skip]
        let path = int_path![
            [0, 10], [0, 8], [0, 5], [0, 2],
            [0, 0], [2, 0], [5, 0], [8, 0],
            [10, 0], [10, 2], [10, 5], [10, 8],
            [10, 10], [8, 10], [5, 10], [2, 10]
        ];
        validate_case_all_rotations(&path, 4);
    }

    fn validate_case_all_rotations(path: &[IntPoint], expected_segments_count: usize) {
        assert!(!path.is_empty(), "path must not be empty");

        for shift in 0..path.len() {
            let n = path.len();
            let uniq_iter = UniqueSegmentsIter::new(
                path[shift..]
                    .iter()
                    .chain(path[..shift].iter())
                    .copied()
                    .enumerate()
                    .map(|(i, p)| (p, (i + shift) % n)),
                n,
            )
            .unwrap();

            let segments: Vec<_> = uniq_iter.collect();

            assert_eq!(
                segments.len(),
                expected_segments_count,
                "unexpected segment count for shift {}",
                shift
            );

            validate_segments(&segments);
        }
    }

    fn validate_segments(segments: &[UniqueSegment]) {
        assert!(!segments.is_empty(), "expected at least one segment");

        for (i, s) in segments.iter().enumerate() {
            assert_ne!(s.a, s.b, "segment {} is degenerate (a == b)", i);
        }

        for (i, w) in segments.windows(2).enumerate() {
            let s0 = &w[0];
            let s1 = &w[1];

            validate_pair(s0, s1, i, i + 1);
        }

        let last_i = segments.len() - 1;
        validate_pair(&segments[last_i], &segments[0], last_i, 0);
    }

    fn validate_pair(s0: &UniqueSegment, s1: &UniqueSegment, i: usize, j: usize) {
        assert_eq!(s0.b, s1.a, "segment {} end does not match segment {} start", i, j);

        let v0 = s0.a - s0.b;
        let v1 = s1.a - s1.b;

        let cross = v0.cross_product(v1);
        if cross == 0 {
            let dot = v0.dot_product(v1);
            assert!(
                dot < 0,
                "segments {} and {} are collinear and same direction",
                i,
                j
            );
        }
    }
}
