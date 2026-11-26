//! Bézier paths: a chain of cubic Bézier segments described by its points, each with a handle
//! on either side, as drawing programs edit them.
//!
//! A path is stored as an ordinary cubic [`Spline`] whose inner knots have multiplicity three,
//! so every span is one Bézier segment and the control points are, in order, a point, its
//! outgoing handle, the next point's incoming handle, the next point, and so on (Piegl and
//! Tiller, *The NURBS Book*, 2nd ed., section 5.3 on Bézier decomposition; splitting a segment is
//! de Casteljau's algorithm, section 1.3). Nothing else is stored: a corner is a point whose
//! handles lie on it, and the two handles of a point count as linked while they are in line
//! through it.

use crate::{EPS, Spline, Vec2};

/// A point of a Bézier path with the ends of its two handles (absolute positions). A handle
/// that lies on the point is retracted: the segment on that side leaves the point straight
/// towards the neighbour's handle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BezierPoint {
    pub at: Vec2,
    /// End of the handle on the incoming side.
    pub before: Vec2,
    /// End of the handle on the outgoing side.
    pub after: Vec2,
    /// The handles move independently: moving one never turns the other.
    pub free: bool,
}

impl BezierPoint {
    /// A point with both handles retracted.
    pub fn corner(at: Vec2) -> Self {
        Self { at, before: at, after: at, free: false }
    }

    /// A point whose outgoing handle ends at `after`, with the incoming handle its mirror image.
    pub fn smooth(at: Vec2, after: Vec2) -> Self {
        Self { at, before: at + (at - after), after, free: false }
    }

    fn tol(&self) -> f64 {
        EPS * (1.0 + self.at.len())
    }

    pub fn has_before(&self) -> bool {
        !self.before.near(self.at, self.tol())
    }

    pub fn has_after(&self) -> bool {
        !self.after.near(self.at, self.tol())
    }

    /// Moving one handle turns the other: the point is not marked [`BezierPoint::free`] and
    /// both handles are out and in one line through it, on opposite sides.
    pub fn is_linked(&self) -> bool {
        let (a, b) = (self.before - self.at, self.after - self.at);
        !self.free && self.has_before() && self.has_after() && a.dot(b) < 0.0 && a.cross(b).abs() <= 1e-6 * a.len() * b.len()
    }
}

/// What a control point of a path's spline is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BezierPart {
    Point(usize),
    /// The incoming handle of this point.
    Before(usize),
    /// The outgoing handle of this point.
    After(usize),
}

/// A chain of cubic Bézier segments through `points`; closed, a last segment returns to the
/// first point.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BezierPath {
    pub points: Vec<BezierPoint>,
    pub closed: bool,
}

fn cubic(c: [Vec2; 4], t: f64) -> Vec2 {
    let [a, b, c, d] = c;
    let (ab, bc, cd) = (a.lerp(b, t), b.lerp(c, t), c.lerp(d, t));
    ab.lerp(bc, t).lerp(bc.lerp(cd, t), t)
}

#[expect(clippy::cast_precision_loss, reason = "Point and sample counts are far below 2^53.")]
fn as_f64(n: usize) -> f64 {
    n as f64
}

impl BezierPath {
    /// Number of segments.
    pub fn segments(&self) -> usize {
        match self.points.len() {
            0 | 1 => 0,
            n if self.closed => n,
            n => n - 1,
        }
    }

    /// The four control points of segment `j`.
    fn segment(&self, j: usize) -> Option<[Vec2; 4]> {
        if j >= self.segments() {
            return None;
        }
        let a = self.points.get(j)?;
        let b = self.points.get((j + 1) % self.points.len())?;
        Some([a.at, a.after, b.before, b.at])
    }

    /// The spline that stores the path; `None` with fewer than two points or a point that is
    /// not finite. An open path has no use for its first incoming and last outgoing handle and
    /// does not keep them.
    pub fn to_spline(&self) -> Option<Spline> {
        let segs = self.segments();
        if segs == 0 || self.points.iter().any(|p| !(p.at.is_finite() && p.before.is_finite() && p.after.is_finite())) {
            return None;
        }
        let mut control = Vec::with_capacity(segs * 3 + 1);
        for j in 0..segs {
            let [a, b, c, _] = self.segment(j)?;
            control.extend([a, b, c]);
        }
        control.push(if self.closed { self.points.first()?.at } else { self.points.last()?.at });
        let breaks: Vec<f64> = (0..=segs).map(as_f64).collect();
        let free_points = self.points.iter().enumerate().filter(|(_, p)| p.free).map(|(i, _)| i).collect();
        Some(Spline { degree: 3, knots: crate::nurbs::bezier_knots(&breaks, 3), control, closed: self.closed, free_points, ..Spline::default() })
    }

    /// The path a spline stores, when the spline is one: cubic, not rational, without fit
    /// points, every inner knot three times. A closed one ends on its first point.
    pub fn from_spline(sp: &Spline) -> Option<BezierPath> {
        let c = &sp.control;
        let segs = c.len().checked_sub(1)? / 3;
        if sp.degree != 3 || segs == 0 || c.len() != segs * 3 + 1 || !sp.fit.is_empty() || sp.weights.iter().any(|w| (w - 1.0).abs() > 1e-12) {
            return None;
        }
        if sp.knots.len() != c.len() + 4 {
            return None;
        }
        // Knot k belongs to break (k + 2) / 3, the ends taking four: 0000 111 222 … nnnn.
        let group = |k: usize| k.div_ceil(3).min(segs + 1).saturating_sub(usize::from(k > 0));
        for (k, w) in sp.knots.windows(2).enumerate() {
            let (a, b) = (w.first()?, w.get(1)?);
            let same = group(k) == group(k + 1);
            if (same && a != b) || (!same && a >= b) {
                return None;
            }
        }
        let first = *c.first()?;
        let closed = sp.closed;
        if closed && !first.near(*c.last()?, EPS * (1.0 + first.len())) {
            return None;
        }
        let n = if closed { segs } else { segs + 1 };
        let mut points = Vec::with_capacity(n);
        for i in 0..n {
            let at = *c.get(i * 3)?;
            let after = c.get(i * 3 + 1).copied().unwrap_or(at);
            let before = match i {
                0 if closed => *c.get(segs * 3 - 1)?,
                0 => at,
                _ => *c.get(i * 3 - 1)?,
            };
            points.push(BezierPoint { at, before, after, free: sp.free_points.contains(&i) });
        }
        Some(BezierPath { points, closed })
    }

    /// What control point `index` of [`BezierPath::to_spline`] is.
    pub fn part(&self, index: usize) -> Option<BezierPart> {
        let n = self.points.len();
        if n == 0 || index > self.segments() * 3 {
            return None;
        }
        Some(match index % 3 {
            0 => BezierPart::Point((index / 3) % n),
            1 => BezierPart::After(index / 3),
            _ => BezierPart::Before((index / 3 + 1) % n),
        })
    }

    /// Move a point; its handles go with it.
    pub fn move_point(&mut self, i: usize, to: Vec2) -> Option<()> {
        let p = self.points.get_mut(i)?;
        let d = to - p.at;
        p.at = to;
        p.before += d;
        p.after += d;
        Some(())
    }

    /// Move the end of a handle. While the point's handles are linked the other one turns to
    /// stay in line and keeps its own length, however far the moved one is pulled out. `alone`
    /// leaves it where it is and marks the point's handles independent from then on.
    pub fn move_handle(&mut self, part: BezierPart, to: Vec2, alone: bool) -> Option<()> {
        let (i, outgoing) = match part {
            BezierPart::Before(i) => (i, false),
            BezierPart::After(i) => (i, true),
            BezierPart::Point(_) => return None,
        };
        let p = self.points.get_mut(i)?;
        let linked = p.is_linked();
        p.free |= alone;
        let (moved, other) = if outgoing { (&mut p.after, &mut p.before) } else { (&mut p.before, &mut p.after) };
        *moved = to;
        let dir = (to - p.at).normalized();
        if linked && !alone && dir != Vec2::ZERO {
            *other = p.at - dir * other.dist(p.at);
        }
        Some(())
    }

    /// Retract one handle onto its point.
    pub fn retract(&mut self, part: BezierPart) -> Option<()> {
        match part {
            BezierPart::Before(i) => {
                let p = self.points.get_mut(i)?;
                p.before = p.at;
            }
            BezierPart::After(i) => {
                let p = self.points.get_mut(i)?;
                p.after = p.at;
            }
            BezierPart::Point(i) => {
                let p = self.points.get_mut(i)?;
                *p = BezierPoint::corner(p.at);
            }
        }
        Some(())
    }

    /// Make the handles of point `i` independent of each other, or not. Nothing moves.
    pub fn set_free(&mut self, i: usize, free: bool) -> Option<()> {
        self.points.get_mut(i)?.free = free;
        Some(())
    }

    /// The neighbours of point `i` that a segment joins it to.
    fn neighbours(&self, i: usize) -> (Option<Vec2>, Option<Vec2>) {
        let n = self.points.len();
        let at = |k: usize| self.points.get(k).map(|p| p.at);
        let prev = if i > 0 {
            at(i - 1)
        } else if self.closed {
            at(n.saturating_sub(1))
        } else {
            None
        };
        let next = if i + 1 < n {
            at(i + 1)
        } else if self.closed {
            at(0)
        } else {
            None
        };
        (prev, next)
    }

    /// Give point `i` handles in one line, no longer independent. Handles that are out keep
    /// their lengths and turn into line (a single one is mirrored); a corner gets handles along
    /// the direction from its previous to its next neighbour, a third of the way to each.
    pub fn make_smooth(&mut self, i: usize) -> Option<()> {
        let (prev, next) = self.neighbours(i);
        let p = self.points.get_mut(i)?;
        let at = p.at;
        p.free = false;
        match (p.has_before(), p.has_after()) {
            (true, true) => {
                let dir = (p.after - p.before).normalized();
                if dir == Vec2::ZERO {
                    return None;
                }
                p.before = at - dir * p.before.dist(at);
                p.after = at + dir * p.after.dist(at);
            }
            (true, false) => p.after = at + (at - p.before),
            (false, true) => p.before = at + (at - p.after),
            (false, false) => {
                let dir = match (prev, next) {
                    (Some(a), Some(b)) => (b - a).normalized(),
                    (Some(a), None) => (at - a).normalized(),
                    (None, Some(b)) => (b - at).normalized(),
                    (None, None) => return None,
                };
                if dir == Vec2::ZERO {
                    return None;
                }
                if let Some(a) = prev {
                    p.before = at - dir * (a.dist(at) / 3.0);
                }
                if let Some(b) = next {
                    p.after = at + dir * (b.dist(at) / 3.0);
                }
            }
        }
        Some(())
    }

    /// The segment and parameter of the path's point nearest to `p`.
    fn nearest(&self, p: Vec2) -> Option<(usize, f64)> {
        const STEPS: usize = 48;
        let mut best: Option<(f64, usize, f64)> = None;
        for j in 0..self.segments() {
            let c = self.segment(j)?;
            for k in 0..=STEPS {
                let t = as_f64(k) / as_f64(STEPS);
                let d = cubic(c, t).dist(p);
                if best.is_none_or(|(bd, ..)| d < bd) {
                    best = Some((d, j, t));
                }
            }
        }
        let (_, j, t) = best?;
        // Narrow the bracket round the best sample.
        let c = self.segment(j)?;
        let step = 1.0 / as_f64(STEPS);
        let (mut lo, mut hi) = ((t - step).max(0.0), (t + step).min(1.0));
        for _ in 0..40 {
            let (a, b) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
            if cubic(c, a).dist(p) <= cubic(c, b).dist(p) {
                hi = b;
            } else {
                lo = a;
            }
        }
        Some((j, f64::midpoint(lo, hi)))
    }

    /// Add a point on the path where it passes nearest to `p`, keeping the path's shape: the
    /// segment is split there. Returns the new point's index; `None` when the nearest place is
    /// an existing point.
    pub fn insert_near(&mut self, p: Vec2) -> Option<usize> {
        let (j, t) = self.nearest(p)?;
        let [a, b, c, d] = self.segment(j)?;
        let at = cubic([a, b, c, d], t);
        let tol = EPS * (1.0 + at.len());
        if !(t > 0.0 && t < 1.0) || at.near(a, tol) || at.near(d, tol) {
            return None;
        }
        let n = self.points.len();
        let straight = !self.points.get(j)?.has_after() && !self.points.get((j + 1) % n)?.has_before();
        let new = if straight {
            BezierPoint::corner(at)
        } else {
            let (ab, bc, cd) = (a.lerp(b, t), b.lerp(c, t), c.lerp(d, t));
            self.points.get_mut(j)?.after = ab;
            self.points.get_mut((j + 1) % n)?.before = cd;
            BezierPoint { at, before: ab.lerp(bc, t), after: bc.lerp(cd, t), free: false }
        };
        self.points.insert(j + 1, new);
        Some(j + 1)
    }

    /// Remove point `i`; its neighbours are joined by one segment that uses their own handles.
    /// `None` when that would leave fewer than two points.
    pub fn remove_point(&mut self, i: usize) -> Option<()> {
        if self.points.len() <= 2 || i >= self.points.len() {
            return None;
        }
        self.points.remove(i);
        Some(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> Vec2 {
        Vec2::new(x, y)
    }

    fn sample() -> BezierPath {
        BezierPath {
            points: vec![
                BezierPoint::smooth(v(0.0, 0.0), v(2.0, 3.0)),
                BezierPoint::smooth(v(10.0, 0.0), v(13.0, -2.0)),
                BezierPoint::corner(v(20.0, 5.0)),
            ],
            closed: false,
        }
    }

    fn close(a: Vec2, b: Vec2) -> bool {
        a.dist(b) < 1e-9
    }

    #[test]
    fn a_path_comes_back_from_its_spline_open_and_closed() {
        let mut p = sample();
        p.closed = true;
        assert_eq!(BezierPath::from_spline(&p.to_spline().unwrap()).unwrap(), p);
        // Open, the unused end handles are not kept.
        p.closed = false;
        let back = BezierPath::from_spline(&p.to_spline().unwrap()).unwrap();
        assert!(!back.points[0].has_before() && back.points[0].has_after());
        assert_eq!(back.points[1], p.points[1]);
        assert_eq!(back.points.len(), 3);
    }

    #[test]
    fn the_spline_passes_through_the_points_and_leaves_along_the_handles() {
        let p = sample();
        let sp = p.to_spline().unwrap();
        assert!(sp.is_valid());
        let (lo, hi) = sp.domain();
        assert!(close(sp.eval(lo), v(0.0, 0.0)) && close(sp.eval(hi), v(20.0, 5.0)));
        let mid = lo + (hi - lo) / 2.0;
        assert!(close(sp.eval(mid), v(10.0, 0.0)));
        // Just past the middle point the curve heads the way its outgoing handle points.
        let heading = (sp.eval(mid + 1e-6) - sp.eval(mid)).normalized();
        assert!(heading.dot((v(13.0, -2.0) - v(10.0, 0.0)).normalized()) > 0.999_999);
    }

    #[test]
    fn other_splines_are_not_paths() {
        let pts: Vec<Vec2> = (0..7).map(|i| v(f64::from(i), f64::from(i * i))).collect();
        assert!(BezierPath::from_spline(&Spline::from_control(pts.clone(), 3)).is_none(), "single inner knots");
        assert!(BezierPath::from_spline(&Spline::from_fit_points(&pts)).is_none(), "fit points");
        let mut rational = sample().to_spline().unwrap();
        rational.weights = vec![1.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        assert!(BezierPath::from_spline(&rational).is_none());
    }

    #[test]
    fn a_linked_handle_turns_its_partner_and_leaves_its_length() {
        let mut p = sample();
        p.points[1].before = v(9.0, 2.0 / 3.0); // shorter than the outgoing one, still in line
        let before_len = p.points[1].before.dist(p.points[1].at);
        p.move_handle(BezierPart::After(1), v(10.0, 5.0), false).unwrap();
        let q = p.points[1];
        assert!(q.is_linked());
        assert!((q.before.dist(q.at) - before_len).abs() < 1e-12);
        assert!(q.before.y < q.at.y, "the partner swung to the opposite side");
        // Moved alone, the partner stays and the point becomes a kink.
        let partner = q.before;
        p.move_handle(BezierPart::After(1), v(14.0, 1.0), true).unwrap();
        assert_eq!(p.points[1].before, partner);
        assert!(!p.points[1].is_linked());
        assert!(p.points[1].free);
        // A kink stays a kink on later plain moves.
        p.move_handle(BezierPart::After(1), v(15.0, 2.0), false).unwrap();
        assert_eq!(p.points[1].before, partner);
    }

    #[test]
    fn independent_handles_stay_put_while_in_line_and_survive_the_spline() {
        let mut p = sample();
        p.set_free(1, true).unwrap();
        let partner = p.points[1].before;
        p.move_handle(BezierPart::After(1), v(10.0, 5.0), false).unwrap();
        assert_eq!(p.points[1].before, partner);
        let mut back = BezierPath::from_spline(&p.to_spline().unwrap()).unwrap();
        assert!(back.points[1].free && !back.points[0].free && !back.points[2].free);
        // Removing an earlier point keeps the mark on the same point.
        back.remove_point(0).unwrap();
        assert!(back.points[0].free);
        // Linked again, the handles come back into line and the partner follows once more.
        back.make_smooth(0).unwrap();
        assert!(back.points[0].is_linked());
    }

    #[test]
    fn a_moved_point_takes_its_handles_along() {
        let mut p = sample();
        let was = p.points[1];
        p.move_point(1, v(11.0, 4.0)).unwrap();
        let q = p.points[1];
        assert_eq!(q.after - q.at, was.after - was.at);
        assert_eq!(q.before - q.at, was.before - was.at);
    }

    #[test]
    fn inserting_a_point_keeps_the_shape_and_removing_it_keeps_the_neighbours() {
        let p = sample();
        let mut q = p.clone();
        let i = q.insert_near(v(5.0, 3.0)).unwrap();
        assert_eq!((i, q.points.len()), (1, 4));
        let (a, b) = (p.to_spline().unwrap(), q.to_spline().unwrap());
        // Every point of the new curve lies on the old one.
        let old = a.tessellate(1e-6);
        for k in 0..=60 {
            let (lo, hi) = b.domain();
            let x = b.eval(lo + (hi - lo) * f64::from(k) / 60.0);
            let d = old.windows(2).map(|w| crate::Line::new(w[0], w[1]).closest(x).dist(x)).fold(f64::INFINITY, f64::min);
            assert!(d < 1e-4, "{x:?} is {d} off the original curve");
        }
        // On a straight segment the new point is a corner.
        let mut line = BezierPath { points: vec![BezierPoint::corner(v(0.0, 0.0)), BezierPoint::corner(v(10.0, 0.0))], closed: false };
        let i = line.insert_near(v(4.0, 1.0)).unwrap();
        assert!(!line.points[i].has_before() && !line.points[i].has_after());
        q.remove_point(1).unwrap();
        assert_eq!(q.points.len(), 3);
        assert!(line.remove_point(1).is_some() && line.remove_point(0).is_none(), "two points are the least");
    }

    #[test]
    fn smoothing_a_corner_gives_linked_handles_and_a_kink_comes_back_into_line() {
        let mut p = BezierPath {
            points: vec![BezierPoint::corner(v(0.0, 0.0)), BezierPoint::corner(v(5.0, 5.0)), BezierPoint::corner(v(10.0, 0.0))],
            closed: false,
        };
        p.make_smooth(1).unwrap();
        assert!(p.points[1].is_linked());
        p.move_handle(BezierPart::After(1), v(6.0, 9.0), true).unwrap();
        assert!(!p.points[1].is_linked());
        p.make_smooth(1).unwrap();
        assert!(p.points[1].is_linked());
        p.retract(BezierPart::Point(1)).unwrap();
        assert!(!p.points[1].has_before() && !p.points[1].has_after());
    }

    #[test]
    fn control_points_map_to_parts_and_a_closed_path_ends_on_its_first_point() {
        let mut p = sample();
        assert_eq!(p.part(0), Some(BezierPart::Point(0)));
        assert_eq!(p.part(1), Some(BezierPart::After(0)));
        assert_eq!(p.part(2), Some(BezierPart::Before(1)));
        assert_eq!(p.part(6), Some(BezierPart::Point(2)));
        assert_eq!(p.part(7), None);
        p.closed = true;
        assert_eq!(p.part(8), Some(BezierPart::Before(0)));
        assert_eq!(p.part(9), Some(BezierPart::Point(0)));
        let sp = p.to_spline().unwrap();
        assert_eq!(sp.control.first(), sp.control.last());
    }
}
