//! BEZIER: a curve drawn point by point, where each point may carry a handle that bends the
//! curve through it. A point placed plainly is a corner; a point given a handle is smooth, its
//! incoming handle the mirror image of the outgoing one. The result is a cubic spline whose
//! control points are the points and handle ends ([`cadcraft_geom::BezierPath`]), so it saves,
//! snaps and plots as any spline and its grips are the points and handles.
//!
//! The way of drawing follows what drawing programs commonly do (click for a corner, press and
//! drag for a smooth point, the handle following the pointer); the behaviour notes it was
//! written from are a contributor's own observation of such a tool, kept out of the repository.

use cadcraft_geom::{BezierPath, BezierPoint};

use super::{Accept, EntityKind, Input, Interactive, Prompt, Result, Session, Step, Value, Vec2, added, bad, bool_or, point_param, point_value};
use crate::cmd::helpers::line;

/// At most this many points in one path, as other generated curves are capped.
const MAX_POINTS: usize = crate::cmd::curves::MAX_GEN;

fn json_point(v: &Value) -> Option<BezierPoint> {
    if let Some(at) = point_value(v) {
        return Some(BezierPoint::corner(at));
    }
    let at = point_param(v, "at")?;
    let mut p = match v.get("handle") {
        None | Some(Value::Null) => BezierPoint::corner(at),
        Some(h) => BezierPoint::smooth(at, point_value(h)?),
    };
    for (key, end) in [("in", &mut p.before), ("out", &mut p.after)] {
        match v.get(key) {
            None | Some(Value::Null) => {}
            Some(h) => *end = point_value(h)?,
        }
    }
    Some(p)
}

pub(super) fn run_bezier(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "bezier";
    let pts = p.get("points").and_then(Value::as_array).ok_or_else(|| bad(cmd, "`points` is required"))?;
    if pts.len() > MAX_POINTS {
        return Err(bad(cmd, "too many points"));
    }
    let points = pts
        .iter()
        .map(json_point)
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| bad(cmd, "each point is [x, y] or {at: [x, y], handle?: [x, y], in?: [x, y], out?: [x, y]}"))?;
    if points.windows(2).any(|w| matches!(w, [a, b] if a.at.near(b.at, 1e-12))) {
        return Err(bad(cmd, "two points in a row are at the same place"));
    }
    let sp = BezierPath { points, closed: bool_or(p, "closed", false) }.to_spline().ok_or_else(|| bad(cmd, "need 2+ points"))?;
    added(s.add_entity(EntityKind::Spline(sp))?)
}

#[derive(Default)]
pub(in crate::cmd) struct BezierM {
    pts: Vec<BezierPoint>,
    /// The next point is the end of the last point's handle.
    handle: bool,
}

impl BezierM {
    fn finish(&self, s: &mut Session, closed: bool) -> Result<Step> {
        if let Some(sp) = (BezierPath { points: self.pts.clone(), closed }).to_spline() {
            s.add_entity(EntityKind::Spline(sp))?;
        }
        Ok(Step::Done)
    }
}

impl Interactive for BezierM {
    fn name(&self) -> &'static str {
        "BEZIER"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        let last = self.pts.last().map(|p| p.at);
        if self.handle {
            return Prompt::new("Specify handle end", Accept::POINT).default("none").base_opt(last);
        }
        match self.pts.len() {
            0 => Prompt::new("Specify first point", Accept::POINT),
            1 => Prompt::new("Specify next point", Accept::POINT).kw(&["Handle", "Undo"]).base_opt(last),
            _ => Prompt::new("Specify next point", Accept::POINT).kw(&["Handle", "Undo", "Close"]).base_opt(last),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if self.handle {
            match i {
                Input::Point(p) => {
                    if let Some(last) = self.pts.last_mut() {
                        *last = BezierPoint::smooth(last.at, p);
                    }
                }
                Input::Enter => {}
                Input::Keyword(_) | Input::Text(_) => return Err(crate::EngineError::Other("Point required.".into())),
                _ => return Ok(Step::Continue),
            }
            self.handle = false;
            return Ok(Step::Continue);
        }
        match i {
            // A point on the last one (the second click of a double-click) ends the curve.
            Input::Point(p) if self.pts.last().is_some_and(|l| l.at.near(p, 1e-9 * (1.0 + p.len()))) => self.finish(s, false),
            Input::Point(p) => {
                if self.pts.len() >= MAX_POINTS {
                    return Err(crate::EngineError::Other("Too many points.".into()));
                }
                self.pts.push(BezierPoint::corner(p));
                Ok(Step::Continue)
            }
            Input::Keyword(k) if k == "Handle" => {
                self.handle = true;
                Ok(Step::Continue)
            }
            Input::Keyword(k) if k == "Undo" => {
                self.pts.pop();
                Ok(Step::Continue)
            }
            Input::Keyword(k) if k == "Close" => self.finish(s, true),
            Input::Enter => self.finish(s, false),
            Input::Keyword(_) | Input::Text(_) => Err(crate::EngineError::Other("Point or option keyword required.".into())),
            _ => Ok(Step::Continue),
        }
    }
    fn preview(&self, _s: &Session, c: Vec2) -> Vec<EntityKind> {
        let mut points = self.pts.clone();
        let mut out = Vec::new();
        if self.handle {
            // The handle follows the cursor; the curve into the point bends with its mirror image.
            if let Some(last) = points.last_mut() {
                *last = BezierPoint::smooth(last.at, c);
                out.push(line(last.at, c));
            }
        } else if !points.is_empty() {
            points.push(BezierPoint::corner(c));
        }
        if let Some(sp) = (BezierPath { points, closed: false }).to_spline() {
            out.insert(0, EntityKind::Spline(sp));
        }
        out
    }
}
