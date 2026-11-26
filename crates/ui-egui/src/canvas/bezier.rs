//! BEZIER on the canvas: a press places a point; dragging before letting go pulls the point's
//! handle out, which ends where the button comes up. A second press on the spot straight after
//! (a double-click) ends the curve. Typed, the same is the Handle option and Enter.
//!
//! A selected Bézier curve shows a line from each point to its handle ends, so the handle
//! grips read as belonging to their point.

use cadcraft_doc::EntityKind;
use cadcraft_engine::Input;
use cadcraft_geom::BezierPath;
use egui::{Color32, Pos2, Stroke};

use super::{CadApp, Xf, press};

/// Presses this close in time (seconds) on one spot are a double-click.
const DOUBLE_CLICK: f64 = 0.4;

pub(super) fn running(app: &CadApp) -> bool {
    app.session.prompting() && app.session.running.as_ref().is_some_and(|r| r.id == "bezier")
}

/// A press at `pos` while BEZIER runs. Returns true when it is the second click of a
/// double-click, which ends the curve instead of placing a point.
pub(super) fn second_click(app: &mut CadApp, pos: Pos2, now: f64) -> bool {
    if !running(app) {
        app.canvas.bezier_click = None;
        return false;
    }
    let again = app.canvas.bezier_click.take().is_some_and(|(at, t)| now - t <= DOUBLE_CLICK && at.distance(pos) <= press::TOLERANCE);
    if again {
        if let Err(e) = app.session.input(Input::Enter) {
            app.session.echo(e.to_string());
        }
    } else {
        app.canvas.bezier_click = Some((pos, now));
    }
    again
}

/// The held press left its spot: it now drags the handle of the point it placed.
pub(super) fn drag_started(app: &mut CadApp) {
    if app.canvas.bezier_handle || !running(app) {
        return;
    }
    // A drag is no double-click.
    app.canvas.bezier_click = None;
    app.canvas.bezier_handle = app.session.input(Input::Keyword("Handle".into())).is_ok();
}

/// Lines from the points of a selected Bézier curve to their handle ends.
pub(super) fn paint_handles(painter: &egui::Painter, xf: &Xf, kind: &EntityKind, color: Color32) {
    let EntityKind::Spline(sp) = kind else { return };
    let Some(path) = BezierPath::from_spline(sp) else { return };
    let stroke = Stroke::new(1.0, color);
    for p in &path.points {
        let at = xf.to_screen(p.at);
        for (out, end) in [(p.has_before(), p.before), (p.has_after(), p.after)] {
            if out {
                painter.line_segment([at, xf.to_screen(end)], stroke);
            }
        }
    }
}
