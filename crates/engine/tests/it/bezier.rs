//! BEZIER: points with handles at the command line and in the JSON form, and the grips of the
//! curve it draws (points carry their handles, linked handles stay in line).

use cadcraft_doc::{EntityKind, Handle};
use cadcraft_engine::{Input, Session};
use cadcraft_geom::{BezierPath, Vec2};
use serde_json::json;

fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

fn last_path(s: &Session) -> BezierPath {
    let e = s.doc().unwrap().model.iter().last().unwrap();
    let EntityKind::Spline(sp) = &e.kind else { panic!("no spline") };
    assert!(sp.is_valid());
    BezierPath::from_spline(sp).expect("a Bezier path")
}

fn last_handle(s: &Session) -> Handle {
    s.doc().unwrap().model.iter().last().unwrap().handle
}

fn count(s: &Session) -> usize {
    s.doc().unwrap().model.iter().count()
}

/// A corner, a smooth point and a corner: control points 0..=6, the middle point at 3.
fn drawn() -> Session {
    let mut s = Session::new();
    s.execute("bezier", &json!({ "points": [[0, 0], { "at": [10, 0], "handle": [13, -2] }, [20, 5]] })).unwrap();
    s
}

fn stretch(s: &mut Session, index: usize, to: [f64; 2]) {
    let h = last_handle(s).hex();
    s.execute("grip.stretch", &json!({ "handle": h, "index": index, "to": to })).unwrap();
}

fn option(s: &mut Session, index: usize, option: &str, to: Option<[f64; 2]>) -> cadcraft_engine::Result<serde_json::Value> {
    let h = last_handle(s).hex();
    s.execute("grip.option", &json!({ "handle": h, "index": index, "option": option, "to": to }))
}

#[test]
fn points_typed_with_handles_make_smooth_points_and_plain_ones_corners() {
    let mut s = Session::new();
    s.cmdline("bezier 0,0 h 2,3 10,0 h 13,-2 20,5").unwrap();
    assert!(s.running.is_some(), "the curve goes on until Enter");
    s.cmdline("").unwrap();
    assert!(s.running.is_none());
    let p = last_path(&s);
    assert_eq!(p.points.len(), 3);
    assert!(!p.closed);
    // The first point leaves along its handle; the middle one has both, in line; the last is a corner.
    assert_eq!(p.points[0].after, v(2.0, 3.0));
    assert!(p.points[1].is_linked());
    assert_eq!(p.points[1].after, v(13.0, -2.0));
    assert_eq!(p.points[1].before, v(7.0, 2.0));
    assert!(!p.points[2].has_before());
}

#[test]
fn enter_at_the_handle_prompt_leaves_a_corner_and_undo_takes_the_last_point_back() {
    let mut s = Session::new();
    s.cmdline("bezier 0,0 5,5 h").unwrap();
    s.cmdline("").unwrap();
    assert!(s.running.is_some(), "Enter answered the handle prompt, not the curve");
    s.cmdline("9,9 u 10,0").unwrap();
    s.cmdline("").unwrap();
    let p = last_path(&s);
    assert_eq!(p.points.iter().map(|q| q.at).collect::<Vec<_>>(), [v(0.0, 0.0), v(5.0, 5.0), v(10.0, 0.0)]);
    assert!(p.points.iter().all(|q| !q.has_before() && !q.has_after()));
}

#[test]
fn close_returns_to_the_first_point_and_a_point_on_the_last_one_ends_the_curve() {
    let mut s = Session::new();
    s.cmdline("bezier 0,0 h 0,4 10,0 10,10 c").unwrap();
    assert!(s.running.is_none());
    let p = last_path(&s);
    assert!(p.closed);
    assert_eq!(p.points.len(), 3);
    // Closed, the first point's mirrored handle is in use: the last segment arrives along it.
    assert_eq!(p.points[0].before, v(0.0, -4.0));
    // The second click of a double-click lands on the point the first one placed.
    s.cmdline("bezier 0,0 5,5").unwrap();
    s.input(Input::Point(v(9.0, 1.0))).unwrap();
    s.input(Input::Point(v(9.0, 1.0))).unwrap();
    assert!(s.running.is_none());
    assert_eq!(last_path(&s).points.len(), 3);
}

#[test]
fn fewer_than_two_points_draw_nothing() {
    let mut s = Session::new();
    s.cmdline("bezier 3,3").unwrap();
    s.cmdline("").unwrap();
    assert!(s.running.is_none());
    assert_eq!(count(&s), 0);
    assert!(s.execute("bezier", &json!({ "points": [[1, 1]] })).is_err());
    assert!(s.execute("bezier", &json!({ "points": [[1, 1], [1, 1]] })).is_err());
    assert!(s.execute("bezier", &json!({ "points": [[1, 1], { "at": [2, 2], "handle": "up" }] })).is_err());
    assert_eq!(count(&s), 0);
}

#[test]
fn a_point_grip_carries_its_handles_and_a_handle_grip_turns_its_partner() {
    let mut s = drawn();
    stretch(&mut s, 3, [10.0, 4.0]);
    let p = last_path(&s);
    assert_eq!(p.points[1].after - p.points[1].at, v(3.0, -2.0));
    assert_eq!(p.points[1].before - p.points[1].at, v(-3.0, 2.0));
    // The outgoing handle, dragged straight up and longer: the incoming one points straight down
    // and is as long as it was.
    let len = p.points[1].before.dist(p.points[1].at);
    stretch(&mut s, 4, [10.0, 12.0]);
    let q = last_path(&s).points[1];
    assert!(q.is_linked());
    assert!(q.before.near(v(10.0, 4.0 - len), 1e-9));
    // The grips on a corner's retracted handles move the corner.
    stretch(&mut s, 5, [22.0, 6.0]);
    let q = last_path(&s).points[2];
    assert_eq!(q.at, v(22.0, 6.0));
    assert!(!q.has_before());
}

#[test]
fn grip_options_break_and_restore_the_link_and_add_and_remove_points() {
    let mut s = drawn();
    let partner = last_path(&s).points[1].before;
    option(&mut s, 4, "moveHandleAlone", Some([14.0, 3.0])).unwrap();
    let q = last_path(&s).points[1];
    assert_eq!(q.before, partner);
    assert!(!q.is_linked());
    option(&mut s, 3, "smoothPoint", None).unwrap();
    assert!(last_path(&s).points[1].is_linked());
    option(&mut s, 2, "retractHandle", None).unwrap();
    let q = last_path(&s).points[1];
    assert!(!q.has_before() && q.has_after());
    option(&mut s, 3, "cornerPoint", None).unwrap();
    assert!(!last_path(&s).points[1].has_after());
    // A corner has no handle options, and a handle none of a point's.
    assert!(option(&mut s, 3, "cornerPoint", None).is_err());
    assert!(option(&mut s, 3, "retractHandle", None).is_err());
    option(&mut s, 0, "addVertex", Some([5.0, 1.0])).unwrap();
    let p = last_path(&s);
    assert_eq!(p.points.len(), 4);
    assert!(p.points[1].at.x > 0.0 && p.points[1].at.x < 10.0);
    option(&mut s, 3, "removeVertex", None).unwrap();
    option(&mut s, 3, "removeVertex", None).unwrap();
    assert_eq!(last_path(&s).points.len(), 2);
    assert!(option(&mut s, 0, "removeVertex", None).is_err(), "two points are the least");
    // One undo per edit.
    s.execute("undo", &json!({})).unwrap();
    assert_eq!(last_path(&s).points.len(), 3);
}

#[test]
fn independent_handles_is_a_lasting_choice_for_one_point_until_it_is_smoothed() {
    let mut s = drawn();
    let was = last_path(&s).points[1];
    // Choosing it moves nothing.
    option(&mut s, 3, "independentHandles", None).unwrap();
    assert_eq!((last_path(&s).points[1].before, last_path(&s).points[1].after), (was.before, was.after));
    assert!(option(&mut s, 3, "independentHandles", None).is_err(), "already independent");
    // Each handle now moves by itself, with the ordinary grip stretch, as often as wanted.
    stretch(&mut s, 4, [10.0, 12.0]);
    stretch(&mut s, 4, [16.0, 3.0]);
    assert_eq!(last_path(&s).points[1].before, was.before);
    stretch(&mut s, 2, [5.0, -4.0]);
    assert_eq!(last_path(&s).points[1].after, v(16.0, 3.0));
    // It is offered from a handle grip too, and Smooth Point links the handles again.
    option(&mut s, 4, "smoothPoint", None).unwrap();
    let q = last_path(&s).points[1];
    assert!(q.is_linked());
    option(&mut s, 2, "independentHandles", None).unwrap();
    stretch(&mut s, 2, [8.0, 8.0]);
    assert_eq!(last_path(&s).points[1].after, q.after);
}

#[test]
fn a_linked_handle_pulled_longer_only_turns_its_partner() {
    let mut s = drawn();
    let was = last_path(&s).points[1];
    let partner_len = was.before.dist(was.at);
    // Far out and round to another direction, twice.
    for to in [[40.0, 30.0], [10.5, -0.5]] {
        stretch(&mut s, 4, to);
        let q = last_path(&s).points[1];
        assert!((q.before.dist(q.at) - partner_len).abs() < 1e-9, "the partner kept its length");
        assert!((q.after.dist(q.at) - partner_len).abs() > 0.5, "the moved handle did not");
        assert!(q.is_linked(), "and they are still in line");
    }
}

#[test]
fn the_preview_bends_into_the_point_while_its_handle_follows_the_cursor() {
    let mut s = Session::new();
    s.cmdline("bezier 0,0 10,0").unwrap();
    let straight = s.preview(v(20.0, 0.0));
    s.cmdline("h").unwrap();
    let bent = s.preview(v(10.0, 5.0));
    let EntityKind::Spline(sp) = &bent.first().unwrap().kind else { panic!("no curve in the preview") };
    // The handle points up, so the curve comes into the point from below.
    let (lo, hi) = sp.domain();
    assert!(sp.eval(lo + (hi - lo) * 0.9).y < 0.0);
    assert!(bent.iter().any(|e| matches!(e.kind, EntityKind::Line(_))), "the handle is drawn");
    assert!(straight.iter().all(|e| !matches!(e.kind, EntityKind::Line(_))));
    s.input(Input::Cancel).unwrap();
    assert_eq!(count(&s), 0);
}
