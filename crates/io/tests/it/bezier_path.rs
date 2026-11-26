//! A Bézier path is a SPLINE in DXF; the points whose handles are independent, which its
//! geometry does not show, come back with it.

use cadcraft_doc::{Common, Drawing, EntityKind, Space};
use cadcraft_geom::{BezierPath, BezierPoint, Vec2};

#[test]
fn a_path_and_its_independent_points_roundtrip_through_dxf() {
    let v = Vec2::new;
    let mut path = BezierPath {
        points: vec![
            BezierPoint::corner(v(0.0, 0.0)),
            BezierPoint::smooth(v(10.0, 0.0), v(13.0, -2.0)),
            BezierPoint::smooth(v(20.0, 5.0), v(22.0, 9.0)),
        ],
        closed: true,
    };
    path.set_free(1, true).unwrap();
    let mut d = Drawing::new_metric();
    d.add(&Space::Model, Common::default(), EntityKind::Spline(path.to_spline().unwrap())).unwrap();
    let bytes = cadcraft_io::write(&d, "a.dxf").unwrap();
    let back = cadcraft_io::read(&bytes, "a.dxf").unwrap();
    let Some(e) = back.model.iter().next() else { panic!("no spline") };
    let EntityKind::Spline(sp) = &e.kind else { panic!("no spline") };
    let read = BezierPath::from_spline(sp).expect("still a Bezier path");
    assert_eq!(read.points.iter().map(|p| p.free).collect::<Vec<_>>(), [false, true, false]);
    assert!(read.closed);
    assert!(read.points[2].is_linked() && !read.points[1].is_linked());
    // Our own data does not come back as another application's.
    assert!(e.common.xdata.is_empty());
}
