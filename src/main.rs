#![warn(clippy::all, clippy::pedantic, clippy::nursery)]

use nalgebra::{Point2, Point3, Vector3};
use sombra::{
    camera::{Camera, PerspectiveCamera},
    scene::{Geometry, Scene},
    shape::Sphere,
    Film, Spectrum,
};

fn main() {
    let resolution = Point2::new(1280, 720);

    let mut c = PerspectiveCamera::new(resolution);
    c.look_at(
        Point3::new(0.0, 0.0, 4.0),
        Point3::new(0.0, 0.0, 0.0),
        Vector3::y(),
    );

    let mut scene = Scene::default();
    let sphere = Geometry::new(Box::new(Sphere));
    scene.add_geometry(sphere);

    let mut film = Film::new(resolution);
    for y in 0..resolution.y {
        for x in 0..resolution.x {
            let p = Point2::new(x, y);
            let r = c.view_ray(p);
            let s = scene.intersect(r).map_or(Spectrum::BLUE, |isect| {
                Spectrum::new(isect.n.x, isect.n.y, isect.n.z)
            });
            film.add_sample(p, s);
        }
    }
    film.write("image.exr").unwrap();
}
