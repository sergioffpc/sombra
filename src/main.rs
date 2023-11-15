#![warn(clippy::all, clippy::pedantic, clippy::nursery)]

use indicatif::ProgressBar;
use nalgebra::{Point2, Point3, Rotation3, Translation3, Vector3};
use sombra::{
    camera::{Camera, PerspectiveCamera},
    integrator::{Integrator, WhittedIntegrator},
    light::PointLight,
    scene::{GeometryPrimitive, LightPrimitive, Scene},
    shape::{Plane, Sphere},
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

    let mut plane = GeometryPrimitive::new(Box::new(Plane));
    plane.set_object_to_world(
        Translation3::new(0.0, 1.0, 0.0).to_homogeneous()
            * Rotation3::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_2)
                .to_homogeneous(),
    );
    scene.add_geometry(plane);

    let mut sphere = GeometryPrimitive::new(Box::new(Sphere));
    sphere.set_object_to_world(Translation3::new(2.0, 0.0, 0.0).to_homogeneous());
    scene.add_geometry(sphere);

    let mut sphere = GeometryPrimitive::new(Box::new(Sphere));
    sphere.set_object_to_world(Translation3::new(-2.0, 0.0, 0.0).to_homogeneous());
    scene.add_geometry(sphere);

    let mut light = LightPrimitive::new(Box::new(PointLight));
    light.set_object_to_world(Translation3::new(10.0, -10.0, 10.0).to_homogeneous());
    scene.add_light(light);

    let integrator = WhittedIntegrator::default();

    let mut film = Film::new(resolution);

    let progress = ProgressBar::new((resolution.x * resolution.y) as u64);
    for y in 0..resolution.y {
        for x in 0..resolution.x {
            let p = Point2::new(x, y);
            let s = scene
                .intersect(c.view_ray(p))
                .map_or(Spectrum::BLACK, |isect| integrator.lo(&scene, isect));
            film.add_sample(p, s);

            progress.inc(1);
        }
    }
    progress.finish();

    film.write("image.exr").unwrap();
}
