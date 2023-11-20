#![warn(clippy::all, clippy::pedantic, clippy::nursery)]

use indicatif::ProgressBar;
use nalgebra::{Point2, Point3, Rotation3, Translation3, Vector3};
use sombra::{
    camera::{Camera, Film, PerspectiveCamera},
    integrator::{Integrator, WhittedIntegrator},
    light::PointLight,
    reflection::{BxDF, LambertianReflection},
    sampler::RandomSampler,
    scene::{GeometryPrimitive, LightPrimitive, Scene},
    shape::{Plane, Sphere},
    Spectrum,
};

fn main() {
    let resolution = Point2::new(1280, 720);
    let samples_per_pixel = 256;

    let mut camera = PerspectiveCamera::new(resolution);
    camera.look_at(
        Point3::new(0.0, 0.0, 5.0),
        Point3::new(0.0, 0.0, 0.0),
        Vector3::y(),
    );

    let mut scene = Scene::default();

    let mut red_matte = BxDF::default();
    red_matte.add_bxdf(Box::new(LambertianReflection::new(Spectrum::RED)));

    let mut plane = GeometryPrimitive::new(Box::new(Plane), red_matte);
    plane.set_object_to_world(
        Translation3::new(0.0, -1.0, 0.0).to_homogeneous()
            * Rotation3::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_2)
                .to_homogeneous(),
    );
    scene.add_geometry(plane);

    let mut green_matte = BxDF::default();
    green_matte.add_bxdf(Box::new(LambertianReflection::new(Spectrum::GREEN)));

    let mut sphere = GeometryPrimitive::new(Box::new(Sphere), green_matte);
    sphere.set_object_to_world(Translation3::new(2.0, 0.0, 0.0).to_homogeneous());
    scene.add_geometry(sphere);

    let mut blue_matte = BxDF::default();
    blue_matte.add_bxdf(Box::new(LambertianReflection::new(Spectrum::BLUE)));

    let mut sphere = GeometryPrimitive::new(Box::new(Sphere), blue_matte);
    sphere.set_object_to_world(Translation3::new(-2.0, 0.0, 0.0).to_homogeneous());
    scene.add_geometry(sphere);

    let mut light = LightPrimitive::new(Box::new(PointLight));
    light.set_object_to_world(Translation3::new(0.0, 1.0, 0.0).to_homogeneous());
    scene.add_light(light);

    let integrator = WhittedIntegrator::new(4);

    let mut film = Film::new(resolution, samples_per_pixel);

    let progress = ProgressBar::new((resolution.x * resolution.y * samples_per_pixel) as u64);
    for y in 0..resolution.y {
        for x in 0..resolution.x {
            for _ in 0..samples_per_pixel {
                let sampler = RandomSampler;
                let p = Point2::new(x, y);
                let lo = integrator.lo(&scene, camera.view_ray(p, &sampler), &sampler);
                film.add_sample(p, lo);

                progress.inc(1);
            }
        }
    }
    progress.finish();

    film.write("image.exr").unwrap();
}
