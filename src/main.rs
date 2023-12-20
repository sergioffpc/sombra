#![warn(clippy::all, clippy::pedantic, clippy::nursery)]

use clap::Parser;
use indicatif::{ProgressBar, ProgressStyle};
use nalgebra::{Point2, Point3, Rotation3, Translation3, Vector3};
use sombra::{
    camera::{Camera, Film, PerspectiveCamera},
    integrator::{Integrator, WhittedIntegrator},
    light::PointLight,
    material::MatteMaterial,
    material::MirrorMaterial,
    sampler::RandomSampler,
    scene::{GeometryPrimitive, LightPrimitive, Scene},
    shape::{Plane, Sphere},
    Spectrum,
};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(long, default_value_t = 1280)]
    width: i32,

    #[arg(long, default_value_t = 720)]
    height: i32,

    #[arg(long, default_value_t = 1)]
    samples_per_pixel: i32,

    #[arg(long, default_value_t = String::from("image.exr"))]
    output: String,
}

fn main() {
    let args = Args::parse();

    let resolution = Point2::new(args.width, args.height);
    let mut camera = PerspectiveCamera::new(resolution);
    camera.look_at(
        Point3::new(0.0, 0.0, 5.0),
        Point3::new(0.0, 0.0, 0.0),
        Vector3::y(),
    );

    let mut scene = Scene::default();

    let plane_material = MatteMaterial::new(Spectrum::WHITE);
    let mut plane = GeometryPrimitive::new(Box::new(Plane), Box::new(plane_material));
    plane.set_object_to_world(
        Translation3::new(0.0, -1.0, 0.0).to_homogeneous()
            * Rotation3::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_2)
                .to_homogeneous(),
    );
    scene.add_geometry(plane);

    let right_sphere_matte = MirrorMaterial::new(Spectrum::WHITE / std::f32::consts::PI);
    let mut right_sphere = GeometryPrimitive::new(Box::new(Sphere), Box::new(right_sphere_matte));
    right_sphere.set_object_to_world(Translation3::new(2.0, 0.0, 0.0).to_homogeneous());
    scene.add_geometry(right_sphere);

    let left_sphere_matte = MatteMaterial::new(Spectrum::WHITE / std::f32::consts::PI);
    let mut left_sphere = GeometryPrimitive::new(Box::new(Sphere), Box::new(left_sphere_matte));
    left_sphere.set_object_to_world(Translation3::new(-2.0, 0.0, 0.0).to_homogeneous());
    scene.add_geometry(left_sphere);

    let mut light = LightPrimitive::new(Box::new(PointLight::default()));
    light.set_object_to_world(Translation3::new(0.0, 1.0, 0.0).to_homogeneous());
    scene.add_light(light);

    let integrator = WhittedIntegrator::new(64);

    let samples_per_pixel = args.samples_per_pixel;
    let mut film = Film::new(resolution, samples_per_pixel);

    let progress = ProgressBar::new((resolution.x * resolution.y * samples_per_pixel) as u64)
        .with_style(
            ProgressStyle::with_template("[{elapsed_precise}] {bar:40} {pos:>7}/{len:7} {msg}")
                .unwrap(),
        );
    for y in 0..resolution.y {
        for x in 0..resolution.x {
            for _ in 0..samples_per_pixel {
                let sampler = RandomSampler;
                let p = Point2::new(x, y);
                let ray = camera.view_ray(p, &sampler);
                let lo = integrator.lo(&scene, ray, &sampler);
                film.add_sample(p, lo);

                progress.inc(1);
            }
        }
    }
    progress.finish();

    film.write(args.output.as_str()).unwrap();
}
