use nalgebra::{Point3, Vector3};

use crate::{quadratic_solver, Ray};

pub trait Shape {
    fn intersect(&self, r: Ray) -> Option<(Point3<f32>, Vector3<f32>, f32)>;
}

pub struct Sphere;

impl Shape for Sphere {
    fn intersect(&self, r: Ray) -> Option<(Point3<f32>, Vector3<f32>, f32)> {
        let sphere_center = Point3::new(0.0, 0.0, 0.0);
        let ray_to_sphere = r.o - sphere_center;

        let a = Vector3::dot(&r.d, &r.d);
        let b = 2.0 * Vector3::dot(&ray_to_sphere, &r.d);
        let c = Vector3::dot(&ray_to_sphere, &ray_to_sphere) - 1.0;

        match quadratic_solver(a, b, c) {
            Some((t_min, _)) if t_min >= 0.0 && t_min < r.t_max => Some(t_min),
            Some((_, t_max)) if t_max >= 0.0 && t_max < r.t_max => Some(t_max),
            _ => None,
        }
        .map(|t| {
            let p = r.position(t);
            let n = (p - sphere_center).normalize();

            (p, n, t)
        })
    }
}
