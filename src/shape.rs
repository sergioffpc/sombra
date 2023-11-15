use nalgebra::{Point3, Vector3};

use crate::{quadratic_solver, Ray};

pub trait Shape {
    fn intersect(&self, r: Ray) -> Option<(Point3<f32>, Vector3<f32>, f32)>;
}

#[derive(Clone, Copy, Debug)]
pub struct Sphere;

impl Shape for Sphere {
    fn intersect(&self, r: Ray) -> Option<(Point3<f32>, Vector3<f32>, f32)> {
        let a = Vector3::dot(&r.d, &r.d);

        let sphere_center = Point3::new(0.0, 0.0, 0.0);
        let ray_to_sphere = r.o - sphere_center;
        let b = 2.0 * Vector3::dot(&ray_to_sphere, &r.d);

        let sphere_radius = 1.0;
        let c = Vector3::dot(&ray_to_sphere, &ray_to_sphere) - sphere_radius;

        match quadratic_solver(a, b, c) {
            Some((t_min, _)) if t_min > std::f32::EPSILON && t_min < r.t_max => Some(t_min),
            Some((_, t_max)) if t_max > std::f32::EPSILON && t_max < r.t_max => Some(t_max),
            _ => None,
        }
        .map(|t| {
            let p = r.position(t);
            let n = (p - sphere_center).normalize();

            (p, n, t)
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Plane;

impl Shape for Plane {
    fn intersect(&self, r: Ray) -> Option<(Point3<f32>, Vector3<f32>, f32)> {
        let plane_n = -Vector3::z();
        let d_dot_n = Vector3::dot(&r.d, &plane_n);

        if d_dot_n > std::f32::EPSILON {
            let plane_p = Point3::new(0.0, 0.0, 0.0);
            let plane_to_ray = plane_p - r.o;
            let t = Vector3::dot(&plane_to_ray, &plane_n) / d_dot_n;

            if t > std::f32::EPSILON {
                let p = r.position(t);
                return Some((p, plane_n, t));
            }
        }

        None
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Disk;

impl Shape for Disk {
    fn intersect(&self, r: Ray) -> Option<(Point3<f32>, Vector3<f32>, f32)> {
        if let Some((p, n, t)) = Plane.intersect(r) {
            let disk_p = Point3::new(0.0, 0.0, 0.0);
            let dist_p = p - disk_p;
            let disk_square_radius = 1.0;
            if Vector3::dot(&dist_p, &dist_p) <= disk_square_radius {
                return Some((p, n, t));
            }
        }

        None
    }
}
