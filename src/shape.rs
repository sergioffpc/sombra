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
        let sphere_to_ray = r.o - sphere_center;
        let b = 2.0 * Vector3::dot(&r.d, &sphere_to_ray);

        let sphere_radius = 1.0;
        let c = Vector3::dot(&sphere_to_ray, &sphere_to_ray) - sphere_radius;

        match quadratic_solver(a, b, c) {
            Some((t_min, _)) if t_min > f32::EPSILON && t_min < r.t_max => Some(t_min),
            Some((_, t_max)) if t_max > f32::EPSILON && t_max < r.t_max => Some(t_max),
            _ => None,
        }
        .map(|t| {
            let p = r.position(t);
            let n = p - sphere_center;

            (p, n, t)
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Plane;

impl Shape for Plane {
    fn intersect(&self, r: Ray) -> Option<(Point3<f32>, Vector3<f32>, f32)> {
        let plane_n = -Vector3::z();
        let ray_dot_plane = Vector3::dot(&r.d, &plane_n);

        if ray_dot_plane > 0.0 {
            let plane_p = Point3::new(0.0, 0.0, 0.0);
            let plane_to_ray = plane_p - r.o;
            let t = Vector3::dot(&plane_to_ray, &plane_n) / ray_dot_plane;

            if t > f32::EPSILON {
                let p = r.position(t);
                return Some((p, plane_n, t));
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Scale3, Translation3, Vector3};

    use crate::{shape::Shape, shape::Sphere, Ray};

    #[test]
    fn test_sphere_intersect() {
        // A ray intersects a sphere at two points.
        assert_eq!(
            Sphere.intersect(Ray::new(
                Point3::new(0.0, 0.0, -5.0),
                Vector3::new(0.0, 0.0, 1.0)
            )),
            Some((
                Point3::new(0.0, 0.0, -1.0),
                Vector3::new(0.0, 0.0, -1.0),
                4.0
            ))
        );

        // A ray intersects a sphere at a tangent.
        assert_eq!(
            Sphere.intersect(Ray::new(
                Point3::new(0.0, 1.0, -5.0),
                Vector3::new(0.0, 0.0, 1.0)
            )),
            Some((Point3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 1.0, 0.0), 5.0))
        );

        // A ray misses a sphere.
        assert_eq!(
            Sphere.intersect(Ray::new(
                Point3::new(0.0, 2.0, -5.0),
                Vector3::new(0.0, 0.0, 1.0)
            )),
            None
        );

        // A ray originates inside a sphere.
        assert_eq!(
            Sphere.intersect(Ray::new(
                Point3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0)
            )),
            Some((Point3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0), 1.0))
        );

        // A sphere is behind a ray.
        assert_eq!(
            Sphere.intersect(Ray::new(
                Point3::new(0.0, 0.0, 5.0),
                Vector3::new(0.0, 0.0, 1.0)
            )),
            None
        );
    }

    #[test]
    fn test_sphere_transform() {
        assert_eq!(
            Sphere.intersect(
                Ray::new(Point3::new(0.0, 0.0, -5.0), Vector3::new(0.0, 0.0, 1.0)).transform(
                    Scale3::new(2.0, 2.0, 2.0)
                        .to_homogeneous()
                        .try_inverse()
                        .unwrap()
                )
            ),
            Some((
                Point3::new(0.0, 0.0, -1.0),
                Vector3::new(0.0, 0.0, -1.0),
                3.0
            ))
        );

        assert_eq!(
            Sphere.intersect(
                Ray::new(Point3::new(0.0, 0.0, -5.0), Vector3::new(0.0, 0.0, 1.0)).transform(
                    Translation3::new(0.0, 0.0, 5.0)
                        .to_homogeneous()
                        .try_inverse()
                        .unwrap()
                )
            ),
            Some((
                Point3::new(0.0, 0.0, -1.0),
                Vector3::new(0.0, 0.0, -1.0),
                9.0
            ))
        );
    }
}
