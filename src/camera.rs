use nalgebra::{Matrix4, Point2, Point3, Vector3};

use crate::Ray;

pub trait Camera {
    fn look_at(&mut self, eye: Point3<f32>, target: Point3<f32>, up: Vector3<f32>);
    fn view_ray(&self, p: Point2<i32>) -> Ray;
}

pub struct PerspectiveCamera {
    raster_to_camera: Matrix4<f32>,
    camera_to_world: Matrix4<f32>,
    world_to_camera: Matrix4<f32>,
}

impl PerspectiveCamera {
    pub fn new(resolution: Point2<i32>) -> Self {
        // Calculate rays for rectangular viewport, see: https://en.wikipedia.org/wiki/Ray_tracing_(graphics).
        let t = -Vector3::z();
        let b = t.cross(&Vector3::y());
        let v = b.cross(&t);

        let gx = std::f32::consts::FRAC_PI_4.tan();
        let gy = gx * (resolution.y as f32 / resolution.x as f32);

        let qx = (2.0 * gx / resolution.x as f32) * b;
        let qy = (2.0 * gy / resolution.y as f32) * v;

        let p0y = t - gx * b - gy * v;

        #[rustfmt::skip]
        let raster_to_camera = Matrix4::new(
            qx.x, qy.x, 0.0, p0y.x,
            qx.y, qy.y, 0.0, p0y.y,
            qx.z, qy.z, 0.0, p0y.z,
            0.0, 0.0, 0.0, 1.0,
        );

        Self {
            raster_to_camera,
            camera_to_world: Matrix4::identity(),
            world_to_camera: Matrix4::identity(),
        }
    }
}

impl Camera for PerspectiveCamera {
    fn look_at(&mut self, eye: Point3<f32>, target: Point3<f32>, up: Vector3<f32>) {
        self.camera_to_world = Matrix4::look_at_rh(&eye, &target, &up);
        self.world_to_camera = self.camera_to_world.try_inverse().unwrap();
    }

    fn view_ray(&self, p: Point2<i32>) -> Ray {
        let o = Point3::new(0.0, 0.0, 0.0);
        let d = self
            .raster_to_camera
            .transform_point(&Point3::new(p.x as f32, p.y as f32, 0.0))
            - o;

        Ray::new(
            self.world_to_camera.transform_point(&o),
            self.world_to_camera.transform_vector(&d).normalize(),
        )
    }
}
