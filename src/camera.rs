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
        let forward = -Vector3::z();
        let right = forward.cross(&Vector3::y());
        let up = right.cross(&forward);

        let half_fov = std::f32::consts::FRAC_PI_4;

        let viewport_half_width = half_fov.tan();
        let viewport_half_height =
            viewport_half_width * (resolution.y as f32 / resolution.x as f32);

        let pixel_shift_right = (2.0 * viewport_half_width / resolution.x as f32) * right;
        let pixel_shift_up = (2.0 * viewport_half_height / resolution.y as f32) * up;

        let viewport_o = forward - viewport_half_width * right - viewport_half_height * up;

        #[rustfmt::skip]
        let raster_to_camera = Matrix4::new(
            pixel_shift_right.x,  pixel_shift_up.x, 0.0,  viewport_o.x,
            pixel_shift_right.y, -pixel_shift_up.y, 0.0, -viewport_o.y,
            pixel_shift_right.z,  pixel_shift_up.z, 0.0,  viewport_o.z,
                            0.0,               0.0, 0.0,           1.0,
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
