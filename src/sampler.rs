use nalgebra::{Point2, Rotation3, Unit, Vector3};
use rand::Rng;

pub trait Sampler {
    fn sample_point2(&self) -> Point2<f32>;
    fn sample_hemisphere(&self, n: Vector3<f32>) -> Vector3<f32>;
}

pub struct RandomSampler;

impl Sampler for RandomSampler {
    fn sample_point2(&self) -> Point2<f32> {
        let mut rng = rand::thread_rng();

        Point2::new(rng.gen(), rng.gen())
    }

    fn sample_hemisphere(&self, n: Vector3<f32>) -> Vector3<f32> {
        let mut rng = rand::thread_rng();

        let theta = rng.gen_range(0.0..std::f32::consts::FRAC_PI_2);
        let phi = rng.gen_range(0.0..2.0 * std::f32::consts::PI);

        let x = theta.sin() * phi.cos();
        let y = theta.sin() * phi.sin();
        let z = theta.cos();

        let random_point = Vector3::new(x, y, z);

        let axis = random_point.cross(&n);
        let angle = random_point.dot(&n).acos();

        Rotation3::from_axis_angle(&Unit::new_unchecked(axis), angle)
            .transform_vector(&random_point)
            .normalize()
    }
}
