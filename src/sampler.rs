use nalgebra::Point2;
use rand::Rng;

pub trait Sampler {
    fn get_sample2d(&self) -> Point2<f32>;
}

pub struct RandomSampler;

impl Sampler for RandomSampler {
    fn get_sample2d(&self) -> Point2<f32> {
        let mut rng = rand::thread_rng();

        Point2::<f32>::new(rng.gen(), rng.gen())
    }
}
