use rand::Rng;

pub trait Sampler {
    fn next(&self) -> f32;
}

pub struct RandomSampler;

impl Sampler for RandomSampler {
    fn next(&self) -> f32 {
        let mut rng = rand::thread_rng();

        rng.gen()
    }
}
