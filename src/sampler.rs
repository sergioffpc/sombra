use rand::Rng;

pub trait Sampler {
    fn next(&mut self) -> f32;
}

#[derive(Default)]
pub struct RandomSampler {
    rng: rand::prelude::ThreadRng,
}

impl Sampler for RandomSampler {
    fn next(&mut self) -> f32 {
        self.rng.gen()
    }
}
