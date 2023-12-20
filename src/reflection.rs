use nalgebra::Vector3;

use crate::{sampler::Sampler, scene::SurfaceInteraction, Spectrum};

pub trait Reflection {
    fn f(&self, isect: &SurfaceInteraction, wi: Vector3<f32>) -> Spectrum;

    fn sample_f(
        &self,
        isect: &SurfaceInteraction,
        sampler: &dyn Sampler,
    ) -> (Spectrum, Vector3<f32>);
}

#[derive(Default)]
pub struct LambertianReflection {
    r: Spectrum,
}

impl LambertianReflection {
    pub fn new(r: Spectrum) -> Self {
        Self { r }
    }
}

impl Reflection for LambertianReflection {
    fn f(&self, _isect: &SurfaceInteraction, _wi: Vector3<f32>) -> Spectrum {
        self.r * std::f32::consts::FRAC_1_PI
    }

    fn sample_f(
        &self,
        isect: &SurfaceInteraction,
        sampler: &dyn Sampler,
    ) -> (Spectrum, Vector3<f32>) {
        // θ (theta) is the inclination angle, ranging from 0 to π/2 for a hemisphere.
        let theta = sampler.next() * std::f32::consts::FRAC_PI_2;
        // φ (phi) is the azimuth angle, ranging from 0 to 2π.
        let phi = sampler.next() * 2.0 * std::f32::consts::PI;

        let x = theta.sin() * phi.cos();
        let y = theta.sin() * phi.sin();
        let z = theta.cos();

        let mut wi = Vector3::new(x, y, z).normalize();
        if wi.dot(&isect.n) < 0.0 {
            wi *= -1.0;
        }

        (self.f(isect, wi), wi)
    }
}

#[derive(Default)]
pub struct SpecularReflection {
    r: Spectrum,
}

impl SpecularReflection {
    pub fn new(r: Spectrum) -> Self {
        Self { r }
    }
}

impl Reflection for SpecularReflection {
    fn f(&self, _isect: &SurfaceInteraction, _wi: Vector3<f32>) -> Spectrum {
        Spectrum::BLACK
    }

    fn sample_f(
        &self,
        isect: &SurfaceInteraction,
        _sampler: &dyn Sampler,
    ) -> (Spectrum, Vector3<f32>) {
        let wi = 2.0 * isect.n.dot(&isect.wo) * isect.n - isect.wo;

        (self.r, wi)
    }
}
