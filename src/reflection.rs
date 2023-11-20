use nalgebra::Vector3;

use crate::{scene::SurfaceInteraction, Spectrum};

pub trait Reflection {
    fn le(&self, isect: &SurfaceInteraction) -> Spectrum;
    fn f(&self, isect: &SurfaceInteraction, wi: Vector3<f32>) -> Spectrum;
}

#[derive(Default)]
pub struct BxDF {
    bxdfs: Vec<Box<dyn Reflection>>,
}

impl BxDF {
    pub fn add_bxdf(&mut self, bxdf: Box<dyn Reflection>) {
        self.bxdfs.push(bxdf);
    }
}

impl Reflection for BxDF {
    fn le(&self, isect: &SurfaceInteraction) -> Spectrum {
        self.bxdfs
            .iter()
            .fold(Spectrum::BLACK, |s, bxdf| s + bxdf.le(isect))
    }

    fn f(&self, isect: &SurfaceInteraction, wi: Vector3<f32>) -> Spectrum {
        self.bxdfs
            .iter()
            .fold(Spectrum::BLACK, |s, bxdf| s + bxdf.f(isect, wi))
    }
}

pub struct LambertianReflection {
    d: Spectrum,
}

impl LambertianReflection {
    pub fn new(d: Spectrum) -> Self {
        Self { d }
    }
}

impl Reflection for LambertianReflection {
    fn le(&self, _isect: &SurfaceInteraction) -> Spectrum {
        Spectrum::BLACK
    }

    fn f(&self, _isect: &SurfaceInteraction, _wi: Vector3<f32>) -> Spectrum {
        self.d * std::f32::consts::FRAC_1_PI
    }
}

pub struct SpecularReflection;

impl Reflection for SpecularReflection {
    fn le(&self, _isect: &SurfaceInteraction) -> Spectrum {
        Spectrum::BLACK
    }

    fn f(&self, _isect: &SurfaceInteraction, _wi: Vector3<f32>) -> Spectrum {
        Spectrum::WHITE
    }
}
