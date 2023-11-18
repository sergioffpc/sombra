use nalgebra::Vector3;

use crate::{scene::SurfaceInteraction, Spectrum};

pub trait Reflection {
    fn f(&self, isect: SurfaceInteraction, wi: Vector3<f32>) -> Spectrum;
}

pub struct LambertianReflection {
    s: Spectrum,
}

impl Reflection for LambertianReflection {
    fn f(&self, _isect: SurfaceInteraction, _wi: Vector3<f32>) -> Spectrum {
        self.s * std::f32::consts::FRAC_1_PI
    }
}

pub struct SpecularReflection;

impl Reflection for SpecularReflection {
    fn f(&self, isect: SurfaceInteraction, wi: Vector3<f32>) -> Spectrum {
        let ds = wi - 2.0 * isect.n;

        ds.into()
    }
}
