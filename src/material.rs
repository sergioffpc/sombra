use crate::reflection::SpecularReflection;
use crate::{
    reflection::{LambertianReflection, Reflection},
    Spectrum,
};

pub trait Material {
    fn bxdf(&self) -> &dyn Reflection;
}

#[derive(Default)]
pub struct MatteMaterial {
    lambertian: LambertianReflection,
}

impl MatteMaterial {
    pub fn new(r: Spectrum) -> Self {
        Self {
            lambertian: LambertianReflection::new(r),
        }
    }
}

impl Material for MatteMaterial {
    fn bxdf(&self) -> &dyn Reflection {
        &self.lambertian
    }
}

#[derive(Default)]
pub struct MirrorMaterial {
    specular: SpecularReflection,
}

impl MirrorMaterial {
    pub fn new(r: Spectrum) -> Self {
        Self {
            specular: SpecularReflection::new(r),
        }
    }
}

impl Material for MirrorMaterial {
    fn bxdf(&self) -> &dyn Reflection {
        &self.specular
    }
}
