use crate::Spectrum;

pub trait Light {
    fn i(&self) -> Spectrum;
}

pub struct PointLight;

impl Light for PointLight {
    fn i(&self) -> Spectrum {
        Spectrum::WHITE
    }
}
