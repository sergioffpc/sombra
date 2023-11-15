use crate::Spectrum;

pub trait Light {
    fn li(&self) -> Spectrum;
}

pub struct PointLight;

impl Light for PointLight {
    fn li(&self) -> Spectrum {
        Spectrum::WHITE
    }
}
