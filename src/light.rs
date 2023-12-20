use crate::Spectrum;

pub trait Light {
    fn i(&self) -> Spectrum;
}

pub struct PointLight {
    i: Spectrum,
}

impl PointLight {
    pub fn new(i: Spectrum) -> Self {
        Self { i }
    }
}

impl Light for PointLight {
    fn i(&self) -> Spectrum {
        self.i
    }
}

impl Default for PointLight {
    fn default() -> Self {
        Self {
            i: Spectrum::WHITE * std::f32::consts::PI,
        }
    }
}
