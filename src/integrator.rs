use crate::{sampler::Sampler, scene::Scene, Ray, Spectrum};
pub trait Integrator {
    fn lo(&self, scene: &Scene, ray: Ray, sampler: &mut dyn Sampler) -> Spectrum;
}

#[derive(Default)]
pub struct WhittedIntegrator {
    max_depth: u32,
}

impl WhittedIntegrator {
    pub fn new(max_depth: u32) -> Self {
        Self { max_depth }
    }

    fn li(scene: &Scene, ray: Ray, sampler: &mut dyn Sampler, depth: u32) -> Spectrum {
        let mut l = Spectrum::BLACK;
        if let Some(isect) = scene.intersect(ray) {
            l += isect.geometry.le(&isect);

            let bxdf = isect.geometry.bxdf();
            for light in scene.lights_iter() {
                if isect.is_visible(scene, light) {
                    let (li, wi) = light.li(&isect);
                    let f = bxdf.f(&isect, wi);
                    l += f * li * wi.dot(&isect.n).abs();
                }
            }

            if depth > 0 {
                let (f, wi) = bxdf.sample_f(&isect, sampler);
                let li = Self::li(scene, isect.spawn_ray(wi), sampler, depth - 1);
                l += f * li * wi.dot(&isect.n).abs();
            }
        }
        l
    }
}

impl Integrator for WhittedIntegrator {
    fn lo(&self, scene: &Scene, ray: Ray, sampler: &mut dyn Sampler) -> Spectrum {
        Self::li(scene, ray, sampler, self.max_depth)
    }
}
