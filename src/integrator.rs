use crate::{sampler::Sampler, scene::Scene, Ray, Spectrum};
pub trait Integrator {
    fn lo(&self, scene: &Scene, ray: Ray, sampler: &dyn Sampler) -> Spectrum;
}

#[derive(Default)]
pub struct WhittedIntegrator {
    max_depth: u32,
}

impl WhittedIntegrator {
    pub fn new(max_depth: u32) -> Self {
        Self { max_depth }
    }

    fn li(scene: &Scene, ray: Ray, sampler: &dyn Sampler, depth: u32) -> Spectrum {
        let mut l = Spectrum::BLACK;
        if let Some(isect) = scene.intersect(ray) {
            l += isect.geometry.le(&isect);

            let bxdf = isect.geometry.bxdf();
            for light in scene.lights_iter() {
                if isect.is_visible(scene, light) {
                    let (li, wi) = light.li(&isect);
                    let wi_dot_n = wi.dot(&isect.n);
                    if wi_dot_n > 0.0 {
                        let f = bxdf.f(&isect, wi);
                        l += f * li * wi_dot_n;
                    }
                }
            }

            if depth > 0 {
                let (f, wi) = bxdf.sample_f(&isect, sampler);
                let ray = isect.spawn_ray(wi);
                let li = Self::li(scene, ray, sampler, depth - 1);
                l += f * li;
            }
        }
        l
    }
}

impl Integrator for WhittedIntegrator {
    fn lo(&self, scene: &Scene, ray: Ray, sampler: &dyn Sampler) -> Spectrum {
        Self::li(scene, ray, sampler, self.max_depth)
    }
}
