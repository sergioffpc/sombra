use nalgebra::Vector3;

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

            // Determine local color due to direct illumination.
            for light in scene.lights_iter() {
                // Check for shadows by sending a ray to the light.
                let hit_to_light = light.position() - isect.p;
                let wi = hit_to_light.normalize();
                let mut shadow_ray = Ray::new(isect.p + isect.n * std::f32::EPSILON, wi);
                shadow_ray.t_max = hit_to_light.norm();

                let wi_dot_n = Vector3::dot(&wi, &isect.n);
                if wi_dot_n > 0.0 && scene.intersect(shadow_ray).is_none() {
                    // Only shades the intersection if not in shadow.
                    l += isect.geometry.f(&isect, wi) * light.li(&isect) * wi_dot_n;
                }
            }

            if depth > 0 {
                // Determine color from a ray comming from the reflection direction.
                let sample_wi = sampler.sample_hemisphere(isect.n);
                let reflect_ray = Ray::new(isect.p + isect.n * std::f32::EPSILON, sample_wi);

                l += Self::li(scene, reflect_ray, sampler, depth - 1);
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
