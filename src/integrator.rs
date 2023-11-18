use nalgebra::Vector3;

use crate::{scene::Scene, Ray, Spectrum};

pub trait Integrator {
    fn lo(&self, scene: &Scene, ray: Ray) -> Spectrum;
}

#[derive(Default)]
pub struct WhittedIntegrator {
    max_depth: u32,
}

impl WhittedIntegrator {
    pub fn new(max_depth: u32) -> Self {
        Self { max_depth }
    }

    fn li(&self, scene: &Scene, ray: Ray, depth: u32) -> Spectrum {
        let mut l = Spectrum::BLACK;
        if let Some(isect) = scene.intersect(ray) {
            // Determine local color due to direct illumination.
            for light in scene.lights_iter() {
                // Check for shadows by sending a ray to the light.
                let shadow_ray = isect.shadow_ray(light);
                let wi_dot_n = Vector3::dot(&shadow_ray.d, &isect.n);
                if wi_dot_n > 0.0 && scene.intersect(shadow_ray).is_none() {
                    // Only shades the intersection if not in shadow.
                    l += Spectrum::BLUE * light.li() * wi_dot_n;
                }
            }

            if depth > 0 {
                // Determine color from a ray comming from the reflection direction.
                l += self.li(scene, isect.reflect_ray(), depth - 1);
            }
        }
        l
    }
}

impl Integrator for WhittedIntegrator {
    fn lo(&self, scene: &Scene, ray: Ray) -> Spectrum {
        self.li(&scene, ray, self.max_depth)
    }
}
