use nalgebra::{Point3, Vector3};

use crate::{
    scene::{Interaction, Scene},
    Ray, Spectrum,
};

pub trait Integrator {
    fn lo(&self, scene: &Scene, isect: Interaction) -> Spectrum;
}

#[derive(Default)]
pub struct WhittedIntegrator;

impl Integrator for WhittedIntegrator {
    fn lo(&self, scene: &Scene, isect: Interaction) -> Spectrum {
        let mut l = Spectrum::BLACK;
        for light in scene.lights_iter() {
            // Check for shadows by sending a ray to the light.
            let light_p = light
                .get_object_to_world()
                .transform_point(&Point3::new(0.0, 0.0, 0.0));
            let hit_to_light = light_p - isect.p;
            let wi = hit_to_light.normalize();

            let mut shadow_ray = Ray::new(isect.p, wi);
            shadow_ray.t_max = hit_to_light.norm();
            if Vector3::dot(&wi, &isect.n) > 0.0 && scene.intersect(shadow_ray).is_none() {
                // Only shades the intersection if not in shadow.
                l += Spectrum::BLUE;
            }
        }

        l
    }
}
