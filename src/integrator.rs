use nalgebra::Point3;

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
            let lp = light
                .get_object_to_world()
                .transform_point(&Point3::new(0.0, 0.0, 0.0));
            let lv = lp - isect.p;

            let mut shadow_ray = Ray::new(isect.p, lv.normalize());
            shadow_ray.t_max = lv.norm();
            if scene.intersect(shadow_ray).is_none() {
                // Only shades the intersection if not in shadow.
                l += Spectrum::BLUE;
            }
        }

        l
    }
}
