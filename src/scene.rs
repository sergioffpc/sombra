use std::{fmt::Display, slice::Iter, sync::Arc};

use nalgebra::{Matrix4, Point3, Vector3};

use crate::{
    light::Light, material::Material, reflection::Reflection, shape::Shape, Ray, Spectrum,
};

pub struct SurfaceInteraction {
    pub p: Point3<f32>,
    pub n: Vector3<f32>,
    pub t: f32,
    pub wo: Vector3<f32>,
    pub geometry: Arc<GeometryPrimitive>,
}

impl SurfaceInteraction {
    pub fn is_visible(&self, scene: &Scene, light: &LightPrimitive) -> bool {
        let hit_to_light = light.position() - self.p;
        let wi = hit_to_light.normalize();
        let mut shadow_ray = self.spawn_ray(wi);
        shadow_ray.t_max = hit_to_light.norm();

        scene.intersect(shadow_ray).is_none()
    }

    pub fn spawn_ray(&self, d: Vector3<f32>) -> Ray {
        Ray::new(self.p + self.n * f32::EPSILON, d)
    }
}

pub struct GeometryPrimitive {
    label: String,
    shape: Box<dyn Shape>,
    material: Box<dyn Material>,
    object_to_world: Matrix4<f32>,
    world_to_object: Matrix4<f32>,
}

impl GeometryPrimitive {
    pub fn new(label: &str, shape: Box<dyn Shape>, material: Box<dyn Material>) -> Self {
        Self {
            label: label.to_string(),
            shape,
            material,
            object_to_world: Matrix4::identity(),
            world_to_object: Matrix4::identity(),
        }
    }

    pub fn intersect(&self, r: Ray) -> Option<(Point3<f32>, Vector3<f32>, f32)> {
        self.shape.intersect(r.transform(self.world_to_object))
    }

    pub fn le(&self, _isect: &SurfaceInteraction) -> Spectrum {
        Spectrum::BLACK
    }

    pub fn bxdf(&self) -> &dyn Reflection {
        self.material.bxdf()
    }

    pub fn position(&self) -> Point3<f32> {
        self.get_object_to_world().column(3).xyz().into()
    }

    pub fn get_object_to_world(&self) -> Matrix4<f32> {
        self.object_to_world
    }

    pub fn set_object_to_world(&mut self, m: Matrix4<f32>) {
        self.object_to_world = m;
        self.world_to_object = m.try_inverse().unwrap();
    }

    pub fn get_world_to_object(&self) -> Matrix4<f32> {
        self.world_to_object
    }
}

impl Display for GeometryPrimitive {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.label)
    }
}

pub struct LightPrimitive {
    light: Box<dyn Light>,
    object_to_world: Matrix4<f32>,
    world_to_object: Matrix4<f32>,
}

impl LightPrimitive {
    pub fn new(light: Box<dyn Light>) -> Self {
        Self {
            light,
            object_to_world: Matrix4::identity(),
            world_to_object: Matrix4::identity(),
        }
    }

    pub fn li(&self, isect: &SurfaceInteraction) -> (Spectrum, Vector3<f32>) {
        let hit_to_light = self.position() - isect.p;
        let wi = hit_to_light.normalize();

        (self.light.i(), wi)
    }

    pub fn position(&self) -> Point3<f32> {
        self.get_object_to_world().column(3).xyz().into()
    }

    pub fn get_object_to_world(&self) -> Matrix4<f32> {
        self.object_to_world
    }

    pub fn set_object_to_world(&mut self, m: Matrix4<f32>) {
        self.object_to_world = m;
        self.world_to_object = m.try_inverse().unwrap();
    }

    pub fn get_world_to_object(&self) -> Matrix4<f32> {
        self.world_to_object
    }
}

#[derive(Default)]
pub struct Scene {
    geometries: Vec<Arc<GeometryPrimitive>>,
    lights: Vec<Arc<LightPrimitive>>,
}

impl Scene {
    pub fn add_geometry(&mut self, geometry: GeometryPrimitive) {
        self.geometries.push(Arc::new(geometry));
    }

    pub fn geometries_iter(&self) -> Iter<'_, Arc<GeometryPrimitive>> {
        self.geometries.iter()
    }

    pub fn add_light(&mut self, light: LightPrimitive) {
        self.lights.push(Arc::new(light));
    }

    pub fn lights_iter(&self) -> Iter<'_, Arc<LightPrimitive>> {
        self.lights.iter()
    }

    pub fn intersect(&self, mut r: Ray) -> Option<SurfaceInteraction> {
        self.geometries_iter().fold(None, |closest, geometry| {
            if let Some((p, n, t)) = geometry.intersect(r) {
                r.t_max = t;

                let mut inverse_transpose = geometry.get_world_to_object();
                inverse_transpose.fill_column(3, 0.0);
                inverse_transpose.transpose_mut();

                Some(SurfaceInteraction {
                    p: geometry.get_object_to_world().transform_point(&p),
                    n: inverse_transpose.transform_vector(&n).normalize(),
                    t,
                    wo: -r.d,
                    geometry: geometry.clone(),
                })
            } else {
                closest
            }
        })
    }
}
