use std::slice::Iter;

use nalgebra::{Matrix4, Point3, Vector3};

use crate::{light::Light, shape::Shape, Ray, Spectrum};

#[derive(Clone, Copy, Debug)]
pub struct Interaction {
    pub p: Point3<f32>,
    pub n: Vector3<f32>,
    pub t: f32,
    pub wo: Vector3<f32>,
}

impl Interaction {
    pub fn reflect_ray(&self) -> Ray {
        let wr = (-self.wo - 2.0 * Vector3::dot(&-self.wo, &self.n) * self.n).normalize();

        Ray::new(self.p + self.n * std::f32::EPSILON, wr)
    }

    pub fn shadow_ray(&self, light: &LightPrimitive) -> Ray {
        let light_p = light
            .get_object_to_world()
            .transform_point(&Point3::new(0.0, 0.0, 0.0));
        let hit_to_light = light_p - self.p;
        let wi = hit_to_light.normalize();

        let mut ray = Ray::new(self.p + self.n * std::f32::EPSILON, wi);
        ray.t_max = hit_to_light.norm();

        ray
    }
}

pub struct GeometryPrimitive {
    shape: Box<dyn Shape>,
    object_to_world: Matrix4<f32>,
    world_to_object: Matrix4<f32>,
}

impl GeometryPrimitive {
    pub fn new(shape: Box<dyn Shape>) -> Self {
        Self {
            shape,
            object_to_world: Matrix4::identity(),
            world_to_object: Matrix4::identity(),
        }
    }

    pub fn intersect(&self, r: Ray) -> Option<Interaction> {
        self.shape
            .intersect(r.transform(self.world_to_object))
            .map(|(p, n, t)| Interaction {
                p: self.object_to_world.transform_point(&p),
                n: self.world_to_object.transpose().transform_vector(&n),
                t,
                wo: -r.d,
            })
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

    pub fn i(&self) -> Spectrum {
        self.light.i()
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
    geometries: Vec<GeometryPrimitive>,
    lights: Vec<LightPrimitive>,
}

impl Scene {
    pub fn add_geometry(&mut self, geometry: GeometryPrimitive) {
        self.geometries.push(geometry);
    }

    pub fn geometries_iter(&self) -> Iter<'_, GeometryPrimitive> {
        self.geometries.iter()
    }

    pub fn add_light(&mut self, light: LightPrimitive) {
        self.lights.push(light);
    }

    pub fn lights_iter(&self) -> Iter<'_, LightPrimitive> {
        self.lights.iter()
    }

    pub fn intersect(&self, mut r: Ray) -> Option<Interaction> {
        self.geometries_iter().fold(None, |closest, geometry| {
            if let Some(isect) = geometry.intersect(r) {
                r.t_max = isect.t;

                Some(isect)
            } else {
                closest
            }
        })
    }
}
