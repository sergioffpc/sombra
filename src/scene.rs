use std::{slice::Iter, sync::Arc};

use nalgebra::{Matrix4, Point3, Vector3};

use crate::{light::Light, reflection::BxDF, reflection::Reflection, shape::Shape, Ray, Spectrum};

pub struct SurfaceInteraction {
    pub p: Point3<f32>,
    pub n: Vector3<f32>,
    pub t: f32,
    pub wo: Vector3<f32>,
    pub geometry: Arc<GeometryPrimitive>,
}

pub struct GeometryPrimitive {
    shape: Box<dyn Shape>,
    bxdf: BxDF,
    object_to_world: Matrix4<f32>,
    world_to_object: Matrix4<f32>,
}

impl GeometryPrimitive {
    pub fn new(shape: Box<dyn Shape>, bxdf: BxDF) -> Self {
        Self {
            shape,
            bxdf,
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

    pub fn f(&self, isect: &SurfaceInteraction, wi: Vector3<f32>) -> Spectrum {
        self.bxdf.f(isect, wi)
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

    pub fn li(&self, _isect: &SurfaceInteraction) -> Spectrum {
        self.light.i()
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

                Some(SurfaceInteraction {
                    p: geometry.get_object_to_world().transform_point(&p),
                    n: geometry
                        .get_world_to_object()
                        .transpose()
                        .transform_vector(&n),
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
