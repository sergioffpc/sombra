use nalgebra::{Matrix4, Point3, Vector3};

use crate::{shape::Shape, Ray};

pub struct Interaction {
    pub p: Point3<f32>,
    pub n: Vector3<f32>,
    pub t: f32,
    pub wo: Vector3<f32>,
}

pub struct Geometry {
    shape: Box<dyn Shape>,
    object_to_world: Matrix4<f32>,
    world_to_object: Matrix4<f32>,
}

impl Geometry {
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
}

#[derive(Default)]
pub struct Scene {
    geometries: Vec<Geometry>,
}

impl Scene {
    pub fn add_geometry(&mut self, geometry: Geometry) {
        self.geometries.push(geometry);
    }

    pub fn intersect(&self, mut r: Ray) -> Option<Interaction> {
        self.geometries.iter().fold(None, |closest, geometry| {
            if let Some(isect) = geometry.intersect(r) {
                r.t_max = isect.t;

                Some(isect)
            } else {
                closest
            }
        })
    }
}
