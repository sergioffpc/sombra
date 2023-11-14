use nalgebra::{Matrix4, Point2, Point3, Vector3};
use openexr::{
    core::header::Header,
    rgba::{Rgba, RgbaChannels, RgbaOutputFile},
};

pub mod camera;
pub mod scene;
pub mod shape;

#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub o: Point3<f32>,
    pub d: Vector3<f32>,
    pub t_max: f32,
}

impl Ray {
    pub fn new(o: Point3<f32>, d: Vector3<f32>) -> Self {
        Self {
            o,
            d,
            t_max: std::f32::MAX,
        }
    }

    pub fn position(&self, t: f32) -> Point3<f32> {
        self.o + t * self.d
    }

    pub fn transform(&self, m: Matrix4<f32>) -> Self {
        Self {
            o: m.transform_point(&self.o),
            d: m.transform_vector(&self.d),
            t_max: self.t_max,
        }
    }
}

pub fn quadratic_solver(a: f32, b: f32, c: f32) -> Option<(f32, f32)> {
    match b * b - 4.0 * a * c {
        discrim if discrim >= 0.0 => {
            let sqrt_discrim = discrim.sqrt();
            let t1 = (-b - sqrt_discrim) / (2.0 * a);
            let t2 = (-b + sqrt_discrim) / (2.0 * a);

            Some((t1.min(t2), t1.max(t2)))
        }
        _ => None,
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Spectrum {
    pub r: f32,
    pub g: f32,
    pub b: f32,
}

impl Spectrum {
    pub const BLACK: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 0.0,
    };
    pub const RED: Self = Self {
        r: 1.0,
        g: 0.0,
        b: 0.0,
    };
    pub const GREEN: Self = Self {
        r: 0.0,
        g: 1.0,
        b: 0.0,
    };
    pub const BLUE: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 1.0,
    };

    pub fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b }
    }
}

pub struct Film {
    pub resolution: Point2<i32>,

    pixels: Vec<Spectrum>,
}

impl Film {
    pub fn new(resolution: Point2<i32>) -> Self {
        Self {
            resolution,
            pixels: vec![Spectrum::BLACK; (resolution.x * resolution.y) as usize],
        }
    }

    pub fn add_sample(&mut self, p: Point2<i32>, s: Spectrum) {
        let index = p.y * self.resolution.x + p.x;
        self.pixels[index as usize] = s;
    }

    pub fn write(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        let header = Header::from_dimensions(self.resolution.x, self.resolution.y);
        let mut file = RgbaOutputFile::new(filename, &header, RgbaChannels::WriteRgba, 1)?;

        file.set_frame_buffer(
            self.pixels
                .iter()
                .map(|s| Rgba::from_f32(s.r, s.g, s.b, 1.0))
                .collect::<Vec<_>>()
                .as_slice(),
            1,
            self.resolution.x as usize,
        )?;
        unsafe {
            file.write_pixels(self.resolution.y)?;
        }

        Ok(())
    }
}
