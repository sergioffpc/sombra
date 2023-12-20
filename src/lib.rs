use std::ops;

use nalgebra::{Matrix4, Point3, Vector3};

pub mod camera;
pub mod integrator;
pub mod light;
pub mod material;
pub mod reflection;
pub mod sampler;
pub mod scene;
pub mod shape;

#[derive(Clone, Copy, Debug, PartialEq)]
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
            t_max: f32::MAX,
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
        discriminant if discriminant >= 0.0 => {
            let sqrt_discriminant = discriminant.sqrt();
            let t1 = (-b - sqrt_discriminant) / (2.0 * a);
            let t2 = (-b + sqrt_discriminant) / (2.0 * a);

            Some((t1.min(t2), t1.max(t2)))
        }
        _ => None,
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
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
    pub const WHITE: Self = Self {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };

    pub fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b }
    }
}

impl ops::Add<Spectrum> for Spectrum {
    type Output = Spectrum;

    fn add(self, rhs: Spectrum) -> Self::Output {
        Self::Output {
            r: self.r + rhs.r,
            g: self.g + rhs.g,
            b: self.b + rhs.b,
        }
    }
}

impl ops::AddAssign<Spectrum> for Spectrum {
    fn add_assign(&mut self, rhs: Spectrum) {
        self.r += rhs.r;
        self.g += rhs.g;
        self.b += rhs.b;
    }
}

impl ops::Div<Spectrum> for Spectrum {
    type Output = Spectrum;

    fn div(self, rhs: Spectrum) -> Self::Output {
        Self::Output {
            r: self.r / rhs.r,
            g: self.g / rhs.g,
            b: self.b / rhs.b,
        }
    }
}

impl ops::DivAssign<Spectrum> for Spectrum {
    fn div_assign(&mut self, rhs: Spectrum) {
        self.r /= rhs.r;
        self.g /= rhs.g;
        self.b /= rhs.b;
    }
}

impl ops::Div<f32> for Spectrum {
    type Output = Spectrum;

    fn div(self, rhs: f32) -> Self::Output {
        Self::Output {
            r: self.r / rhs,
            g: self.g / rhs,
            b: self.b / rhs,
        }
    }
}

impl ops::DivAssign<f32> for Spectrum {
    fn div_assign(&mut self, rhs: f32) {
        self.r /= rhs;
        self.g /= rhs;
        self.b /= rhs;
    }
}

impl ops::Mul<Spectrum> for Spectrum {
    type Output = Spectrum;

    fn mul(self, rhs: Spectrum) -> Self::Output {
        Self::Output {
            r: self.r * rhs.r,
            g: self.g * rhs.g,
            b: self.b * rhs.b,
        }
    }
}

impl ops::MulAssign<Spectrum> for Spectrum {
    fn mul_assign(&mut self, rhs: Spectrum) {
        self.r *= rhs.r;
        self.g *= rhs.g;
        self.b *= rhs.b;
    }
}

impl ops::Mul<f32> for Spectrum {
    type Output = Spectrum;

    fn mul(self, rhs: f32) -> Self::Output {
        Self::Output {
            r: self.r * rhs,
            g: self.g * rhs,
            b: self.b * rhs,
        }
    }
}

impl ops::MulAssign<f32> for Spectrum {
    fn mul_assign(&mut self, rhs: f32) {
        self.r *= rhs;
        self.g *= rhs;
        self.b *= rhs;
    }
}

impl ops::Mul<Spectrum> for f32 {
    type Output = Spectrum;

    fn mul(self, rhs: Spectrum) -> Self::Output {
        Self::Output {
            r: self * rhs.r,
            g: self * rhs.g,
            b: self * rhs.b,
        }
    }
}

impl ops::Sub<Spectrum> for Spectrum {
    type Output = Spectrum;

    fn sub(self, rhs: Spectrum) -> Self::Output {
        Self::Output {
            r: self.r - rhs.r,
            g: self.g - rhs.g,
            b: self.b - rhs.b,
        }
    }
}

impl ops::SubAssign<Spectrum> for Spectrum {
    fn sub_assign(&mut self, rhs: Spectrum) {
        self.r -= rhs.r;
        self.g -= rhs.g;
        self.b -= rhs.b;
    }
}

impl From<Vector3<f32>> for Spectrum {
    fn from(value: Vector3<f32>) -> Self {
        Self {
            r: value.x,
            g: value.y,
            b: value.z,
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Scale3, Translation3, Vector3};

    use crate::Ray;

    #[test]
    fn test_ray_position() {
        assert_eq!(
            Ray::new(Point3::new(2.0, 3.0, 4.0), Vector3::new(1.0, 0.0, 0.0)).position(0.0),
            Point3::new(2.0, 3.0, 4.0)
        );

        assert_eq!(
            Ray::new(Point3::new(2.0, 3.0, 4.0), Vector3::new(1.0, 0.0, 0.0)).position(1.0),
            Point3::new(3.0, 3.0, 4.0)
        );

        assert_eq!(
            Ray::new(Point3::new(2.0, 3.0, 4.0), Vector3::new(1.0, 0.0, 0.0)).position(-1.0),
            Point3::new(1.0, 3.0, 4.0)
        );

        assert_eq!(
            Ray::new(Point3::new(2.0, 3.0, 4.0), Vector3::new(1.0, 0.0, 0.0)).position(2.5),
            Point3::new(4.5, 3.0, 4.0)
        );
    }

    #[test]
    fn test_ray_transform() {
        assert_eq!(
            Ray::new(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 1.0, 0.0))
                .transform(Translation3::new(3.0, 4.0, 5.0).to_homogeneous()),
            Ray::new(Point3::new(4.0, 6.0, 8.0), Vector3::new(0.0, 1.0, 0.0))
        );

        assert_eq!(
            Ray::new(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 1.0, 0.0))
                .transform(Scale3::new(2.0, 3.0, 4.0).to_homogeneous()),
            Ray::new(Point3::new(2.0, 6.0, 12.0), Vector3::new(0.0, 3.0, 0.0))
        );
    }
}
