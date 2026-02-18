use std::{
    f32,
    fmt::Debug,
    path::{Path, PathBuf},
};

use clap::Parser;
use indicatif::{ParallelProgressIterator, ProgressBar, ProgressStyle};
use rand::RngExt;
use rayon::{
    iter::{IndexedParallelIterator, ParallelIterator},
    slice::ParallelSliceMut,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    pub o: nalgebra::Point3<f32>,
    pub d: nalgebra::Vector3<f32>,
}

impl Ray {
    pub fn new(o: nalgebra::Point3<f32>, d: nalgebra::Vector3<f32>) -> Self {
        Self { o, d }
    }

    pub fn at(&self, t: f32) -> nalgebra::Point3<f32> {
        self.o + self.d * t
    }

    pub fn transform(&self, m: &nalgebra::Matrix4<f32>) -> Self {
        let o = m.transform_point(&self.o);
        let d = m.transform_vector(&self.d);
        Ray::new(o, d)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Camera {
    viewport: nalgebra::Point2<f32>,
    camera_to_world: nalgebra::Matrix4<f32>,
    world_to_camera: nalgebra::Matrix4<f32>,
}

impl Camera {
    pub fn new(aspect: f32, fovy: f32, camera_to_world: nalgebra::Matrix4<f32>) -> Self {
        let h = 2.0 * f32::tan(fovy / 2.0);
        let w = aspect * h;
        let viewport = nalgebra::Point2::new(w, h);

        Self {
            viewport,
            camera_to_world,
            world_to_camera: camera_to_world.try_inverse().unwrap(),
        }
    }

    pub fn ray(&self, u: f32, v: f32) -> Ray {
        let x = (u - 0.5) * self.viewport.x;
        let y = (0.5 - v) * self.viewport.y;
        let z = -1.0;

        Ray::new(nalgebra::Point3::origin(), nalgebra::Vector3::new(x, y, z))
            .transform(&self.camera_to_world)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Intersection {
    pub p: nalgebra::Point3<f32>,
    pub n: nalgebra::UnitVector3<f32>,
    pub t: f32,
}

pub trait Shape: Debug + Sync {
    fn intersect(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<Intersection>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphereShape;

impl Shape for SphereShape {
    #[inline(always)]
    fn intersect(&self, r: &Ray, t_min: f32, t_max: f32) -> Option<Intersection> {
        let a = nalgebra::Vector3::dot(&r.d, &r.d);
        let half_b = nalgebra::Vector3::dot(&r.o.coords, &r.d);
        let c = nalgebra::Vector3::dot(&r.o.coords, &r.o.coords) - 1.0;

        let (t0, t1) = quadratic_solver(a, half_b, c)?;
        let t = if t0 >= t_min && t0 <= t_max {
            t0
        } else if t1 >= t_min && t1 <= t_max {
            t1
        } else {
            return None;
        };

        let p = r.at(t);
        let n = nalgebra::UnitVector3::new_normalize(p.coords);
        Some(Intersection { p, n, t })
    }
}

#[inline(always)]
pub fn quadratic_solver(a: f32, half_b: f32, c: f32) -> Option<(f32, f32)> {
    let discrim = half_b * half_b - a * c;
    if discrim < 0.0 {
        return None;
    }

    let root = discrim.sqrt();
    let mut t0 = (-half_b - root) / a;
    let mut t1 = (-half_b + root) / a;

    if t0 > t1 {
        std::mem::swap(&mut t0, &mut t1);
    }

    Some((t0, t1))
}

pub trait Sampler {
    fn sample(&self, n: &nalgebra::UnitVector3<f32>) -> nalgebra::UnitVector3<f32>;
}

/// Generates random directions uniformly distributed over the hemisphere
/// oriented by a given surface normal. It uses rejection sampling inside
/// a unit sphere and only keeps directions with a positive dot product
/// with the normal.
#[derive(Debug)]
pub struct UniformHemisphereSampler;

impl Sampler for UniformHemisphereSampler {
    fn sample(&self, n: &nalgebra::UnitVector3<f32>) -> nalgebra::UnitVector3<f32> {
        let mut rng = rand::rng();
        loop {
            // Generate a random vector inside the cube [-1,1]^3
            let v = nalgebra::Vector3::new(rng.random(), rng.random(), rng.random()) * 2.0
                - nalgebra::Vector3::repeat(1.0);
            // Keep only vectors inside the unit sphere
            if v.norm_squared() < 1.0 {
                // Normalize the vector to get a direction
                let d = nalgebra::UnitVector3::new_normalize(v);

                // Only accept vectors in the same hemisphere as the given normal
                if d.dot(n) > 0.0 {
                    return d;
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BsdfSample {
    pub wi: nalgebra::UnitVector3<f32>,
    pub attenuation: Spectrum,
}

pub trait Material: Debug + Sync {
    fn sample(&self, si: &SurfaceInteraction, sampler: &dyn Sampler) -> Option<BsdfSample>;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LambertianMaterial {
    albedo: Spectrum,
}

impl LambertianMaterial {
    pub fn new(albedo: Spectrum) -> Self {
        Self { albedo }
    }
}

impl Material for LambertianMaterial {
    fn sample(&self, si: &SurfaceInteraction, sampler: &dyn Sampler) -> Option<BsdfSample> {
        let wi = sampler.sample(&si.n);
        Some(BsdfSample {
            wi,
            attenuation: self.albedo,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct LightSample {
    pub wi: nalgebra::UnitVector3<f32>,
    pub radiance: Spectrum,
    pub t: f32,
}

pub trait Light: Debug + Sync {
    fn sample(&self, si: &SurfaceInteraction, sampler: &dyn Sampler) -> Option<LightSample>;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PointLight {
    intensity: Spectrum,
}

impl PointLight {
    pub fn new(intensity: Spectrum) -> Self {
        Self { intensity }
    }
}

impl Light for PointLight {
    fn sample(
        &self,
        interaction: &SurfaceInteraction,
        _sampler: &dyn Sampler,
    ) -> Option<LightSample> {
        let v = nalgebra::Point3::origin() - interaction.p;
        let t = v.norm();
        if t == 0.0 {
            return None;
        }

        let wi = nalgebra::UnitVector3::new_normalize(v);
        let i = self.intensity / (t * t);
        Some(LightSample { wi, radiance: i, t })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Spectrum {
    pub r: f32,
    pub g: f32,
    pub b: f32,
}

impl Spectrum {
    pub const BLACK: Self = Self::splat(0.0);
    pub const WHITE: Self = Self::splat(1.0);

    pub const RED: Self = Self::new(1.0, 0.0, 0.0);
    pub const GREEN: Self = Self::new(0.0, 1.0, 0.0);
    pub const BLUE: Self = Self::new(0.0, 0.0, 1.0);

    #[inline(always)]
    pub const fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b }
    }

    #[inline(always)]
    pub const fn splat(v: f32) -> Self {
        Self { r: v, g: v, b: v }
    }

    #[inline(always)]
    pub fn is_black(&self) -> bool {
        self.r == 0.0 && self.g == 0.0 && self.b == 0.0
    }

    #[inline(always)]
    pub fn clamp(&self, min: f32, max: f32) -> Self {
        Self {
            r: self.r.clamp(min, max),
            g: self.g.clamp(min, max),
            b: self.b.clamp(min, max),
        }
    }

    #[inline(always)]
    pub fn sqrt(&self) -> Self {
        Self {
            r: self.r.sqrt(),
            g: self.g.sqrt(),
            b: self.b.sqrt(),
        }
    }
}

impl std::ops::Add<Spectrum> for Spectrum {
    type Output = Self;

    #[inline(always)]
    fn add(self, rhs: Spectrum) -> Self::Output {
        Self::new(self.r + rhs.r, self.g + rhs.g, self.b + rhs.b)
    }
}

impl std::ops::AddAssign<Spectrum> for Spectrum {
    #[inline(always)]
    fn add_assign(&mut self, rhs: Spectrum) {
        *self = *self + rhs
    }
}

impl std::ops::Mul<f32> for Spectrum {
    type Output = Self;

    #[inline(always)]
    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.r * rhs, self.g * rhs, self.b * rhs)
    }
}

impl std::ops::MulAssign<f32> for Spectrum {
    #[inline(always)]
    fn mul_assign(&mut self, rhs: f32) {
        *self = *self * rhs
    }
}

impl std::ops::Mul<Spectrum> for Spectrum {
    type Output = Self;

    #[inline(always)]
    fn mul(self, rhs: Spectrum) -> Self::Output {
        Self::new(self.r * rhs.r, self.g * rhs.g, self.b * rhs.b)
    }
}

impl std::ops::MulAssign<Spectrum> for Spectrum {
    #[inline(always)]
    fn mul_assign(&mut self, rhs: Spectrum) {
        *self = *self * rhs
    }
}

impl std::ops::Div<f32> for Spectrum {
    type Output = Self;

    #[inline(always)]
    fn div(self, rhs: f32) -> Self::Output {
        let inv = 1.0 / rhs;
        Self::new(self.r * inv, self.g * inv, self.b * inv)
    }
}

impl std::ops::DivAssign<f32> for Spectrum {
    #[inline(always)]
    fn div_assign(&mut self, rhs: f32) {
        *self = *self / rhs
    }
}

impl std::ops::Div<Spectrum> for Spectrum {
    type Output = Self;

    #[inline(always)]
    fn div(self, rhs: Spectrum) -> Self::Output {
        Self::new(self.r / rhs.r, self.g / rhs.g, self.b / rhs.b)
    }
}

impl std::ops::DivAssign<Spectrum> for Spectrum {
    #[inline(always)]
    fn div_assign(&mut self, rhs: Spectrum) {
        *self = *self / rhs
    }
}

impl std::ops::Sub<Spectrum> for Spectrum {
    type Output = Self;

    #[inline(always)]
    fn sub(self, rhs: Spectrum) -> Self::Output {
        Self::new(self.r - rhs.r, self.g - rhs.g, self.b - rhs.b)
    }
}

impl std::ops::SubAssign<Spectrum> for Spectrum {
    #[inline(always)]
    fn sub_assign(&mut self, rhs: Spectrum) {
        *self = *self - rhs
    }
}

#[derive(Clone, Debug)]
pub struct Film {
    width: u32,
    height: u32,
    pixels: Vec<Pixel>,
}

impl Film {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            pixels: vec![Pixel::default(); (width * height) as usize],
        }
    }

    pub fn scanlines<OP>(&mut self, op: OP)
    where
        OP: Fn(usize, &mut [Pixel], u32, u32) + Sync + Send,
    {
        let progress = ProgressBar::new(self.height as u64);
        progress.set_style(
            ProgressStyle::with_template(
                "[{elapsed_precise}] {bar:40.cyan/blue} {pos}/{len} scanlines ({eta})",
            )
            .unwrap()
            .progress_chars("█▉▊▋▌▍▎▏ "),
        );
        self.pixels
            .par_chunks_mut(self.width as usize)
            .progress_with(progress.clone())
            .enumerate()
            .for_each(|(y, row)| op(y, row, self.width, self.height));
        progress.finish();
    }

    pub fn write<P>(&self, path: P) -> anyhow::Result<()>
    where
        P: AsRef<Path>,
    {
        let mut imgbuf = image::ImageBuffer::new(self.width, self.height);
        for (x, y, rgb) in imgbuf.enumerate_pixels_mut() {
            let index = y * self.width + x;
            let sample = self.pixels[index as usize].gamma_corrected();
            *rgb = image::Rgb([sample.r, sample.g, sample.b]);
        }
        imgbuf
            .save_with_format(path, image::ImageFormat::OpenExr)
            .map_err(Into::into)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Pixel {
    acc: Spectrum,
    count: usize,
}

impl Pixel {
    #[inline(always)]
    pub fn add_sample(&mut self, sample: Spectrum) {
        self.acc += sample;
        self.count += 1;
    }

    #[inline(always)]
    pub fn mean(&self) -> Spectrum {
        if self.count > 0 {
            self.acc / self.count as f32
        } else {
            Spectrum::BLACK
        }
    }

    #[inline(always)]
    pub fn gamma_corrected(&self) -> Spectrum {
        self.mean().sqrt()
    }
}

#[derive(Debug)]
pub struct GeometricPrimitive {
    shape: Box<dyn Shape>,
    material: Box<dyn Material>,
    object_to_world: nalgebra::Matrix4<f32>,
    normal_to_world: nalgebra::Matrix4<f32>,
    world_to_object: nalgebra::Matrix4<f32>,
}

impl GeometricPrimitive {
    pub fn new<S, M>(shape: S, material: M, object_to_world: nalgebra::Matrix4<f32>) -> Self
    where
        S: Shape + 'static,
        M: Material + 'static,
    {
        let world_to_object = object_to_world.try_inverse().unwrap();
        let normal_to_world = world_to_object.transpose();
        Self {
            shape: Box::new(shape),
            material: Box::new(material),
            object_to_world,
            normal_to_world,
            world_to_object,
        }
    }

    pub fn intersect(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<Intersection> {
        let inv_ray = ray.transform(&self.world_to_object);
        self.shape
            .intersect(&inv_ray, t_min, t_max)
            .map(|intersection| {
                let p = self.object_to_world.transform_point(&intersection.p);
                let n = nalgebra::UnitVector3::new_normalize(
                    self.normal_to_world
                        .transform_vector(intersection.n.as_ref()),
                );
                let t = (p - ray.o).dot(&ray.d) / ray.d.dot(&ray.d);

                Intersection { p, n, t }
            })
    }
}

#[derive(Debug)]
pub struct LightPrimitive {
    light: Box<dyn Light>,

    light_to_world: nalgebra::Matrix4<f32>,
    world_to_light: nalgebra::Matrix4<f32>,
}

impl LightPrimitive {
    pub fn new<L>(light: L, light_to_world: nalgebra::Matrix4<f32>) -> Self
    where
        L: Light + 'static,
    {
        let world_to_light = light_to_world.try_inverse().unwrap();
        Self {
            light: Box::new(light),
            light_to_world,
            world_to_light,
        }
    }

    pub fn sample(
        &self,
        interaction: &SurfaceInteraction,
        sampler: &dyn Sampler,
    ) -> Option<LightSample> {
        let interaction = SurfaceInteraction {
            p: self.world_to_light.transform_point(&interaction.p),
            ..*interaction
        };
        self.light
            .sample(&interaction, sampler)
            .map(|sample| LightSample {
                wi: nalgebra::UnitVector3::new_normalize(
                    self.light_to_world
                        .transform_vector(&sample.wi.into_inner()),
                ),
                ..sample
            })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SurfaceInteraction<'a> {
    pub p: nalgebra::Point3<f32>,
    pub n: nalgebra::UnitVector3<f32>,
    pub wo: nalgebra::UnitVector3<f32>,
    pub t: f32,
    pub primitive: &'a GeometricPrimitive,
}

#[derive(Debug, Default)]
pub struct Scene {
    geometries: Vec<GeometricPrimitive>,
    lights: Vec<LightPrimitive>,
}

impl Scene {
    pub fn add_geometric_primitive(&mut self, primitive: GeometricPrimitive) {
        self.geometries.push(primitive);
    }

    pub fn add_light_primitive(&mut self, primitive: LightPrimitive) {
        self.lights.push(primitive);
    }

    pub fn intersect<'a>(
        &'a self,
        ray: &Ray,
        t_min: f32,
        t_max: f32,
    ) -> Option<SurfaceInteraction<'a>> {
        let mut closest_t = t_max;
        let mut closest_si = None;
        for primitive in self.geometries.iter() {
            if let Some(intersection) = primitive.intersect(ray, t_min, closest_t) {
                closest_t = intersection.t;
                closest_si = Some(SurfaceInteraction {
                    p: intersection.p,
                    n: intersection.n,
                    wo: nalgebra::UnitVector3::new_normalize(-ray.d),
                    t: intersection.t,
                    primitive,
                });
            }
        }
        closest_si
    }
}

pub trait Integrator: Debug + Sync {
    fn li(&self, ray: &Ray, scene: &Scene, sampler: &dyn Sampler) -> Spectrum;
}

#[derive(Clone, Copy, Debug)]
pub struct WhittedIntegrator {
    max_depth: usize,
}

impl WhittedIntegrator {
    fn radiance(&self, ray: &Ray, scene: &Scene, sampler: &dyn Sampler, depth: usize) -> Spectrum {
        if depth == 0 {
            return Spectrum::BLACK;
        }

        let mut total_radiance = Spectrum::BLACK;
        if let Some(si) = scene.intersect(ray, f32::EPSILON, f32::MAX)
            && let Some(bsdf_sample) = si.primitive.material.sample(&si, sampler)
        {
            let indirect_ray = Ray::new(si.p, bsdf_sample.wi.into_inner());
            let indirect_radiance = self.radiance(&indirect_ray, scene, sampler, depth - 1);

            total_radiance = bsdf_sample.attenuation * indirect_radiance;
            for light in scene.lights.iter() {
                if let Some(li_sample) = light.sample(&si, sampler) {
                    let shadow_ray = Ray::new(si.p, li_sample.wi.into_inner());
                    if scene
                        .intersect(&shadow_ray, f32::EPSILON, li_sample.t - f32::EPSILON)
                        .is_none()
                    {
                        let cos_theta = f32::max(0.0, li_sample.wi.into_inner().dot(&si.n));
                        total_radiance += li_sample.radiance * cos_theta;
                    }
                }
            }
        }
        total_radiance
    }
}

impl Integrator for WhittedIntegrator {
    fn li(&self, ray: &Ray, scene: &Scene, sampler: &dyn Sampler) -> Spectrum {
        self.radiance(ray, scene, sampler, self.max_depth)
    }
}

impl Default for WhittedIntegrator {
    fn default() -> Self {
        Self { max_depth: 3 }
    }
}

#[derive(Parser, Debug)]
#[command(author, version, about)]
pub struct Args {
    #[arg(long, default_value_t = 1280)]
    pub width: u32,

    #[arg(long, default_value_t = 720)]
    pub height: u32,

    #[arg(long, default_value_t = 16)]
    pub spp: u32,

    #[arg(short, long, default_value = "output.exr")]
    pub output: PathBuf,
}

fn main() {
    let args = Args::parse();

    let eye = nalgebra::Point3::<f32>::new(0.0, 0.0, 2.0);
    let target = nalgebra::Point3::origin();
    let up = nalgebra::Vector3::y();
    let lookat = nalgebra::Isometry3::look_at_rh(&eye, &target, &up);
    let camera_to_world = lookat.inverse().to_homogeneous();
    let camera = Camera::new(16.0 / 9.0, 60f32.to_radians(), camera_to_world);

    let mut scene = Scene::default();
    scene.add_geometric_primitive(GeometricPrimitive::new(
        SphereShape,
        LambertianMaterial::new(Spectrum::BLUE),
        nalgebra::Matrix4::identity(),
    ));
    scene.add_light_primitive(LightPrimitive::new(
        PointLight::new(Spectrum::WHITE),
        nalgebra::Translation3::new(0.0, 2.0, 2.0).to_homogeneous(),
    ));

    let integrator = WhittedIntegrator::default();
    let sampler = UniformHemisphereSampler;
    let mut film = Film::new(args.width, args.height);
    film.scanlines(|y, row, width, height| {
        let mut rng = rand::rng();
        for (x, pixel) in row.iter_mut().enumerate() {
            for _ in 0..args.spp {
                let du: f32 = rng.random();
                let dv: f32 = rng.random();
                let u = (x as f32 + du) / (width - 1) as f32;
                let v = (y as f32 + dv) / (height - 1) as f32;
                let camera_ray = camera.ray(u, v);

                let radiance = integrator.li(&camera_ray, &scene, &sampler);
                pixel.add_sample(radiance);
            }
        }
    });
    film.write(args.output).unwrap()
}
