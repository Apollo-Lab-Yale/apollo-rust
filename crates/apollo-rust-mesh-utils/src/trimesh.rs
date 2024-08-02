use std::collections::BTreeMap;
use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use nalgebra::{Vector3};
use parry3d_f64::transformation::convex_hull;
use parry3d_f64::transformation::vhacd::{VHACD, VHACDParameters};
use parry3d_f64::transformation::voxelization::FillMode;
use std::io::Write;
use base64::Engine;
use gltf::json;
use gltf_json::accessor::ComponentType::{F32, U32};
use gltf_json::accessor::GenericComponentType;
use gltf_json::validation::{Checked, USize64};
use parry3d_f64::math::Point;
use serde_json::json;

pub trait ToTriMesh {
    fn to_trimesh(&self) -> TriMesh;
}

#[derive(Clone, Debug)]
pub struct TriMesh {
    pub points: Vec<[f64;3]>,
    pub indices: Vec<[usize;3]>,
}
impl TriMesh {
    pub fn new_empty() -> Self {
        Self { points: vec![], indices: vec![] }
    }
    pub fn extend(&mut self, trimesh: &Self) {
        self.extend_from_points_and_indices(&trimesh.points, &trimesh.indices);
    }
    pub fn extend_from_points_and_indices(&mut self, new_points: &Vec<[f64; 3]>, new_indices: &Vec<[usize;3]>) {
        let points_len = self.points.len();
        let new_indices: Vec<[usize; 3]> = new_indices.iter().map(|x| [x[0] + points_len, x[1] + points_len, x[2] + points_len]).collect();
        self.points.extend(new_points);
        self.indices.extend(new_indices);
    }
    pub fn save_to_stl(&self, path: &PathBuf) {
        path.verify_extension(&vec!["stl", "STL"]).expect("error");

        let mut mesh = vec![];

        let triangles = self.to_triangles();
        triangles.iter().for_each(|t| {
            let v1 = t[0];
            let v2 = t[1];
            let v3 = t[2];
            let vv1 = Vector3::new(v1[0], v1[1], v1[2]);
            let vv2 = Vector3::new(v2[0], v2[1], v2[2]);
            let vv3 = Vector3::new(v3[0], v3[1], v3[2]);

            let a = vv2 - vv1;
            let b = vv3 - vv2;
            let n = a.cross(&b);

            let normal = stl_io::Normal::new([n[0] as f32, n[1] as f32, n[2] as f32]);

            let v1 = stl_io::Vertex::new( [v1[0] as f32, v1[1] as f32, v1[2] as f32]  );
            let v2 = stl_io::Vertex::new( [v2[0] as f32, v2[1] as f32, v2[2] as f32]  );
            let v3 = stl_io::Vertex::new( [v3[0] as f32, v3[1] as f32, v3[2] as f32]  );

            let triangle = stl_io::Triangle{ normal, vertices: [v1, v2, v3] };
            mesh.push(triangle);
        });

        let mut f = path.get_file_for_writing();
        stl_io::write_stl(&mut f, mesh.iter()).expect("could not write stl");
    }
    pub fn save_to_obj(&self, path: &PathBuf) {
        path.verify_extension(&vec!["obj", "OBJ"]).expect("error");

        let mut f = path.get_file_for_writing();

        for point in &self.points {
            // this default setting must be loaded into blender with z up and y forward
            writeln!(f, "v {} {} {}", point[0], point[1], point[2]).expect("error");
            // writeln!(f, "v {} {} {}", point[0], point[2], -point[1]).expect("error");
        }

        for index in &self.indices {
            writeln!(f, "f {} {} {}", index[0] + 1, index[1] + 1, index[2] + 1).expect("error");
        }
    }
    pub fn save_to_glb(&self, path: &PathBuf) {
        path.verify_extension(&vec!["glb", "GLB", "gltf", "GLTF"]).expect("error");

        let mut vertices: Vec<f32> = Vec::new();
        for point in &self.points {
            // vertices.push(point[0] as f32);  // X remains the same
            // vertices.push(point[2] as f32);  // Z becomes Y
            // vertices.push(-(point[1] as f32));  // -Y becomes Zx

            vertices.push(point[0] as f32);
            vertices.push(point[1] as f32);
            vertices.push(point[2] as f32);
        }

        let mut indices: Vec<u32> = Vec::new();
        for index in &self.indices {
            indices.extend(index.iter().map(|&i| i as u32));
        }

        let mut buffer_data: Vec<u8> = Vec::new();
        buffer_data.extend(vertices.iter().flat_map(|v| v.to_le_bytes()).collect::<Vec<u8>>());
        buffer_data.extend(indices.iter().flat_map(|i| i.to_le_bytes()).collect::<Vec<u8>>());

        let buffer_base64 = base64::engine::general_purpose::STANDARD.encode(&buffer_data);

        let buffer = json::Buffer {
            byte_length: USize64::from(buffer_data.len() as u64),
            name: None,
            uri: Some(format!("data:application/octet-stream;base64,{}", buffer_base64)),
            extensions: None,
            extras: Default::default(),
        };

        let (min, max) = calculate_min_max(&vertices);

        let position_accessor = json::Accessor {
            buffer_view: Some(json::Index::new(0)),
            byte_offset: Some(USize64::from(0u64)),
            count: USize64::from(self.points.len()),
            component_type: Checked::Valid(GenericComponentType(F32)),
            extensions: None,
            extras: Default::default(),
            type_: Checked::Valid(json::accessor::Type::Vec3),
            min: Some(json!(min)),
            max: Some(json!(max)),
            name: None,
            normalized: false,
            sparse: None,
        };

        let index_accessor = json::Accessor {
            buffer_view: Some(json::Index::new(1)),
            byte_offset: Some(USize64::from(0u64)),
            count: USize64::from(self.indices.len() * 3),
            component_type: Checked::Valid(GenericComponentType(U32)),
            extensions: None,
            extras: Default::default(),
            type_: Checked::Valid(json::accessor::Type::Scalar),
            min: Some(json!(vec![0.0])),
            max: Some(json!(vec![indices.iter().cloned().fold(0, u32::max) as f64])),
            name: None,
            normalized: false,
            sparse: None,
        };

        let buffer_view_vertices = json::buffer::View {
            buffer: json::Index::new(0),
            byte_length: USize64::from(vertices.len() * std::mem::size_of::<f32>()),
            byte_offset: Some(USize64::from(0u64)),
            byte_stride: None,
            name: None,
            target: Some(Checked::Valid(json::buffer::Target::ArrayBuffer)),
            extensions: None,
            extras: Default::default(),
        };

        let buffer_view_indices = json::buffer::View {
            buffer: json::Index::new(0),
            byte_length: USize64::from(indices.len() * std::mem::size_of::<u32>()),
            byte_offset: Some(USize64::from(vertices.len() * std::mem::size_of::<f32>())),
            byte_stride: None,
            name: None,
            target: Some(Checked::Valid(json::buffer::Target::ElementArrayBuffer)),
            extensions: None,
            extras: Default::default(),
        };

        let primitive = json::mesh::Primitive {
            attributes: {
                let mut map = BTreeMap::new();
                map.insert(
                    Checked::Valid(json::mesh::Semantic::Positions),
                    json::Index::new(0),
                );
                map
            },
            extensions: None,
            extras: Default::default(),
            indices: Some(json::Index::new(1)),
            material: None,
            mode: Checked::Valid(json::mesh::Mode::Triangles),
            targets: None,
        };

        let mesh = json::Mesh {
            extensions: None,
            extras: Default::default(),
            name: None,
            primitives: vec![primitive],
            weights: None,
        };

        let node = json::Node {
            mesh: Some(json::Index::new(0)),
            ..Default::default()
        };

        let scene = json::Scene {
            extensions: None,
            extras: Default::default(),
            name: None,
            nodes: vec![json::Index::new(0)],
        };

        let document = json::Root {
            accessors: vec![position_accessor, index_accessor],
            buffers: vec![buffer],
            buffer_views: vec![buffer_view_vertices, buffer_view_indices],
            meshes: vec![mesh],
            nodes: vec![node],
            scenes: vec![scene],
            scene: Some(json::Index::new(0)),
            ..Default::default()
        };

        let json_string = json::serialize::to_string(&document).expect("Failed to serialize glTF document");

        let mut file = path.get_file_for_writing();
        write!(file, "{}", json_string).expect("Failed to write glTF file");
    }
    pub fn to_triangles(&self) -> Vec<[[f64; 3]; 3]> {
        let mut triangles = vec![];

        self.indices.iter().for_each(|idxs| {
            let triangle = [ self.points[idxs[0]], self.points[idxs[1]], self.points[idxs[2]] ];
            triangles.push(triangle);
        });

        triangles
    }
    pub fn to_convex_hull(&self) -> TriMesh {
        let points: Vec<Point<f64>> = self.points.iter().map(|x| Point::from_slice(x)).collect();

        let (ch_points, ch_indices) = convex_hull(points.as_slice());

        let points: Vec<[f64; 3]> = ch_points.iter().map(|x| [x[0], x[1], x[2]] ).collect();
        let indices: Vec<[usize; 3]> = ch_indices.iter().map(|x| [ x[0] as usize, x[1] as usize, x[2] as usize]).collect();

        TriMesh {
            points,
            indices
        }
    }
    pub fn to_convex_decomposition(&self, max_convex_hulls: u32) -> Vec<TriMesh> {
        let mut out = vec![];

        let points: Vec<Point<f64>> = self.points.iter().map(|x| Point::from_slice(x)).collect();
        let indices: Vec<[u32; 3]> = self.indices.iter().map(|x| [ x[0] as u32, x[1] as u32, x[2] as u32 ]).collect();

        let params = VHACDParameters {
            max_convex_hulls,
            resolution: 128,
            fill_mode: FillMode::FloodFill { detect_cavities: true },
            convex_hull_approximation: false,
            ..Default::default()
        };
        let res = VHACD::decompose(&params, &points, &indices, true);
        let convex_hulls = res.compute_convex_hulls(5);

        convex_hulls.iter().for_each(|(ch_points, ch_indices)| {
            let points: Vec<[f64; 3]> = ch_points.iter().map(|x| [x[0], x[1], x[2]] ).collect();
            let indices: Vec<[usize; 3]> = ch_indices.iter().map(|x| [ x[0] as usize, x[1] as usize, x[2] as usize]).collect();

            out.push(TriMesh {points, indices } );
        });

        out
    }
    #[inline(always)]
    pub fn points(&self) -> &Vec<[f64; 3]> {
        &self.points
    }
    #[inline(always)]
    pub fn indices(&self) -> &Vec<[usize; 3]> {
        &self.indices
    }
    #[inline(always)]
    pub fn indices_as_u32s(&self) -> Vec<[u32; 3]> {
        self.indices.iter().map(|x| [x[0] as u32, x[1] as u32, x[2] as u32] ).collect()
    }
}

pub trait SaveToSTL {
    fn save_to_stl(&self, path: &PathBuf);
}

impl<M: ToTriMesh> SaveToSTL for M {
    fn save_to_stl(&self, path: &PathBuf) {
        self.to_trimesh().save_to_stl(path);
    }
}

pub trait SaveToOBJ {
    fn save_to_obj(&self, path: &PathBuf);
}
impl<M: ToTriMesh> SaveToOBJ for M {
    fn save_to_obj(&self, path: &PathBuf) {
        self.to_trimesh().save_to_obj(path);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub fn calculate_min_max(points: &[f32]) -> (Vec<f64>, Vec<f64>) {
    let mut min = vec![f64::INFINITY; 3];
    let mut max = vec![f64::NEG_INFINITY; 3];

    for chunk in points.chunks(3) {
        for i in 0..3 {
            if (chunk[i] as f64) < min[i] {
                min[i] = chunk[i] as f64;
            }
            if (chunk[i] as f64) > max[i] {
                max[i] = chunk[i] as f64;
            }
        }
    }

    (min, max)
}