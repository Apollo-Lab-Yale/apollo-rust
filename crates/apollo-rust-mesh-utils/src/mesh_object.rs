use std::collections::BTreeMap;
use std::path::PathBuf;
use base64::Engine;
use gltf::{json};
use gltf_json::accessor::ComponentType::{F32, U32};
use gltf_json::accessor::GenericComponentType;
use gltf_json::validation::{Checked, USize64};
use serde_json::json;
use apollo_rust_file::ApolloPathBufTrait;
use crate::trimesh::{calculate_min_max, TriMesh};
use std::io::Write;
use gltf_json::scene::UnitQuaternion;
use nalgebra::Isometry3;

#[derive(Clone, Debug)]
pub struct MeshObject {
    pub local_space_trimeshes: Vec<TriMesh>,
    pub offset_from_parent: Isometry3<f64>,
    pub scale: [f64; 3],
    pub extra_info: ExtraInfo,
}
impl MeshObject {
    pub fn save_to_glb(&self, path: &PathBuf) {
        path.verify_extension(&vec!["glb", "GLB", "gltf", "GLTF"]).expect("error");

        let mut buffer_view_count = 0;

        let mut buffers = vec![];
        let mut accessors = vec![];
        let mut buffer_views = vec![];

        for (i, local_space_trimesh) in self.local_space_trimeshes.iter().enumerate() {
            let mut vertices: Vec<f32> = Vec::new();
            for point in &local_space_trimesh.points {
                vertices.push(point[0] as f32);
                vertices.push(point[1] as f32);
                vertices.push(point[2] as f32);
            }

            let mut indices: Vec<u32> = Vec::new();
            for index in &local_space_trimesh.indices {
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
            buffers.push(buffer);

            let (min, max) = calculate_min_max(&vertices);

            let position_accessor = json::Accessor {
                buffer_view: Some(json::Index::new(buffer_view_count)),
                byte_offset: Some(USize64::from(0u64)),
                count: USize64::from(local_space_trimesh.points.len()),
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
            accessors.push(position_accessor);
            buffer_view_count += 1;

            let index_accessor = json::Accessor {
                buffer_view: Some(json::Index::new(buffer_view_count)),
                byte_offset: Some(USize64::from(0u64)),
                count: USize64::from(local_space_trimesh.indices.len() * 3),
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
            accessors.push(index_accessor);
            buffer_view_count += 1;

            let buffer_view_vertices = json::buffer::View {
                buffer: json::Index::new(i as u32),
                byte_length: USize64::from(vertices.len() * size_of::<f32>()),
                byte_offset: Some(USize64::from(0u64)),
                byte_stride: None,
                name: None,
                target: Some(Checked::Valid(json::buffer::Target::ArrayBuffer)),
                extensions: None,
                extras: Default::default(),
            };
            buffer_views.push(buffer_view_vertices);

            let buffer_view_indices = json::buffer::View {
                buffer: json::Index::new(i as u32),
                byte_length: USize64::from(indices.len() * size_of::<u32>()),
                byte_offset: Some(USize64::from(vertices.len() * size_of::<f32>())),
                byte_stride: None,
                name: None,
                target: Some(Checked::Valid(json::buffer::Target::ElementArrayBuffer)),
                extensions: None,
                extras: Default::default(),
            };
            buffer_views.push(buffer_view_indices);
        }

        let materials = match &self.extra_info {
            ExtraInfo::None => { vec![] }
            ExtraInfo::GLB { materials } => { materials.clone() }
        };

        let mut primitives = vec![];

        for (material_index, _material) in materials.iter().enumerate() {
            let primitive = json::mesh::Primitive {
                attributes: {
                    let mut map = BTreeMap::new();
                    map.insert(
                        Checked::Valid(json::mesh::Semantic::Positions),
                        json::Index::new((material_index * 2) as u32),
                    );
                    map
                },
                extensions: None,
                extras: Default::default(),
                indices: Some(json::Index::new((material_index * 2 + 1) as u32)),
                material: Some(json::Index::new(material_index as u32)),
                mode: Checked::Valid(json::mesh::Mode::Triangles),
                targets: None,
            };
            primitives.push(primitive);
        }

        let mesh = json::Mesh {
            extensions: None,
            extras: Default::default(),
            name: None,
            primitives,
            weights: None,
        };

        let node = json::Node {
            mesh: Some(json::Index::new(0)),
            translation: Some([self.offset_from_parent.translation.x as f32, self.offset_from_parent.translation.y as f32, self.offset_from_parent.translation.z as f32]),
            rotation: Some(UnitQuaternion([self.offset_from_parent.rotation.i as f32, self.offset_from_parent.rotation.j as f32, self.offset_from_parent.rotation.k as f32, self.offset_from_parent.rotation.w as f32])),
            scale: Some([self.scale[0] as f32, self.scale[1] as f32, self.scale[2] as f32]),
            ..Default::default()
        };

        let scene = json::Scene {
            extensions: None,
            extras: Default::default(),
            name: None,
            nodes: vec![json::Index::new(0)],
        };

        let document = json::Root {
            accessors,
            buffers,
            buffer_views,
            meshes: vec![mesh],
            nodes: vec![node],
            scenes: vec![scene],
            scene: Some(json::Index::new(0)),
            materials,
            ..Default::default()
        };

        let json_string = json::serialize::to_string(&document).expect("Failed to serialize GLB document");

        let mut file = path.get_file_for_writing();
        write!(file, "{}", json_string).expect("Failed to write GLB file");


    }
}

/*
pub fn save_to_glb(&self, path: &PathBuf) {
    path.verify_extension(&vec!["glb", "GLB", "gltf", "GLTF"]).expect("error");

    let mut vertices: Vec<f32> = Vec::new();
    for point in &self.local_space_trimesh.points {
        vertices.push(point[0] as f32);
        vertices.push(point[1] as f32);
        vertices.push(point[2] as f32);
    }

    let mut indices: Vec<u32> = Vec::new();
    for index in &self.local_space_trimesh.indices {
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
        count: USize64::from(self.local_space_trimesh.points.len()),
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
        count: USize64::from(self.local_space_trimesh.indices.len() * 3),
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

    let materials = match &self.extra_info {
        ExtraInfo::None => { vec![] }
        ExtraInfo::GLB { materials } => { materials.clone() }
    };

    let mut primitives = vec![];

    // Assuming the mesh is divided into sub-meshes by material
    for (material_index, _material) in materials.iter().enumerate() {
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
            material: Some(json::Index::new(material_index as u32)),
            mode: Checked::Valid(json::mesh::Mode::Triangles),
            targets: None,
        };
        primitives.push(primitive);
    }

    let mesh = json::Mesh {
        extensions: None,
        extras: Default::default(),
        name: None,
        primitives,
        weights: None,
    };

    let node = json::Node {
        mesh: Some(json::Index::new(0)),
        translation: Some([self.offset_from_parent.translation.x as f32, self.offset_from_parent.translation.y as f32, self.offset_from_parent.translation.z as f32]),
        rotation: Some(UnitQuaternion([self.offset_from_parent.rotation.i as f32, self.offset_from_parent.rotation.j as f32, self.offset_from_parent.rotation.k as f32, self.offset_from_parent.rotation.w as f32])),
        scale: Some([self.scale[0] as f32, self.scale[1] as f32, self.scale[2] as f32]),
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
        materials,
        ..Default::default()
    };

    let json_string = json::serialize::to_string(&document).expect("Failed to serialize GLB document");

    let mut file = path.get_file_for_writing();
    write!(file, "{}", json_string).expect("Failed to write GLB file");
}
*/


#[derive(Clone, Debug)]
pub enum ExtraInfo {
    None,
    GLB {
        materials: Vec<gltf_json::Material>
    },
}