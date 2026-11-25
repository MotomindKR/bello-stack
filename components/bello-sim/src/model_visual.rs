use anyhow::{Context, Result, ensure};
use mujoco_rs::prelude::MjSpec;
use nalgebra::{Matrix4, Point3, Vector3};
use std::{
    collections::BTreeMap,
    io::Write,
    path::{Path, PathBuf},
};
use xmltree::{Element, XMLNode};

type ColoredMesh = (PathBuf, [f32; 4]);
type Triangles = Vec<[[f32; 3]; 3]>;

pub(crate) fn imported_spec(urdf: &Path, visuals: bool) -> Result<(MjSpec, tempfile::TempDir)> {
    let assets = tempfile::tempdir()?;
    let mut document = Element::parse(std::fs::File::open(urdf)?)?;
    // URDF import discards visuals by default. This must be set before parsing,
    // rather than on the resulting spec, where those geoms have already gone.
    if visuals {
        if document.get_child("mujoco").is_none() {
            document
                .children
                .push(XMLNode::Element(Element::new("mujoco")));
        }
        let extension = document.get_mut_child("mujoco").unwrap();
        if extension.get_child("compiler").is_none() {
            extension
                .children
                .push(XMLNode::Element(Element::new("compiler")));
        }
        extension
            .get_mut_child("compiler")
            .unwrap()
            .attributes
            .insert("discardvisual".into(), "false".into());
    }
    let root = urdf.parent().context("URDF has no parent")?;
    let mut cache = BTreeMap::<PathBuf, Vec<ColoredMesh>>::new();
    for link in document
        .children
        .iter_mut()
        .filter_map(XMLNode::as_mut_element)
        .filter(|e| e.name == "link")
    {
        let link_name = link
            .attributes
            .get("name")
            .context("link name missing")?
            .clone();
        let mut children = Vec::with_capacity(link.children.len());
        for child in link.children.drain(..) {
            let XMLNode::Element(mut element) = child else {
                children.push(child);
                continue;
            };
            let is_visual = element.name == "visual";
            let mesh = element
                .get_mut_child("geometry")
                .and_then(|g| g.get_mut_child("mesh"));
            let Some(mesh) = mesh else {
                children.push(XMLNode::Element(element));
                continue;
            };
            let filename = mesh
                .attributes
                .get("filename")
                .context("visual mesh filename missing")?;
            let path = root.join(filename).canonicalize()?;
            if path
                .extension()
                .is_some_and(|s| s.eq_ignore_ascii_case("glb"))
            {
                if visuals && is_visual {
                    if !cache.contains_key(&path) {
                        let prefix = format!("mesh_{}", cache.len());
                        cache.insert(
                            path.clone(),
                            split_materials(&path, assets.path(), &prefix)?,
                        );
                    }
                    for (index, (path, rgba)) in cache[&path].iter().enumerate() {
                        let mut part = element.clone();
                        let name = element
                            .attributes
                            .get("name")
                            .map(String::as_str)
                            .unwrap_or("visual");
                        part.attributes
                            .insert("name".into(), format!("{name}_material_{index}"));
                        part.get_mut_child("geometry")
                            .unwrap()
                            .get_mut_child("mesh")
                            .unwrap()
                            .attributes
                            .insert("filename".into(), path.to_string_lossy().into());
                        if part.get_child("material").is_none() {
                            let mut material = Element::new("material");
                            material
                                .attributes
                                .insert("name".into(), format!("{link_name}_{name}_color_{index}"));
                            let mut color = Element::new("color");
                            color.attributes.insert(
                                "rgba".into(),
                                rgba.iter()
                                    .enumerate()
                                    .map(|(i, &c)| if i == 3 { c } else { linear_to_srgb(c) })
                                    .map(|v| v.to_string())
                                    .collect::<Vec<_>>()
                                    .join(" "),
                            );
                            material.children.push(XMLNode::Element(color));
                            part.children.push(XMLNode::Element(material));
                        }
                        children.push(XMLNode::Element(part));
                    }
                    continue;
                }
                mesh.attributes.insert(
                    "filename".into(),
                    path.with_extension("stl")
                        .canonicalize()
                        .with_context(|| {
                            format!("missing MuJoCo STL companion for {}", path.display())
                        })?
                        .to_string_lossy()
                        .into(),
                );
            } else {
                mesh.attributes
                    .insert("filename".into(), path.to_string_lossy().into());
            }
            children.push(XMLNode::Element(element));
        }
        link.children = children;
    }
    let path = assets.path().join("robot.urdf");
    document.write(std::fs::File::create(&path)?)?;
    Ok((MjSpec::from_xml(&path)?, assets))
}

fn linear_to_srgb(c: f32) -> f32 {
    if c <= 0.0031308 {
        12.92 * c
    } else {
        1.055 * c.powf(1. / 2.4) - 0.055
    }
}

fn split_materials(path: &Path, output: &Path, prefix: &str) -> Result<Vec<ColoredMesh>> {
    let gltf = gltf::Gltf::open(path).with_context(|| format!("load visual {}", path.display()))?;
    let blob = gltf
        .blob
        .as_deref()
        .context("visual GLB has no embedded buffer")?;
    ensure!(
        gltf.buffers()
            .all(|b| b.index() == 0 && matches!(b.source(), gltf::buffer::Source::Bin)),
        "visual GLB must be self-contained"
    );
    let scene = gltf
        .default_scene()
        .context("visual GLB has no default scene")?;
    let mut groups = BTreeMap::<Option<usize>, ([f32; 4], Triangles)>::new();
    for node in scene.nodes() {
        collect(node, &Matrix4::identity(), blob, &mut groups)?;
    }
    ensure!(!groups.is_empty(), "visual GLB has no triangles");
    let mut result = Vec::with_capacity(groups.len());
    for (index, (_, (rgba, triangles))) in groups.into_iter().enumerate() {
        let path = output.join(format!("{prefix}_{index}.stl"));
        let mut file = std::io::BufWriter::new(std::fs::File::create(&path)?);
        file.write_all(&[0; 80])?;
        file.write_all(&u32::try_from(triangles.len())?.to_le_bytes())?;
        for triangle in triangles {
            let [a, b, c] = triangle.map(Vector3::from);
            let normal = (b - a)
                .cross(&(c - a))
                .try_normalize(1e-12)
                .unwrap_or_else(Vector3::zeros);
            for value in normal.iter().chain(triangle.iter().flatten()) {
                file.write_all(&value.to_le_bytes())?;
            }
            file.write_all(&0u16.to_le_bytes())?;
        }
        file.flush()?;
        result.push((path, rgba));
    }
    Ok(result)
}

fn collect(
    node: gltf::Node<'_>,
    parent: &Matrix4<f32>,
    blob: &[u8],
    groups: &mut BTreeMap<Option<usize>, ([f32; 4], Triangles)>,
) -> Result<()> {
    let transform = parent * Matrix4::from_column_slice(node.transform().matrix().as_flattened());
    if let Some(mesh) = node.mesh() {
        for primitive in mesh.primitives() {
            ensure!(
                primitive.mode() == gltf::mesh::Mode::Triangles,
                "visual GLB requires triangle primitives"
            );
            let material = primitive.material();
            let pbr = material.pbr_metallic_roughness();
            ensure!(
                pbr.base_color_texture().is_none(),
                "textured GLB requires an explicit texture importer"
            );
            let reader = primitive.reader(|_| Some(blob));
            let vertices = reader
                .read_positions()
                .context("visual GLB has no positions")?
                .map(|p| {
                    let p = transform.transform_point(&Point3::from(p));
                    [p.x, p.y, p.z]
                })
                .collect::<Vec<_>>();
            ensure!(
                vertices.iter().flatten().all(|v| v.is_finite()),
                "nonfinite visual vertices"
            );
            let indices = reader
                .read_indices()
                .context("visual GLB requires indexed triangles")?
                .into_u32()
                .collect::<Vec<_>>();
            ensure!(
                indices.len().is_multiple_of(3),
                "incomplete visual triangle"
            );
            let group = groups
                .entry(material.index())
                .or_insert_with(|| (pbr.base_color_factor(), Vec::new()));
            for indices in indices.chunks_exact(3) {
                let mut triangle = [[0.; 3]; 3];
                for (i, &vertex) in indices.iter().enumerate() {
                    triangle[i] = *vertices
                        .get(vertex as usize)
                        .context("visual vertex index out of range")?;
                }
                if transform.determinant() < 0. {
                    triangle.swap(1, 2);
                }
                group.1.push(triangle);
            }
        }
    }
    for child in node.children() {
        collect(child, &transform, blob, groups)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model;
    use mujoco_rs::prelude::{MjtObj, MjtTexture};

    #[test]
    fn colored_mini_visuals_preserve_native_joints_and_inertias() {
        let config = model::ModelConfig::load(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("config/bello_mini_torso.yaml"),
        )
        .unwrap();
        let physics = model::build(&config, None).unwrap().simulation;
        let visuals = model::build_visual(&config).unwrap().simulation;
        let a = physics.data.model();
        let b = visuals.data.model();
        assert_eq!(a.nq(), b.nq());
        assert_eq!(a.nv(), b.nv());
        assert_eq!(a.nu(), b.nu());
        assert_eq!(a.jnt_qposadr(), b.jnt_qposadr());
        for i in 0..a.njnt() as usize {
            assert_eq!(
                a.id_to_name(MjtObj::mjOBJ_JOINT, i),
                b.id_to_name(MjtObj::mjOBJ_JOINT, i)
            );
        }
        for (a, b) in a.body_mass().iter().zip(b.body_mass()) {
            assert!((a - b).abs() < 1e-10);
        }
        assert_eq!(b.tex_type(), &[MjtTexture::mjTEXTURE_SKYBOX]);
        let body = b.name_to_id(MjtObj::mjOBJ_BODY, "torso_link").unwrap() as i32;
        let colors = (0..b.ngeom() as usize)
            .filter(|&i| {
                b.geom_bodyid()[i] == body
                    && b.geom_contype()[i] == 0
                    && b.geom_conaffinity()[i] == 0
            })
            .map(|i| b.geom_rgba()[i])
            .collect::<Vec<_>>();
        assert!(
            colors
                .iter()
                .any(|c| c[0] > 0.8 && c[1] < 0.4 && c[2] < 0.2),
            "authored orange body material missing: {colors:?}"
        );
        assert!(
            colors
                .iter()
                .any(|c| c[0] > 0.8 && c[1] > 0.8 && c[2] > 0.8),
            "authored white body material missing"
        );
        assert!(
            colors
                .iter()
                .any(|c| c[0] < 0.15 && c[1] < 0.15 && c[2] < 0.15),
            "authored black body material missing"
        );
    }
}
