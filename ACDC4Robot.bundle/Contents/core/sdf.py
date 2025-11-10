# -*- coding: utf-8 -*-
"""
SDF helpers for ACDC4Robot.

Genera un archivo .sdf a partir de:
- Link (core.link.Link)
- Joint (core.joint.Joint)

Notas:
- NO depende de '..commands.ACDC4Robot'.
- Usa sólo imports internos del paquete 'core'.
- Expone get_link_element / get_joint_element para compatibilidad con write.py.
"""

import os
from xml.etree.ElementTree import Element, SubElement, ElementTree

import adsk
import adsk.core
import adsk.fusion

from .link import Link
from .joint import Joint


# ================== Helpers internos ================== #

def _link_to_sdf_element(link: Link) -> Element:
    """
    Convierte un Link a <link> SDF básico.
    Usa CoM e inercia tal como están definidos en Link.
    """
    name = link.get_name()
    pose = link.get_pose_sdf()  # [x,y,z,r,p,y]

    link_ele = Element("link", {"name": name})

    # Pose del link en el modelo
    pose_ele = SubElement(link_ele, "pose")
    pose_ele.text = f"{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}"

    # Inertial
    inertial = SubElement(link_ele, "inertial")

    mass_ele = SubElement(inertial, "mass")
    mass_ele.text = f"{link.get_mass()}"

    ixx, iyy, izz, ixy, iyz, ixz = link.get_inertia_sdf()
    inertia_ele = SubElement(inertial, "inertia")
    inertia_ele.attrib = {
        "ixx": f"{ixx}",
        "iyy": f"{iyy}",
        "izz": f"{izz}",
        "ixy": f"{ixy}",
        "iyz": f"{iyz}",
        "ixz": f"{ixz}",
    }

    # Visual simple (usa mismo mesh que URDF; escala en m)
    visual = SubElement(link_ele, "visual", {"name": f"{name}_visual"})
    v_geom = SubElement(visual, "geometry")
    v_mesh = SubElement(v_geom, "mesh")
    v_mesh.attrib = {"uri": f"meshes/{name}.stl"}

    # Collision simple
    collision = SubElement(link_ele, "collision", {"name": f"{name}_collision"})
    c_geom = SubElement(collision, "geometry")
    c_mesh = SubElement(c_geom, "mesh")
    c_mesh.attrib = {"uri": f"meshes/{name}.stl"}

    return link_ele


def _joint_to_sdf_element(joint: Joint) -> Element:
    """
    Convierte un Joint a <joint> SDF básico.
    """
    name = joint.get_name()
    parent = joint.get_parent()
    child = joint.get_child()
    jtype_urdf = joint.get_urdf_joint_type()

    # Map URDF -> SDF
    if jtype_urdf in ("fixed",):
        jtype_sdf = "fixed"
    elif jtype_urdf in ("revolute", "continuous"):
        jtype_sdf = "revolute"
    elif jtype_urdf in ("prismatic",):
        jtype_sdf = "prismatic"
    else:
        jtype_sdf = "fixed"

    joint_ele = Element("joint", {"name": name, "type": jtype_sdf})

    # parent / child
    parent_ele = SubElement(joint_ele, "parent")
    parent_ele.text = parent

    child_ele = SubElement(joint_ele, "child")
    child_ele.text = child

    # pose del joint en el modelo
    o = joint.get_urdf_origin()
    pose_ele = SubElement(joint_ele, "pose")
    pose_ele.text = f"{o[0]} {o[1]} {o[2]} {o[3]} {o[4]} {o[5]}"

    # axis + límites si aplica
    axis_vec = joint.get_axis_for_urdf()
    if axis_vec is not None and jtype_sdf in ("revolute", "prismatic"):
        axis_ele = SubElement(joint_ele, "axis")
        xyz_ele = SubElement(axis_ele, "xyz")
        xyz_ele.text = f"{axis_vec[0]} {axis_vec[1]} {axis_vec[2]}"

        limits = joint.get_limits()
        if limits is not None:
            lower, upper = limits
            limit_ele = SubElement(axis_ele, "limit")
            limit_ele.attrib = {
                "lower": f"{lower}",
                "upper": f"{upper}",
                "effort": "1000000",
                "velocity": "1000000",
            }

    return joint_ele


# ============ API pública (usada por write.py) ============ #

def get_link_element(link: Link) -> Element:
    """Compatibilidad con write.write_sdf."""
    return _link_to_sdf_element(link)


def get_joint_element(joint: Joint) -> Element:
    """Compatibilidad con write.write_sdf."""
    return _joint_to_sdf_element(joint)


def write_sdf(link_list, joint_list, output_dir, model_name):
    """
    Genera un archivo .sdf:

    <sdf version="1.6">
      <model name="...">
        ...links...
        ...joints...
      </model>
    </sdf>
    """
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    sdf_ele = Element("sdf", {"version": "1.6"})
    model_ele = SubElement(sdf_ele, "model", {"name": model_name})

    for link in link_list:
        model_ele.append(_link_to_sdf_element(link))

    for joint in joint_list:
        model_ele.append(_joint_to_sdf_element(joint))

    sdf_path = os.path.join(output_dir, f"{model_name}.sdf")
    tree = ElementTree(sdf_ele)
    tree.write(sdf_path, encoding="utf-8", xml_declaration=True)

    try:
        print(f"[ACDC4Robot] SDF generado: {sdf_path}")
    except:
        pass
