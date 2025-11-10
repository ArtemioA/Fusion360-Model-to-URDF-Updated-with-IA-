# -*- coding: utf-8 -*-
"""
Generación de elementos URDF a partir de Link y Joint.
"""

from xml.etree.ElementTree import Element, SubElement

from .link import Link
from .joint import Joint
from . import utils


def get_link_visual_geo(link: Link) -> str:
    visual_body = link.get_visual_body()
    col_body = link.get_collision_body()
    name = link.get_name()

    if (visual_body is None) and (col_body is None):
        return f"meshes/{name}.stl"
    elif (visual_body is not None) and (col_body is not None):
        return f"meshes/{name}_visual.stl"
    elif (visual_body is None) and (col_body is not None):
        utils.error_box(
            f"Link {name}: falta cuerpo 'visual' (tienes 'collision')."
        )
        utils.terminate_box()
    else:
        utils.error_box(
            f"Link {name}: falta cuerpo 'collision' (tienes 'visual')."
        )
        utils.terminate_box()


def get_link_collision_geo(link: Link) -> str:
    visual_body = link.get_visual_body()
    col_body = link.get_collision_body()
    name = link.get_name()

    if (visual_body is None) and (col_body is None):
        return f"meshes/{name}.stl"
    elif (visual_body is not None) and (col_body is not None):
        return f"meshes/{name}_collision.stl"
    elif (visual_body is None) and (col_body is not None):
        utils.error_box(
            f"Link {name}: falta 'visual' para usar con 'collision'."
        )
        utils.terminate_box()
    else:
        utils.error_box(
            f"Link {name}: falta 'collision' para usar con 'visual'."
        )
        utils.terminate_box()


def get_link_element(link: Link) -> Element:
    link_ele = Element("link")
    name = link.get_name()
    link_ele.attrib = {"name": name}

    # inertial
    inertial = SubElement(link_ele, "inertial")
    CoM = link.get_CoM_urdf()
    origin_i = SubElement(inertial, "origin")
    origin_i.attrib = {
        "xyz": f"{CoM[0]} {CoM[1]} {CoM[2]}",
        "rpy": f"{CoM[3]} {CoM[4]} {CoM[5]}",
    }

    mass_ele = SubElement(inertial, "mass")
    mass_ele.attrib = {"value": f"{link.get_mass()}"}

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

    # visual
    visual = SubElement(link_ele, "visual")
    visual.attrib = {"name": f"{name}_visual"}
    origin_v = SubElement(visual, "origin")
    mo = link.get_mesh_origin()
    origin_v.attrib = {
        "xyz": f"{mo[0]} {mo[1]} {mo[2]}",
        "rpy": f"{mo[3]} {mo[4]} {mo[5]}",
    }
    geometry_v = SubElement(visual, "geometry")
    mesh_v = SubElement(geometry_v, "mesh")
    mesh_v.attrib = {
        "filename": get_link_visual_geo(link),
        "scale": "0.001 0.001 0.001",
    }

    # collision
    collision = SubElement(link_ele, "collision")
    collision.attrib = {"name": f"{name}_collision"}
    origin_c = SubElement(collision, "origin")
    origin_c.attrib = {
        "xyz": f"{mo[0]} {mo[1]} {mo[2]}",
        "rpy": f"{mo[3]} {mo[4]} {mo[5]}",
    }
    geometry_c = SubElement(collision, "geometry")
    mesh_c = SubElement(geometry_c, "mesh")
    mesh_c.attrib = {
        "filename": get_link_collision_geo(link),
        "scale": "0.001 0.001 0.001",
    }

    return link_ele


def get_joint_element(joint: Joint) -> Element:
    joint_ele = Element("joint")
    joint_ele.attrib = {
        "name": joint.get_name(),
        "type": joint.get_urdf_joint_type(),
    }

    o = joint.get_urdf_origin()
    origin = SubElement(joint_ele, "origin")
    origin.attrib = {
        "xyz": f"{o[0]} {o[1]} {o[2]}",
        "rpy": f"{o[3]} {o[4]} {o[5]}",
    }

    parent = SubElement(joint_ele, "parent")
    parent.attrib = {"link": joint.get_parent()}

    child = SubElement(joint_ele, "child")
    child.attrib = {"link": joint.get_child()}

    axis_vec = joint.get_axis_for_urdf()
    if axis_vec is not None:
        axis_ele = SubElement(joint_ele, "axis")
        axis_ele.attrib = {
            "xyz": f"{axis_vec[0]} {axis_vec[1]} {axis_vec[2]}"
        }

    limits = joint.get_limits()
    if limits is not None and joint.get_urdf_joint_type() in ("revolute", "prismatic", "continuous"):
        lower, upper = limits
        limit_ele = SubElement(joint_ele, "limit")
        limit_ele.attrib = {
            "lower": f"{lower}",
            "upper": f"{upper}",
            "effort": "1000000",
            "velocity": "1000000",
        }

    return joint_ele
