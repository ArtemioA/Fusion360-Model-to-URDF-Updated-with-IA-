# -*- coding: utf-8 -*-
"""
MJCF helpers for ACDC4Robot.

Genera una estructura MJCF (MuJoCo) a partir de:
- Link (core.link.Link)
- Joint (core.joint.Joint)

Notas:
- NO depende de '..commands.ACDC4Robot'.
- Usa sólo imports internos.
- No usa métodos inexistentes en Link/Joint.
"""

import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, SubElement

import adsk
import adsk.core
import adsk.fusion

from .link import Link
from .joint import Joint
from . import math_operation as math_op


# ================== Helpers ================== #

def get_mjcf_mesh(link: Link) -> Element:
    """
    Crea un elemento <mesh> para el asset MJCF.
    """
    mesh_ele = ET.Element("mesh")
    name = link.get_name()
    file_name = f"meshes/{name}.stl"
    mesh_ele.attrib = {
        "name": name,
        "file": file_name,
        "scale": "0.001 0.001 0.001",
    }
    return mesh_ele


def _compute_body_pose(parent_link: Link | None, link: Link):
    """
    Devuelve (pos, euler) para el body MJCF:
    - Si parent_link es None: pose global del link.
    - Si tiene padre: pose relativa padre->hijo.
    """
    if parent_link is None:
        pose = math_op.matrix3d_2_euler_xyz(link.pose)
    else:
        parent_frame = parent_link.pose
        child_frame = link.pose
        parent_T_child = math_op.coordinate_transform(parent_frame, child_frame)
        pose = math_op.matrix3d_2_euler_xyz(parent_T_child)

    pos_attr = f"{pose[0]} {pose[1]} {pose[2]}"
    euler_attr = f"{pose[3]} {pose[4]} {pose[5]}"
    return pos_attr, euler_attr


def get_mjcf_inertial(link: Link) -> Element:
    """
    Inercial MJCF usando datos disponibles en Link.
    """
    inertial_ele = Element("inertial")

    # CoM en frame del link -> usamos get_CoM_sdf (CoM relativo al link)
    cx, cy, cz, _, _, _ = link.get_CoM_sdf()
    mass = link.get_mass()
    ixx, iyy, izz, ixy, iyz, ixz = link.get_inertia_sdf()

    inertial_ele.attrib = {
        "mass": f"{mass}",
        "pos": f"{cx} {cy} {cz}",
        # fullinertia: ixx, iyy, izz, ixy, ixz, iyz
        "fullinertia": f"{ixx} {iyy} {izz} {ixy} {ixz} {iyz}",
    }

    return inertial_ele


def get_mjcf_geom(link: Link) -> Element:
    """
    Geom MJCF simple basado en mesh.
    """
    geom_ele = Element("geom")
    geom_name = link.get_name() + "_geom"

    # Hacemos coincidir geom con el frame del body (pos = 0)
    geom_ele.attrib = {
        "name": geom_name,
        "type": "mesh",
        "mesh": link.get_name(),
        "pos": "0 0 0",
        "euler": "0 0 0",
    }

    return geom_ele


def get_mjcf_joint(joint: Joint) -> Element | None:
    """
    Crea un <joint> MJCF entre un body y su parent.
    Si es fixed, devuelve None (MuJoCo lo interpreta como soldado).
    """
    jtype = joint.get_urdf_joint_type()

    if jtype in ("fixed", None):
        return None

    if jtype in ("revolute", "continuous"):
        mjcf_type = "hinge"
    elif jtype == "prismatic":
        mjcf_type = "slide"
    else:
        return None

    o = joint.get_urdf_origin()
    axis = joint.get_axis_for_urdf() or [1, 0, 0]

    joint_ele = Element("joint")
    joint_ele.attrib = {
        "name": joint.get_name(),
        "type": mjcf_type,
        "pos": f"{o[0]} {o[1]} {o[2]}",
        "axis": f"{axis[0]} {axis[1]} {axis[2]}",
    }

    limits = joint.get_limits()
    if limits is not None and mjcf_type in ("hinge", "slide"):
        lower, upper = limits
        # En MJCF se usan attrs separados; aquí ponemos sólo range si tiene límites.
        joint_ele.attrib["range"] = f"{lower} {upper}"

    return joint_ele


def get_mjcf_body(link: Link, parent_link: Link | None = None) -> Element:
    """
    Crea un <body> MJCF para un link concreto.
    Incluye:
    - joint padre (si existe y no es fixed)
    - geom (mesh)
    - inertial
    """
    body_ele = Element("body")
    body_name = link.get_name()

    pos_attr, euler_attr = _compute_body_pose(parent_link, link)
    body_ele.attrib = {
        "name": body_name,
        "pos": pos_attr,
        "euler": euler_attr,
    }

    # Joint padre (si existe)
    parent_joint = link.get_parent_joint()
    if parent_joint is not None:
        j_ele = get_mjcf_joint(Joint(parent_joint))
        if j_ele is not None:
            body_ele.append(j_ele)

    # Geom + Inertial
    body_ele.append(get_mjcf_geom(link))
    body_ele.append(get_mjcf_inertial(link))

    return body_ele


# ================== Árbol de cuerpos y modelo ================== #

def get_mjcf(root_comp: adsk.fusion.Component, robot_name: str, dir: str) -> Element:
    """
    Construye el árbol MJCF completo (sin escribir archivo).
    Usado por write.write_mjcf.
    """
    root = ET.Element("mujoco", {"model": robot_name})

    # Compiler: configuraciones básicas
    ET.SubElement(root, "compiler", {"angle": "radian"})

    # Assets
    asset_ele = ET.SubElement(root, "asset")

    # Worldbody
    worldbody_ele = ET.SubElement(root, "worldbody")

    # Luz
    light_ele = ET.SubElement(worldbody_ele, "light")
    light_ele.attrib = {
        "directional": "true",
        "pos": "-0.5 0.5 3",
        "dir": "0 0 -1",
    }

    # Piso
    floor = ET.SubElement(worldbody_ele, "geom")
    floor.attrib = {
        "pos": "0 0 0",
        "size": "1 1 1",
        "type": "plane",
        "rgba": "1 0.83 0.61 0.5",
    }

    # Construir relación padre-hijo desde joints de Fusion
    parent_child_dict = {}
    joints = root_comp.allJoints

    for j in joints:
        parent = j.occurrenceTwo
        child = j.occurrenceOne
        if parent is None or child is None:
            continue
        parent_child_dict.setdefault(parent.fullPathName, []).append(child)

    all_occs = list(root_comp.allOccurrences)

    # Añadir assets mesh para todos los links sólidos
    for occ in all_occs:
        link = Link(occ)
        asset_ele.append(get_mjcf_mesh(link))

    # Función recursiva para armar el árbol MJCF
    def add_body_element(parent_occ: adsk.fusion.Occurrence, parent_body_ele: Element):
        children = parent_child_dict.get(parent_occ.fullPathName, [])
        parent_link = Link(parent_occ)

        for child_occ in children:
            child_link = Link(child_occ)
            child_body_ele = get_mjcf_body(child_link, parent_link)
            parent_body_ele.append(child_body_ele)
            add_body_element(child_occ, child_body_ele)

    # Raíces: occurrences que no son child en ningún joint
    all_children = {
        c.fullPathName
        for childs in parent_child_dict.values()
        for c in childs
    }

    for occ in all_occs:
        if occ.fullPathName not in all_children:
            # Este es un root body
            root_link = Link(occ)
            root_body_ele = get_mjcf_body(root_link, None)
            worldbody_ele.append(root_body_ele)
            add_body_element(occ, root_body_ele)

    return root
