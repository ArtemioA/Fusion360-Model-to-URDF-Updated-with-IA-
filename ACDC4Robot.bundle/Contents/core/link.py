# -*- coding: utf-8 -*-
"""
Link helper: representa una occurrence como link URDF/SDF/MJCF.
Calcula nombres, masas, CoM, orígenes de malla, etc.
"""

import adsk.core
import adsk.fusion
import adsk

from . import utils
from .joint import Joint
from . import math_operation as math_op


class Link:
    """
    Wrapper sobre adsk.fusion.Occurrence.

    Atributos:
        link  : Occurrence
        name  : nombre URDF-safe
        pose  : Matrix3D (transform2 global)
        phyPro: PhysicalProperties
    """

    # Directorio opcional para meshes (lo setea robot.py si se quiere usar)
    dir = ""

    def __init__(self, occurrence: adsk.fusion.Occurrence) -> None:
        self.link = occurrence
        self.name = None
        # Pose global del link
        self.pose: adsk.core.Matrix3D = occurrence.transform2
        # Propiedades físicas con alta precisión
        self.phyPro = occurrence.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy
        )

    # ==================== Básicos ====================

    def get_link_occ(self) -> adsk.fusion.Occurrence:
        return self.link

    def get_parent_joint(self):
        """
        Devuelve el joint padre donde este link es occurrenceOne,
        o None si es raíz.
        """
        joint_list = self.link.joints
        for j in joint_list:
            if j.occurrenceOne == self.link:
                return j
        return None

    def get_name(self) -> str:
        """
        Nombre único/seguro para URDF basado en fullPathName.
        Cacheado en self.name.
        """
        if self.name:
            return self.name

        if self.link.component.name == "base_link":
            nm = "base_link"
        else:
            nm = utils.get_valid_filename(self.link.fullPathName)

        self.name = nm
        return nm

    # ==================== Masas / Inercia / CoM ====================

    def get_pose_sdf(self):
        """[x, y, z, roll, pitch, yaw] desde self.pose."""
        return math_op.matrix3d_2_pose(self.pose)

    def get_inertia_sdf(self):
        """
        Inercia en el frame del link (CoM).
        Usa phyPro y transforma al frame del link.
        """
        (_, w_ixx, w_iyy, w_izz, w_ixy, w_iyz, w_ixz) = self.phyPro.getXYZMomentsOfInertia()
        x = self.phyPro.centerOfMass.x * 0.01
        y = self.phyPro.centerOfMass.y * 0.01
        z = self.phyPro.centerOfMass.z * 0.01
        mass = self.phyPro.mass

        # Traslado al CoM
        com_ixx = w_ixx * 0.0001 - mass * (y ** 2 + z ** 2)
        com_iyy = w_iyy * 0.0001 - mass * (x ** 2 + z ** 2)
        com_izz = w_izz * 0.0001 - mass * (x ** 2 + y ** 2)
        com_ixy = w_ixy * 0.0001 + mass * (x * y)
        com_iyz = w_iyz * 0.0001 + mass * (y * z)
        com_ixz = w_ixz * 0.0001 + mass * (x * z)

        inertia_tensor = [
            [com_ixx, com_ixy, com_ixz],
            [com_ixy, com_iyy, com_iyz],
            [com_ixz, com_iyz, com_izz],
        ]

        # Rotar al frame del link
        R = math_op.get_rotation_matrix(self.pose)
        R_T = math_op.matrix_transpose(R)
        I = math_op.matrix_multi(math_op.matrix_multi(R_T, inertia_tensor), R)

        ixx = I[0][0]
        iyy = I[1][1]
        izz = I[2][2]
        ixy = I[0][1]
        iyz = I[1][2]
        ixz = I[0][2]

        return [ixx, iyy, izz, ixy, iyz, ixz]

    def get_mass(self) -> float:
        return self.phyPro.mass

    def get_CoM_sdf(self):
        """
        CoM respecto al frame del link (para SDF).
        """
        roll = pitch = yaw = 0.0

        w_Cx = self.phyPro.centerOfMass.x
        w_Cy = self.phyPro.centerOfMass.y
        w_Cz = self.phyPro.centerOfMass.z

        w_Lx = self.pose.translation.x
        w_Ly = self.pose.translation.y
        w_Lz = self.pose.translation.z

        w_Lo_CoM = [
            [(w_Cx - w_Lx) * 0.01],
            [(w_Cy - w_Ly) * 0.01],
            [(w_Cz - w_Lz) * 0.01],
        ]

        L_R_w = [
            [self.pose.getCell(0, 0), self.pose.getCell(1, 0), self.pose.getCell(2, 0)],
            [self.pose.getCell(0, 1), self.pose.getCell(1, 1), self.pose.getCell(2, 1)],
            [self.pose.getCell(0, 2), self.pose.getCell(1, 2), self.pose.getCell(2, 2)],
        ]

        L_Lo_CoM = math_op.change_orientation(L_R_w, w_Lo_CoM)
        return [L_Lo_CoM[0][0], L_Lo_CoM[1][0], L_Lo_CoM[2][0], roll, pitch, yaw]

    def get_CoM_urdf(self):
        """
        CoM respecto al frame padre:
        - Si no tiene joint padre: frame del propio link.
        - Si tiene joint padre: se expresa en el frame del joint padre.
        """
        parent_joint = self.get_parent_joint()
        if parent_joint is None:
            return self.get_CoM_sdf()

        joint = Joint(parent_joint)
        parent_joint_frame = joint.get_joint_frame()

        w_Cx = self.phyPro.centerOfMass.x
        w_Cy = self.phyPro.centerOfMass.y
        w_Cz = self.phyPro.centerOfMass.z

        CoM_O = adsk.core.Point3D.create(w_Cx, w_Cy, w_Cz)
        from_o, from_x, from_y, from_z = parent_joint_frame.getAsCoordinateSystem()

        CoM_frame = adsk.core.Matrix3D.create()
        CoM_frame.setWithCoordinateSystem(CoM_O, from_x, from_y, from_z)

        transform = math_op.coordinate_transform(parent_joint_frame, CoM_frame)
        return math_op.matrix3d_2_pose(transform)

    # ==================== Mesh origin ====================

    def get_mesh_origin(self):
        """
        Origen del mesh (visual/collision) respecto al joint padre.
        Si no hay joint padre, se usa (0,0,0,0,0,0).
        """
        parent_joint = self.get_parent_joint()
        if parent_joint is None:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        joint = Joint(parent_joint)
        parent_joint_frame = joint.get_joint_frame()
        link_frame = self.pose

        transform = math_op.coordinate_transform(parent_joint_frame, link_frame)
        return math_op.matrix3d_2_pose(transform)

    # ==================== Visual / Collision bodies ====================

    def get_visual_body(self):
        """Devuelve body que contiene 'visual' en el nombre, si existe."""
        for body in self.link.bRepBodies:
            if "visual" in body.name:
                return body
        return None

    def get_collision_body(self):
        """Devuelve body que contiene 'collision' en el nombre, si existe."""
        for body in self.link.bRepBodies:
            if "collision" in body.name:
                return body
        return None
