# -*- coding: utf-8 -*-
"""
Joint helper: lee joints de Fusion y los mapea a URDF.
"""

import adsk.core
import adsk.fusion
import adsk

from . import utils
from . import math_operation as math_op


class Joint:
    def __init__(self, joint: adsk.fusion.Joint) -> None:
        self.joint = joint
        self.name = joint.name
        # Por convención de este exporter:
        # parent = occurrenceTwo, child = occurrenceOne
        self.parent = joint.occurrenceTwo
        self.child = joint.occurrenceOne

    # -------------------- Nombres --------------------

    def get_name(self) -> str:
        parent_name = utils.get_valid_filename(self.parent.fullPathName)
        return parent_name + "_" + utils.get_valid_filename(self.name)

    def get_parent(self) -> str:
        if self.parent.component.name == "base_link":
            return "base_link"
        return utils.get_valid_filename(self.parent.fullPathName)

    def get_child(self) -> str:
        if self.child.component.name == "base_link":
            return "base_link"
        return utils.get_valid_filename(self.child.fullPathName)

    # -------------------- Tipo / límites --------------------

    def get_urdf_joint_type(self) -> str:
        # 0=fixed, 1=revolute, 2=slider en esta lib original
        jm = self.joint.jointMotion
        jt = jm.jointType
        if jt == 0:
            return "fixed"
        if jt == 1:
            # revolute o continuous según tenga límites
            if self.get_limits() is None:
                return "continuous"
            return "revolute"
        if jt == 2:
            return "prismatic"
        return "fixed"

    def get_limits(self):
        jm = self.joint.jointMotion
        jt = jm.jointType

        if jt == 1:  # revolute
            if jm.rotationLimits.isMaximumValueEnabled and jm.rotationLimits.isMinimumValueEnabled:
                lower = jm.rotationLimits.minimumValue
                upper = jm.rotationLimits.maximumValue
                return [round(lower, 6), round(upper, 6)]
            return None

        if jt == 2:  # slide
            if jm.slideLimits.isMaximumValueEnabled and jm.slideLimits.isMinimumValueEnabled:
                lower = jm.slideLimits.minimumValue * 0.01
                upper = jm.slideLimits.maximumValue * 0.01
                return [round(lower, 6), round(upper, 6)]
            return None

        return None

    # -------------------- Frames --------------------

    def get_joint_frame(self) -> adsk.core.Matrix3D:
        """
        Frame del joint en el mundo, usando geometryOrOriginTwo.
        """
        if self.joint.geometryOrOriginTwo == adsk.fusion.JointOrigin:
            w_P_J = self.joint.geometryOrOriginTwo.geometry.origin.asArray()
        else:
            w_P_J = self.joint.geometryOrOriginTwo.origin.asArray()

        w_P_J = [round(v, 6) for v in w_P_J]

        zAxis = self.joint.geometryOrOriginTwo.primaryAxisVector
        xAxis = self.joint.geometryOrOriginTwo.secondaryAxisVector
        yAxis = self.joint.geometryOrOriginTwo.thirdAxisVector

        origin = adsk.core.Point3D.create(w_P_J[0], w_P_J[1], w_P_J[2])

        joint_frame = adsk.core.Matrix3D.create()
        joint_frame.setWithCoordinateSystem(origin, xAxis, yAxis, zAxis)
        return joint_frame

    def get_urdf_origin(self):
        """
        Origen URDF = transform padre->joint.
        """
        parent_link = self.parent

        def get_parent_joint(link):
            joint_list = link.joints
            for j in joint_list:
                if j.occurrenceOne == link:
                    return j
            return None

        parent_joint = get_parent_joint(parent_link)
        if parent_joint is None:
            parent_frame = parent_link.transform2
        else:
            # frame del joint padre como parent_frame
            if parent_joint.geometryOrOriginTwo == adsk.fusion.JointOrigin:
                w_P_J = parent_joint.geometryOrOriginTwo.geometry.origin.asArray()
            else:
                w_P_J = parent_joint.geometryOrOriginTwo.origin.asArray()
            w_P_J = [round(v, 6) for v in w_P_J]
            zAxis = parent_joint.geometryOrOriginTwo.primaryAxisVector
            xAxis = parent_joint.geometryOrOriginTwo.secondaryAxisVector
            yAxis = parent_joint.geometryOrOriginTwo.thirdAxisVector
            origin = adsk.core.Point3D.create(w_P_J[0], w_P_J[1], w_P_J[2])
            parent_frame = adsk.core.Matrix3D.create()
            parent_frame.setWithCoordinateSystem(origin, xAxis, yAxis, zAxis)

        joint_frame = self.get_joint_frame()
        transform = math_op.coordinate_transform(parent_frame, joint_frame)
        return math_op.matrix3d_2_pose(transform)

    # -------------------- Eje --------------------

    def get_axis_for_urdf(self):
        jm = self.joint.jointMotion
        jt = jm.jointType

        if jt == 0:
            return None

        if jt == 1:
            w_axis = jm.rotationAxisVector.asArray()
        elif jt == 2:
            w_axis = jm.slideDirectionVector.asArray()
        else:
            w_axis = jm.rotationAxisVector.asArray()

        joint_frame = self.get_joint_frame()
        w_R_J = math_op.get_rotation_matrix(joint_frame)
        J_R_w = math_op.matrix_transpose(w_R_J)

        w_vec = [[w_axis[0]], [w_axis[1]], [w_axis[2]]]
        J_vec = math_op.change_orientation(J_R_w, w_vec)
        return [J_vec[0][0], J_vec[1][0], J_vec[2][0]]
