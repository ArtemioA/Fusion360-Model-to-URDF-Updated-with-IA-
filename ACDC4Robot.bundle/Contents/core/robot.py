import adsk.core
import adsk.fusion
import os
import math
import traceback

# =========================================================
# Utility helpers
# =========================================================

def _get_app_ui():
    app = adsk.core.Application.get()
    ui = app.userInterface if app else None
    return app, ui


def _sanitize(name: str) -> str:
    """Limpia nombres para URDF y archivos."""
    if not name:
        return "link"
    bad = '<>:"/\\|?* '
    for c in bad:
        name = name.replace(c, '_')
    return name


def _matrix_to_xyz_rpy(m: adsk.core.Matrix3D, design) -> tuple:
    """Convierte Matrix3D de Fusion a (xyz, rpy) en metros."""
    p = m.translation
    try:
        du = design.unitsManager.defaultLengthUnits
        scale = design.unitsManager.convert(1.0, du, "m")
    except Exception:
        scale = 0.001  # assume mm -> m

    x = p.x * scale
    y = p.y * scale
    z = p.z * scale

    r11, r12, r13 = m.getCell(0, 0), m.getCell(0, 1), m.getCell(0, 2)
    r21, r22, r23 = m.getCell(1, 0), m.getCell(1, 1), m.getCell(1, 2)
    r31, r32, r33 = m.getCell(2, 0), m.getCell(2, 1), m.getCell(2, 2)

    # roll (X), pitch (Y), yaw (Z) - ZYX
    if abs(r31) < 1.0:
        pitch = math.asin(-r31)
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)
    else:
        pitch = math.pi / 2 if r31 <= -1.0 else -math.pi / 2
        roll = 0.0
        yaw = math.atan2(-r12, r22)

    return (x, y, z), (roll, pitch, yaw)


def _copy_matrix(m: adsk.core.Matrix3D) -> adsk.core.Matrix3D:
    nm = adsk.core.Matrix3D.create()
    nm.copy(m)
    return nm


def _inverse_matrix(m: adsk.core.Matrix3D) -> adsk.core.Matrix3D:
    inv = adsk.core.Matrix3D.create()
    inv.copy(m)
    inv.invert()
    return inv


def _mul_matrix(a: adsk.core.Matrix3D, b: adsk.core.Matrix3D) -> adsk.core.Matrix3D:
    """Devuelve a * b."""
    r = adsk.core.Matrix3D.create()
    r.copy(a)
    r.transformBy(b)
    return r


def _axis_world_to_parent(axis, parent_world: adsk.core.Matrix3D):
    """Convierte eje de coords mundo a coords del parent."""
    ax, ay, az = axis

    r00 = parent_world.getCell(0, 0)
    r01 = parent_world.getCell(0, 1)
    r02 = parent_world.getCell(0, 2)
    r10 = parent_world.getCell(1, 0)
    r11 = parent_world.getCell(1, 1)
    r12 = parent_world.getCell(1, 2)
    r20 = parent_world.getCell(2, 0)
    r21 = parent_world.getCell(2, 1)
    r22 = parent_world.getCell(2, 2)

    px = r00 * ax + r10 * ay + r20 * az
    py = r01 * ax + r11 * ay + r21 * az
    pz = r02 * ax + r12 * ay + r22 * az

    length = math.sqrt(px * px + py * py + pz * pz)
    if length > 1e-9:
        px /= length
        py /= length
        pz /= length

    return (px, py, pz)


# =========================================================
# Robot Exporter
# =========================================================

class RobotExporter:
    """
    Joint-aware exporter con soporte para componentes azules / subensamblajes:

      - Un link por occurrence con sólidos:
          * cuerpos directos en su componente, o
          * cualquier sólido en sus descendientes (icono azul).
      - STL exportado desde la occurrence (incluye subárbol).
      - Joints de Fusion -> joints URDF (revolute/prismatic/fixed/continuous)
        usando transformaciones relativas correctas.
      - Links sin joints:
          * fixed a base_link usando su pose real (occ.transform2).
      - Raíces de árboles de joints:
          * también ancladas a base_link con su pose real.
      - Si solo hay una pieza sin joints:
          * URDF simple de un solo link.
    """

    def __init__(self, robot_name: str, base_output_dir: str = None):
        self.app, self.ui = _get_app_ui()
        if not self.app:
            raise RuntimeError("No se pudo obtener la aplicación de Fusion 360.")

        product = self.app.activeProduct
        self.design = adsk.fusion.Design.cast(product)
        if not self.design:
            raise RuntimeError("El producto activo no es un diseño de Fusion 360.")

        self.robot_name = _sanitize(robot_name) or "acdc_robot"

        if base_output_dir is None:
            home = os.path.expanduser("~")
            base_output_dir = os.path.join(home, "Desktop")

        self.output_dir = os.path.join(base_output_dir, f"{self.robot_name}_urdf")
        self.meshes_dir = os.path.join(self.output_dir, "meshes")
        os.makedirs(self.meshes_dir, exist_ok=True)

        self.links = []          # [{name, mesh, occ}]
        self.joints = []         # [{name,parent,child,type,origin_xyz,origin_rpy,axis,limit}]
        self.occ_to_link = {}    # occKey -> link name
        self.link_world = {}     # link_name -> Matrix3D world
        self.base_link_name = None
        self.single_link_no_joints = False

    # -----------------------------------------------------
    # Public
    # -----------------------------------------------------

    def export_all(self):
        try:
            self._log(f"[ACDC4Robot] Exportando robot '{self.robot_name}'...")
            self._build_links()
            self._build_joints()
            self._export_meshes()
            self._write_urdf()
            self._log(f"[ACDC4Robot] Export completado en:\n{self.output_dir}")
        except Exception:
            msg = "Error en RobotExporter.export_all():\n\n" + traceback.format_exc()
            self._log(msg, also_messagebox=True)
            raise

    # -----------------------------------------------------
    # Logging
    # -----------------------------------------------------

    def _log(self, msg: str, also_messagebox: bool = False):
        try:
            if self.ui:
                try:
                    palettes = self.ui.palettes
                    text_palette = palettes.itemById('TextCommands')
                    if text_palette:
                        text_palette.writeText(msg)
                except:
                    pass

                if also_messagebox:
                    try:
                        self.ui.messageBox(msg)
                    except:
                        pass
            else:
                print(msg)
        except:
            try:
                print(msg)
            except:
                pass

    # -----------------------------------------------------
    # Helpers
    # -----------------------------------------------------

    def _occ_key(self, occ) -> str:
        if occ is None:
            return ""
        try:
            token = getattr(occ, "entityToken", None)
            if token:
                return token
        except:
            pass
        try:
            fp = getattr(occ, "fullPathName", None)
            if fp:
                return fp
        except:
            pass
        try:
            return f"{occ.name}_{id(occ)}"
        except:
            return str(id(occ))

    def _get_world_matrix(self, occ):
        if not occ:
            m = adsk.core.Matrix3D.create()
            return m
        try:
            m = occ.transform2
            if m:
                return m
        except:
            pass
        m = adsk.core.Matrix3D.create()
        return m

    def _occ_has_descendant_solid(self, occ) -> bool:
        """
        True si la occurrence o alguno de sus descendientes tiene cuerpos sólidos.

        Esto es lo que permite que el "componente azul" (subensamblaje con sólidos dentro)
        se trate como un link y se exporte a STL.
        """
        try:
            comp = occ.component
        except:
            comp = None

        if not comp:
            return False

        # cuerpos directos
        for b in getattr(comp, "bRepBodies", []):
            try:
                if b and b.isSolid:
                    return True
            except:
                continue

        # hijos
        try:
            child_occs = comp.occurrences
        except:
            child_occs = None

        if child_occs:
            for child in child_occs:
                if self._occ_has_descendant_solid(child):
                    return True

        return False

    # -----------------------------------------------------
    # Build links (with blue-components support)
    # -----------------------------------------------------

    def _build_links(self):
        root = self.design.rootComponent
        all_occs = list(root.allOccurrences)

        idx = 0
        seen = set()

        for occ in all_occs:
            comp = getattr(occ, "component", None)
            if not comp:
                continue

            has_bodies = False

            # Direct solids on this component
            for b in getattr(comp, "bRepBodies", []):
                try:
                    if b and b.isSolid:
                        has_bodies = True
                        break
                except:
                    continue

            # Fallback: solids in descendants (blue subassembly)
            if not has_bodies and self._occ_has_descendant_solid(occ):
                has_bodies = True

            if not has_bodies:
                continue

            occ_key = getattr(occ, "fullPathName", None) or occ.name
            if occ_key in seen:
                # evita links duplicados para la misma rama
                continue
            seen.add(occ_key)

            link_name = _sanitize(f"link_{idx}_{occ.name}")
            mesh_name = f"{link_name}.stl"

            self.links.append({"name": link_name, "mesh": mesh_name, "occ": occ})
            self.occ_to_link[self._occ_key(occ)] = link_name
            self.link_world[link_name] = self._get_world_matrix(occ)

            idx += 1

        if not self.links:
            raise RuntimeError("[ACDC4Robot] No se encontraron occurrences con sólidos.")

        has_joints = self._has_fusion_joints(root)

        # Caso: una pieza sin joints -> URDF mínimo
        if not has_joints and len(self.links) == 1:
            self.single_link_no_joints = True
            self.base_link_name = None
            self._log("[ACDC4Robot] Una sola pieza sin joints -> URDF con un link.")
            return

        # Crear base_link en el origen
        self.base_link_name = "base_link"
        self.links.insert(0, {"name": self.base_link_name, "mesh": None, "occ": None})
        self.link_world[self.base_link_name] = adsk.core.Matrix3D.create()
        self._log("[ACDC4Robot] base_link creado en el origen.")

    def _has_fusion_joints(self, root) -> bool:
        try:
            if len(root.joints) > 0:
                return True
        except:
            pass
        try:
            if len(root.asBuiltJoints) > 0:
                return True
        except:
            pass
        return False

    # -----------------------------------------------------
    # Build joints
    # -----------------------------------------------------

    def _build_joints(self):
        if self.single_link_no_joints:
            return

        self._create_joints_from_fusion()

        joint_children = {j["child"] for j in self.joints}
        joint_parents = {j["parent"] for j in self.joints}

        # Links totalmente aislados -> fixed a base_link con pose real
        for link in self.links:
            name = link["name"]
            if name == self.base_link_name:
                continue
            if name not in joint_children and name not in joint_parents:
                m = self.link_world.get(name)
                (x, y, z), (r, p, yw) = _matrix_to_xyz_rpy(m, self.design)
                self.joints.append({
                    "name": f"fixed_{name}",
                    "parent": self.base_link_name,
                    "child": name,
                    "type": "fixed",
                    "origin_xyz": (x, y, z),
                    "origin_rpy": (r, p, yw),
                    "axis": (0.0, 0.0, 1.0),
                    "limit": None,
                })
                self._log(f"[ACDC4Robot] Link aislado fijado: base_link -> {name}")

        # Raíces de árboles de joints -> también anclar a base_link
        joint_children = {j["child"] for j in self.joints}
        joint_parents = {j["parent"] for j in self.joints}
        for link in self.links:
            name = link["name"]
            if name in (self.base_link_name, None):
                continue
            if name in joint_parents and name not in joint_children:
                # si ya tiene padre, saltamos
                if any(j["child"] == name for j in self.joints):
                    continue
                m = self.link_world.get(name)
                (x, y, z), (r, p, yw) = _matrix_to_xyz_rpy(m, self.design)
                self.joints.append({
                    "name": f"root_{name}",
                    "parent": self.base_link_name,
                    "child": name,
                    "type": "fixed",
                    "origin_xyz": (x, y, z),
                    "origin_rpy": (r, p, yw),
                    "axis": (0.0, 0.0, 1.0),
                    "limit": None,
                })
                self._log(
                    f"[ACDC4Robot] Raíz de joints anclada: base_link -> {name}"
                )

    def _create_joints_from_fusion(self):
        root = self.design.rootComponent
        all_fusion_joints = []

        try:
            all_fusion_joints.extend(root.joints)
        except:
            pass
        try:
            all_fusion_joints.extend(root.asBuiltJoints)
        except:
            pass

        if not all_fusion_joints:
            return

        try:
            from adsk.fusion import JointTypes
        except Exception:
            JointTypes = None

        used_children = set()

        for j in all_fusion_joints:
            try:
                occ1 = getattr(j, "occurrenceOne", None)
                occ2 = getattr(j, "occurrenceTwo", None)

                k1 = self._occ_key(occ1)
                k2 = self._occ_key(occ2)
                l1 = self.occ_to_link.get(k1)
                l2 = self.occ_to_link.get(k2)
                if not l1 or not l2:
                    continue

                parent, child = self._pick_parent_child(l1, l2, used_children)
                used_children.add(child)

                Tp = self.link_world.get(parent)
                Tc = self.link_world.get(child)
                if not Tp or not Tc:
                    continue

                invTp = _inverse_matrix(Tp)
                Tpc = _mul_matrix(invTp, Tc)
                (ox, oy, oz), (rr, pp, yy) = _matrix_to_xyz_rpy(Tpc, self.design)

                jtype, axis_world, limit = self._map_joint_type_axis_limit(j, JointTypes)
                axis = _axis_world_to_parent(axis_world, Tp) if axis_world else (0.0, 0.0, 1.0)

                j_name = _sanitize(getattr(j, "name", "") or f"joint_{parent}_to_{child}")

                self.joints.append({
                    "name": j_name,
                    "parent": parent,
                    "child": child,
                    "type": jtype,
                    "origin_xyz": (ox, oy, oz),
                    "origin_rpy": (rr, pp, yy),
                    "axis": axis,
                    "limit": limit,
                })
                self._log(
                    f"[ACDC4Robot] Joint Fusion->URDF: {j_name} ({jtype}) {parent}->{child}"
                )

            except Exception:
                self._log(
                    "[ACDC4Robot] Error procesando joint Fusion:\n" + traceback.format_exc()
                )

    def _pick_parent_child(self, l1: str, l2: str, used_children: set):
        if l1 in used_children and l2 not in used_children:
            return l2, l1
        if l2 in used_children and l1 not in used_children:
            return l1, l2
        return (l1, l2) if l1 < l2 else (l2, l1)

    def _map_joint_type_axis_limit(self, j, JointTypes):
        jtype = "fixed"
        axis = None
        limit = None

        motion = getattr(j, "jointMotion", None)
        jt = getattr(motion, "jointType", None)

        if JointTypes and jt is not None:
            try:
                if jt == JointTypes.RigidJointType:
                    jtype = "fixed"
                elif jt == JointTypes.RevoluteJointType:
                    jtype = "revolute"
                elif jt == JointTypes.SliderJointType:
                    jtype = "prismatic"
            except:
                pass

        # axis in world
        try:
            if motion and hasattr(motion, "rotationAxisVector"):
                av = motion.rotationAxisVector
                axis = (av.x, av.y, av.z)
            elif hasattr(j, "geometry") and j.geometry and hasattr(j.geometry, "primaryAxisVector"):
                av = j.geometry.primaryAxisVector
                axis = (av.x, av.y, av.z)
        except:
            axis = None

        if axis:
            length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
            if length > 1e-9:
                axis = (axis[0]/length, axis[1]/length, axis[2]/length)

        if jtype == "revolute":
            rl = getattr(motion, "rotationLimits", None)
            if rl:
                lo = hi = None
                if getattr(rl, "isMinimumValueEnabled", False):
                    lo = rl.minimumValue
                if getattr(rl, "isMaximumValueEnabled", False):
                    hi = rl.maximumValue
                if lo is not None and hi is not None and hi > lo:
                    if abs(lo) > 2*math.pi or abs(hi) > 2*math.pi:
                        lo = math.radians(lo)
                        hi = math.radians(hi)
                    limit = (lo, hi)
            if limit is None:
                jtype = "continuous"

        elif jtype == "prismatic":
            sl = getattr(motion, "slideLimits", None)
            if sl:
                lo = hi = None
                if getattr(sl, "isMinimumValueEnabled", False):
                    lo = sl.minimumValue
                if getattr(sl, "isMaximumValueEnabled", False):
                    hi = sl.maximumValue
                if lo is not None and hi is not None and hi > lo:
                    limit = (lo, hi)
            if limit is None:
                limit = (-0.1, 0.1)

        return jtype, axis, limit

    # -----------------------------------------------------
    # STL export (occurrences + blue components)
    # -----------------------------------------------------

    def _export_meshes(self):
        export_mgr = self.design.exportManager

        for link in self.links:
            mesh_name = link.get("mesh")
            occ = link.get("occ")
            if not mesh_name or not occ:
                continue

            stl_path = os.path.join(self.meshes_dir, mesh_name)

            try:
                # Exportar desde la occurrence:
                # para componentes azules incluye todos sus descendientes.
                opts = export_mgr.createSTLExportOptions(occ, stl_path)
                try:
                    opts.meshRefinement = adsk.fusion.MeshRefinementOptions.High
                except AttributeError:
                    try:
                        opts.meshRefinement = (
                            adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                        )
                    except AttributeError:
                        pass

                if hasattr(opts, "isBinaryFormat"):
                    opts.isBinaryFormat = True

                export_mgr.execute(opts)
                self._log(f"[ACDC4Robot] STL generado: {stl_path}")
            except Exception:
                self._log(
                    f"[ACDC4Robot] Error exportando STL para {link.get('name','?')}:\n"
                    + traceback.format_exc()
                )

    # -----------------------------------------------------
    # URDF writer
    # -----------------------------------------------------

    def _write_urdf(self):
        urdf_path = os.path.join(self.output_dir, f"{self.robot_name}.urdf")
        sx = sy = sz = 0.001  # mm -> m

        lines = [f'<robot name="{self.robot_name}">']

        # Caso simple: un único link sin joints
        if self.single_link_no_joints and len(self.links) == 1:
            link = self.links[0]
            name = link["name"]
            mesh = link.get("mesh")

            lines.append(f'  <link name="{name}">')
            if mesh:
                lines.append('    <visual>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(
                    f'      <geometry><mesh filename="meshes/{mesh}" '
                    f'scale="{sx} {sy} {sz}"/></geometry>'
                )
                lines.append('    </visual>')
            lines.append('  </link>')
            lines.append('</robot>')

            with open(urdf_path, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
            self._log(f"[ACDC4Robot] URDF generado (modelo único): {urdf_path}")
            return

        # Links
        for link in self.links:
            name = link["name"]
            mesh = link.get("mesh")

            lines.append(f'  <link name="{name}">')

            if mesh:
                lines.append('    <visual>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(
                    f'      <geometry><mesh filename="meshes/{mesh}" '
                    f'scale="{sx} {sy} {sz}"/></geometry>'
                )
                lines.append('    </visual>')

                lines.append('    <collision>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(
                    f'      <geometry><mesh filename="meshes/{mesh}" '
                    f'scale="{sx} {sy} {sz}"/></geometry>'
                )
                lines.append('    </collision>')

            lines.append('    <inertial>')
            lines.append('      <mass value="1.0"/>')
            lines.append('      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>')
            lines.append('    </inertial>')

            lines.append('  </link>')

        # Joints
        for j in self.joints:
            name = j["name"]
            parent = j["parent"]
            child = j["child"]
            jtype = j["type"]
            ox, oy, oz = j["origin_xyz"]
            rr, pp, yy = j["origin_rpy"]
            ax, ay, az = j["axis"]
            limit = j.get("limit")

            lines.append(f'  <joint name="{name}" type="{jtype}">')
            lines.append(f'    <origin xyz="{ox} {oy} {oz}" rpy="{rr} {pp} {yy}"/>')
            lines.append(f'    <parent link="{parent}"/>')
            lines.append(f'    <child link="{child}"/>')

            if jtype in ("revolute", "continuous", "prismatic"):
                lines.append(f'    <axis xyz="{ax} {ay} {az}"/>')

            if limit is not None and jtype in ("revolute", "prismatic"):
                lo, hi = limit
                lines.append(
                    f'    <limit lower="{lo}" upper="{hi}" effort="10" velocity="1"/>'
                )

            lines.append('  </joint>')

        lines.append('</robot>')

        with open(urdf_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        self._log(f"[ACDC4Robot] URDF generado: {urdf_path}")


# =========================================================
# Entry called from acdc4robot.py
# =========================================================

def export_robot(robot_name: str, base_output_dir: str = None, *args, **kwargs):
    """
    Compatible con:
        export_robot(name, base_output_dir, ...)
    """
    if robot_name is None and len(args) > 0:
        robot_name = args[0]
    if base_output_dir is None and len(args) > 1:
        base_output_dir = args[1]

    exporter = RobotExporter(robot_name, base_output_dir)
    exporter.export_all()
