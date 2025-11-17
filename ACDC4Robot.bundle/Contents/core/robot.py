import adsk.core
import adsk.fusion
import os
import math
import traceback

# =========================================================
# Utility functions
# =========================================================

def _get_app_ui():
    app = adsk.core.Application.get()
    ui = app.userInterface if app else None
    return app, ui


def _log(ui, msg: str):
    try:
        full = f"[ACDC4Robot] {msg}"
        print(full)
        if ui:
            palette = ui.palettes.itemById("TextCommands")
            if palette:
                palette.writeText(full)
    except:
        print("[ACDC4Robot][LOG-ERROR]", msg)


def _sanitize(name: str) -> str:
    if not name:
        return "link"
    bad = '<>:"/\\|?* ,.'
    for c in bad:
        name = name.replace(c, '_')
    if name and name[0].isdigit():
        name = "l_" + name
    return name


def _matrix_to_xyz_rpy(m: adsk.core.Matrix3D, design) -> tuple:
    """Mismo sistema antiguo (cm -> m, misma fórmula de RPY)."""
    if m is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    p = m.translation
    try:
        scale = design.unitsManager.convert(1.0, "cm", "m")
    except Exception:
        scale = 0.01

    x = p.x * scale
    y = p.y * scale
    z = p.z * scale

    r11, r12, r13 = m.getCell(0, 0), m.getCell(0, 1), m.getCell(0, 2)
    r21, r22, r23 = m.getCell(1, 0), m.getCell(1, 1), m.getCell(1, 2)
    r31, r32, r33 = m.getCell(2, 0), m.getCell(2, 1), m.getCell(2, 2)

    if abs(r31) < 1.0:
        pitch = math.asin(-r31)
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)
    else:
        pitch = math.pi / 2 if r31 <= -1.0 else -math.pi / 2
        roll = 0.0
        yaw = math.atan2(-r12, r22)

    return (x, y, z), (roll, pitch, yaw)


# =========================================================
# Recogida de cuerpos por occurrence
# =========================================================

def _collect_bodies_for_occurrence(occ: adsk.fusion.Occurrence, ui):
    comp = occ.component

    comp_brep_all = list(comp.bRepBodies)
    occ_brep_all = list(occ.bRepBodies)

    comp_brep_export = []
    occ_brep_export = []

    _log(ui, f"Analizando occurrence '{occ.name}'...")
    _log(ui, f"  [DEBUG] bRep(component) total={len(comp_brep_all)}, bRep(occurrence) total={len(occ_brep_all)}")

    for b in occ_brep_all:
        try:
            is_solid = getattr(b, "isSolid", None)
            is_visible = getattr(b, "isVisible", None)
        except:
            is_solid = None
            is_visible = None

        export = True
        _log(ui, f"    [occ.bRep] body='{b.name}' isSolid={is_solid} visible={is_visible} -> export={export}")
        if export:
            occ_brep_export.append(b)

    if not occ_brep_export:
        for b in comp_brep_all:
            try:
                is_solid = getattr(b, "isSolid", None)
                is_visible = getattr(b, "isVisible", None)
            except:
                is_solid = None
                is_visible = None

            export = True
            _log(ui, f"    [comp.bRep] body='{b.name}' isSolid={is_solid} visible={is_visible} -> export={export}")
            if export:
                comp_brep_export.append(b)

    comp_mesh_all = list(getattr(comp, "meshBodies", []))
    occ_mesh_all = list(getattr(occ, "meshBodies", []))

    comp_mesh_export = []
    occ_mesh_export = []

    _log(ui, f"  [DEBUG] mesh(component) total={len(comp_mesh_all)}, mesh(occurrence) total={len(occ_mesh_all)}")

    for m in occ_mesh_all:
        vis = getattr(m, "isVisible", None)
        export = True
        _log(ui, f"    [occ.mesh] body='{m.name}' visible={vis} -> export={export}")
        if export:
            occ_mesh_export.append(m)

    if not occ_mesh_export:
        for m in comp_mesh_all:
            vis = getattr(m, "isVisible", None)
            export = True
            _log(ui, f"    [comp.mesh] body='{m.name}' visible={vis} -> export={export}")
            if export:
                comp_mesh_export.append(m)

    total_exportables = (
        len(comp_brep_export)
        + len(occ_brep_export)
        + len(comp_mesh_export)
        + len(occ_mesh_export)
    )

    _log(
        ui,
        "  Bodies en '{}': "
        "bRep(comp)={}sel/{}tot, bRep(occ)={}sel/{}tot, "
        "mesh(comp)={}sel/{}tot, mesh(occ)={}sel/{}tot, total_usados={}".format(
            occ.name,
            len(comp_brep_export), len(comp_brep_all),
            len(occ_brep_export), len(occ_brep_all),
            len(comp_mesh_export), len(comp_mesh_all),
            len(occ_mesh_export), len(occ_mesh_all),
            total_exportables,
        ),
    )

    return comp_brep_export, occ_brep_export, comp_mesh_export, occ_mesh_export


# =========================================================
# Exporter
# =========================================================

class RobotExporter:
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

        self.links = []          # [{name, mesh, occ, item, kind}]
        self.joints = []         # [{name,parent,child,type,origin_xyz,origin_rpy,axis,limit}]
        self.occ_to_link = {}    # occurrenceKey -> main link name
        self.base_link_name = None
        self.single_link_no_joints = False

    # ------------------------------

    def export_all(self):
        try:
            self._log(f"=== EXPORTANDO '{self.robot_name}' ===")
            self._build_links_and_joints()
            self._export_meshes()
            self._write_urdf()
            self._log(f"[ACDC4Robot] Export completado en: {self.output_dir}")
        except Exception:
            msg = "Error en RobotExporter.export_all():\n\n" + traceback.format_exc()
            self._log(msg, also_messagebox=True)
            raise

    # ------------------------------

    def _log(self, msg: str, also_messagebox: bool = False):
        _log(self.ui, msg)
        if also_messagebox and self.ui:
            try:
                self.ui.messageBox(msg)
            except:
                pass

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

    def _occ_abs_pose(self, occ):
        if not occ:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        try:
            if hasattr(occ, "transform2"):
                return _matrix_to_xyz_rpy(occ.transform2, self.design)
        except:
            pass
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    # ------------------------------

    def _build_links_and_joints(self):
        root = self.design.rootComponent
        all_occs = list(root.allOccurrences)

        self._log(f"[DBG] _build_links_and_joints(): total occurrences = {len(all_occs)}")

        # 1) Links por cuerpos
        idx = 0
        for occ in all_occs:
            comp_brep, occ_brep, comp_mesh, occ_mesh = _collect_bodies_for_occurrence(occ, self.ui)

            bodies = occ_brep if occ_brep else comp_brep
            meshes = occ_mesh if occ_mesh else comp_mesh

            exported_items = []
            kind = None
            if bodies:
                exported_items = bodies
                kind = "brep"
            elif meshes:
                exported_items = meshes
                kind = "mesh"

            if not exported_items:
                self._log(f"[DBG] Occurrence '{occ.name}' sin cuerpos exportables -> omitida")
                continue

            occ_key = self._occ_key(occ)

            main_item = exported_items[0]
            main_name = _sanitize(f"link_{idx}_{occ.name}")
            main_mesh = f"{main_name}.stl"

            self.links.append({
                "name": main_name,
                "mesh": main_mesh,
                "occ": occ,
                "item": main_item,
                "kind": kind,
            })
            self.occ_to_link[occ_key] = main_name
            self._log(f"[DBG] Link principal para occurrence '{occ.name}': {main_name} (item='{main_item.name}')")
            idx += 1

            for i, extra in enumerate(exported_items[1:], start=1):
                extra_name = _sanitize(f"link_{idx}_{occ.name}_b{i}_{extra.name}")
                extra_mesh = f"{extra_name}.stl"

                self.links.append({
                    "name": extra_name,
                    "mesh": extra_mesh,
                    "occ": occ,
                    "item": extra,
                    "kind": kind,
                })
                idx += 1

                self._log(
                    f"  + Extra body link: {extra_name} (body='{extra.name}') fixed a {main_name}"
                )

                self.joints.append({
                    "name": f"fixed_extra_{extra_name}",
                    "parent": main_name,
                    "child": extra_name,
                    "type": "fixed",
                    "origin_xyz": (0.0, 0.0, 0.0),
                    "origin_rpy": (0.0, 0.0, 0.0),
                    "axis": (0.0, 0.0, 1.0),
                    "limit": None,
                })

        # 1.b) Cuerpos sueltos en root
        try:
            root_bodies = [b for b in root.bRepBodies if b.isSolid]
        except:
            root_bodies = []

        for i, body in enumerate(root_bodies):
            link_name = _sanitize(f"root_body_{i}_{body.name}")
            mesh_name = f"{link_name}.stl"
            self.links.append({
                "name": link_name,
                "mesh": mesh_name,
                "occ": None,
                "item": body,
                "kind": "brep",
            })
            self._log(f"[ACDC4Robot] Link root-body creado: {link_name}")

        if not self.links:
            raise RuntimeError("[ACDC4Robot] No se encontraron solids (occurrences ni root bodies).")

        has_fusion_joints = self._has_fusion_joints(root)

        if not has_fusion_joints and len(self.links) == 1:
            self.single_link_no_joints = True
            self.base_link_name = None
            self._log("[ACDC4Robot] Una sola pieza sin joints -> URDF con un solo link.")
            return

        # 2) base_link
        self.base_link_name = "base_link"
        self.links.insert(0, {
            "name": self.base_link_name,
            "mesh": None,
            "occ": None,
            "item": None,
            "kind": "none",
        })
        self._log("[ACDC4Robot] base_link creado en el origen.")

        # 3) Joints de Fusion (solo móviles; rígidos se ignoran)
        if has_fusion_joints:
            self._create_joints_from_fusion()

        joint_children = {j["child"] for j in self.joints}
        joint_parents = {j["parent"] for j in self.joints}

        # 4) Links que no salen en ningún joint -> fixed a base_link con pose absoluta
        for link in self.links:
            name = link["name"]
            if name == self.base_link_name:
                continue
            if name not in joint_children and name not in joint_parents:
                occ = link.get("occ")
                if occ is not None:
                    xyz, rpy = self._occ_abs_pose(occ)
                else:
                    xyz, rpy = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

                self.joints.append({
                    "name": f"fixed_{name}",
                    "parent": self.base_link_name,
                    "child": name,
                    "type": "fixed",
                    "origin_xyz": xyz,
                    "origin_rpy": rpy,
                    "axis": (0.0, 0.0, 1.0),
                    "limit": None,
                })
                self._log(f"[ACDC4Robot] Joint fijo (sin joints) base_link -> {name} XYZ={xyz} RPY={rpy}")

        # 5) Raíces de árboles de joints (por si quedan)
        joint_children = {j["child"] for j in self.joints}
        for link in self.links:
            name = link["name"]
            if name == self.base_link_name:
                continue
            if name in joint_parents and name not in joint_children:
                already = any(j["child"] == name for j in self.joints)
                if not already:
                    occ = link.get("occ")
                    if occ is not None:
                        xyz, rpy = self._occ_abs_pose(occ)
                    else:
                        xyz, rpy = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

                    self.joints.append({
                        "name": f"root_{name}",
                        "parent": self.base_link_name,
                        "child": name,
                        "type": "fixed",
                        "origin_xyz": xyz,
                        "origin_rpy": rpy,
                        "axis": (0.0, 0.0, 1.0),
                        "limit": None,
                    })
                    self._log(f"[ACDC4Robot] Joint fijo (root joint tree) base_link -> {name} XYZ={xyz} RPY={rpy}")

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
    # Fusion joints -> URDF (solo móviles)
    # -----------------------------------------------------

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

                jtype, axis, limit = self._map_joint_type_axis_limit(j, JointTypes)

                # *** CLAVE: ignorar joints rígidos (no afectan a la pose absoluta) ***
                if jtype == "fixed":
                    self._log(self.ui, f"[DBG] Joint rígido ignorado: {getattr(j,'name','(sin nombre)')}")
                    continue

                parent, child = self._pick_parent_child(l1, l2, used_children)
                used_children.add(child)

                ox, oy, oz, rr, pp, yy = self._get_joint_origin_from_fusion(j)

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
                self._log(f"[ACDC4Robot] Joint móvil Fusion -> URDF: {j_name} ({jtype}) {parent}->{child} "
                          f"origin=({ox},{oy},{oz}) rpy=({rr},{pp},{yy})")
            except Exception:
                self._log("[ACDC4Robot] Error procesando joint Fusion:\n" + traceback.format_exc())

    def _pick_parent_child(self, l1: str, l2: str, used_children: set):
        if l1 in used_children and l2 not in used_children:
            return l1, l2
        if l2 in used_children and l1 not in used_children:
            return l2, l1
        return (l1, l2) if l1 < l2 else (l2, l1)

    def _map_joint_type_axis_limit(self, j, JointTypes):
        jtype = "fixed"
        axis = (0.0, 0.0, 1.0)
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

        try:
            if motion and hasattr(motion, "rotationAxisVector"):
                av = motion.rotationAxisVector
                axis = (av.x, av.y, av.z)
            elif hasattr(j, "geometry") and j.geometry and hasattr(j.geometry, "primaryAxisVector"):
                av = j.geometry.primaryAxisVector
                axis = (av.x, av.y, av.z)
        except:
            pass

        length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
        if length > 1e-6:
            axis = (axis[0] / length, axis[1] / length, axis[2] / length)

        if jtype == "revolute":
            rl = getattr(motion, "rotationLimits", None)
            if rl:
                lo = hi = None
                if getattr(rl, "isMinimumValueEnabled", False):
                    lo = rl.minimumValue
                if getattr(rl, "isMaximumValueEnabled", False):
                    hi = rl.maximumValue
                if lo is not None and hi is not None and hi > lo:
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

    def _get_joint_origin_from_fusion(self, j):
        try:
            geo = getattr(j, "geometryOrOriginTwo", None)
            if geo and hasattr(geo, "transform"):
                m = geo.transform
                (ox, oy, oz), (rr, pp, yy) = _matrix_to_xyz_rpy(m, self.design)
                return ox, oy, oz, rr, pp, yy
        except:
            pass
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    # -----------------------------------------------------
    # STL export
    # -----------------------------------------------------

    def _export_meshes(self):
        export_mgr = self.design.exportManager
        for link in self.links:
            mesh_name = link.get("mesh")
            if not mesh_name:
                continue

            item = link.get("item")
            occ = link.get("occ")
            export_target = item if item is not None else occ
            if not export_target:
                continue

            stl_path = os.path.join(self.meshes_dir, mesh_name)

            try:
                stl_opts = export_mgr.createSTLExportOptions(export_target, stl_path)
                try:
                    stl_opts.meshRefinement = adsk.fusion.MeshRefinementOptions.High
                except:
                    try:
                        stl_opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                    except:
                        pass
                if hasattr(stl_opts, "isBinaryFormat"):
                    stl_opts.isBinaryFormat = True

                export_mgr.execute(stl_opts)
                self._log(f"[ACDC4Robot] STL generado: {stl_path}")
            except Exception:
                self._log(
                    f"[ACDC4Robot] Error exportando STL para {link.get('name', '?')}:\n"
                    + traceback.format_exc()
                )

    # -----------------------------------------------------
    # URDF writer
    # -----------------------------------------------------

    def _normalize_angle_limits(self, lo, hi):
        try:
            if abs(lo) > 2 * math.pi or abs(hi) > 2 * math.pi:
                lo = math.radians(lo)
                hi = math.radians(hi)
        except:
            pass
        return lo, hi

    def _write_urdf(self):
        urdf_path = os.path.join(self.output_dir, f"{self.robot_name}.urdf")
        lines = [f'<robot name="{self.robot_name}">']

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
                    f'scale="0.001 0.001 0.001"/></geometry>'
                )
                lines.append('    </visual>')
            lines.append('  </link>')
            lines.append('</robot>')

            with open(urdf_path, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
            self._log(f"[ACDC4Robot] URDF generado (modelo único): {urdf_path}")
            return

        for link in self.links:
            name = link["name"]
            mesh = link.get("mesh")

            lines.append(f'  <link name="{name}">')
            if mesh:
                lines.append('    <visual>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(
                    f'      <geometry><mesh filename="meshes/{mesh}" '
                    f'scale="0.001 0.001 0.001"/></geometry>'
                )
                lines.append('    </visual>')
                lines.append('    <collision>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(
                    f'      <geometry><mesh filename="meshes/{mesh}" '
                    f'scale="0.001 0.001 0.001"/></geometry>'
                )
                lines.append('    </collision>')
            lines.append('    <inertial>')
            lines.append('      <mass value="1.0"/>')
            lines.append('      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>')
            lines.append('    </inertial>')
            lines.append('  </link>')

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
                if jtype == "revolute":
                    lo, hi = self._normalize_angle_limits(lo, hi)
                lines.append(f'    <limit lower="{lo}" upper="{hi}" effort="10" velocity="1"/>')
            lines.append('  </joint>')

        lines.append('</robot>')

        with open(urdf_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        self._log(f"[ACDC4Robot] URDF generado: {urdf_path}")


# =========================================================
# Entrada pública
# =========================================================

def export_robot(robot_name: str = "acdc_robot", base_output_dir: str = None, *args, **kwargs):
    if robot_name is None and args:
        robot_name = args[0]
    if base_output_dir is None and len(args) > 1:
        base_output_dir = args[1]

    exporter = RobotExporter(robot_name, base_output_dir)
    exporter.export_all()
