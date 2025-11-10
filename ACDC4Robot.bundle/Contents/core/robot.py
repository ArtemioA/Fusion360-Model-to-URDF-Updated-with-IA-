import adsk.core
import adsk.fusion
import os
import math
import traceback


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
        scale = design.unitsManager.convert(1.0, "cm", "m")
    except Exception:
        scale = 0.01

    x, y, z = p.x * scale, p.y * scale, p.z * scale

    r11, r12, r13 = m.getCell(0, 0), m.getCell(0, 1), m.getCell(0, 2)
    r21, r22, r23 = m.getCell(1, 0), m.getCell(1, 1), m.getCell(1, 2)
    r31, r32, r33 = m.getCell(2, 0), m.getCell(2, 1), m.getCell(2, 2)

    # Convención: roll (X), pitch (Y), yaw (Z)
    if abs(r31) < 1.0:
        pitch = math.asin(-r31)
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)
    else:
        pitch = math.pi / 2 if r31 <= -1.0 else -math.pi / 2
        roll = 0.0
        yaw = math.atan2(-r12, r22)

    return (x, y, z), (roll, pitch, yaw)


class RobotExporter:
    """Exporta todos los sólidos del diseño como STL y genera un URDF con poses correctas."""

    def __init__(self, robot_name: str, base_output_dir: str = None):
        self.app, self.ui = _get_app_ui()
        if not self.app:
            raise RuntimeError("No se pudo obtener la aplicación de Fusion 360.")

        product = self.app.activeProduct
        self.design = adsk.fusion.Design.cast(product)
        if not self.design:
            raise RuntimeError("El producto activo no es un diseño de Fusion 360.")

        self.robot_name = _sanitize(robot_name) or "acdc_robot"

        # Si no se pasa carpeta base, usar Desktop
        if base_output_dir is None:
            home = os.path.expanduser("~")
            base_output_dir = os.path.join(home, "Desktop")

        self.output_dir = os.path.join(base_output_dir, f"{self.robot_name}_urdf")
        self.meshes_dir = os.path.join(self.output_dir, "meshes")
        os.makedirs(self.meshes_dir, exist_ok=True)

        self.links = []
        self.joints = []

    # =========================================================
    # API principal
    # =========================================================
    def export_all(self):
        try:
            self._log(f"[ACDC4Robot] Exportando robot '{self.robot_name}'...")
            self._build_links_and_joints()
            self._export_meshes()
            self._write_urdf()
            self._log(f"[ACDC4Robot] Export completado en:\n{self.output_dir}")
        except Exception:
            if self.ui:
                self.ui.messageBox("Error en RobotExporter.export_all():\n\n" + traceback.format_exc())
            raise

    # =========================================================
    # Construcción de links/joints
    # =========================================================
    def _build_links_and_joints(self):
        root = self.design.rootComponent

        # base_link en el origen
        base_link_name = "base_link"
        self.links.append({"name": base_link_name, "mesh": None})

        all_occs = root.allOccurrences
        idx = 0

        for occ in all_occs:
            comp = occ.component

            # Determinar si la occurrence o su componente tienen sólidos
            has_bodies = False
            try:
                if comp and any(b.isSolid for b in comp.bRepBodies):
                    has_bodies = True
                elif hasattr(occ, "bRepBodies") and any(b.isSolid for b in occ.bRepBodies):
                    has_bodies = True
            except:
                pass

            if not has_bodies:
                continue

            link_name = _sanitize(f"link_{idx}_{occ.name}")
            mesh_name = f"{link_name}.stl"
            idx += 1

            # Obtener posición y orientación de la occurrence
            try:
                xyz, rpy = _matrix_to_xyz_rpy(occ.transform2, self.design)
            except Exception:
                xyz, rpy = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

            self.links.append({
                "name": link_name,
                "mesh": mesh_name,
            })

            # Joint fijo con la pose real del componente
            self.joints.append({
                "name": f"joint_{link_name}",
                "parent": base_link_name,
                "child": link_name,
                "type": "fixed",
                "origin_xyz": xyz,
                "origin_rpy": rpy,
                "axis": (0.0, 0.0, 1.0),
            })

        # Si solo existe base_link (raro)
        if len(self.links) == 1:
            self.links[0]["mesh"] = f"{base_link_name}.stl"

    # =========================================================
    # Export de mallas STL
    # =========================================================
    def _export_meshes(self):
        export_mgr = self.design.exportManager
        root = self.design.rootComponent
        all_occs = list(root.allOccurrences)

        for link in self.links:
            mesh_name = link.get("mesh")
            if not mesh_name:
                continue

            stl_path = os.path.join(self.meshes_dir, mesh_name)
            base_name = os.path.splitext(mesh_name)[0]

            # Buscar la occurrence correspondiente
            target_occ = None
            for occ in all_occs:
                if _sanitize(occ.name) in base_name:
                    target_occ = occ
                    break

            export_target = target_occ or root

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
                self._log(f"[ACDC4Robot] Error exportando STL para {link.get('name','?')}")
                self._log(traceback.format_exc())

    # =========================================================
    # Generación del archivo URDF
    # =========================================================
    def _write_urdf(self):
        urdf_path = os.path.join(self.output_dir, f"{self.robot_name}.urdf")

        lines = []
        lines.append(f'<robot name="{self.robot_name}">')

        # Links
        for link in self.links:
            name = link["name"]
            mesh = link.get("mesh")
            lines.append(f'  <link name="{name}">')

            if mesh:
                lines.append('    <visual>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(f'      <geometry><mesh filename="meshes/{mesh}" scale="0.001 0.001 0.001"/></geometry>')
                lines.append('    </visual>')

                lines.append('    <collision>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(f'      <geometry><mesh filename="meshes/{mesh}" scale="0.001 0.001 0.001"/></geometry>')
                lines.append('    </collision>')

            lines.append('    <inertial>')
            lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
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

            lines.append(f'  <joint name="{name}" type="{jtype}">')
            lines.append(f'    <origin xyz="{ox} {oy} {oz}" rpy="{rr} {pp} {yy}"/>')
            lines.append(f'    <parent link="{parent}"/>')
            lines.append(f'    <child link="{child}"/>')
            lines.append(f'    <axis xyz="{ax} {ay} {az}"/>')
            lines.append('  </joint>')

        lines.append('</robot>')

        with open(urdf_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        self._log(f"[ACDC4Robot] URDF generado: {urdf_path}")

    # =========================================================
    # Logs
    # =========================================================
    def _log(self, msg: str):
        try:
            print(msg)
        except:
            pass
        # Si quieres popups:
        # if self.ui:
        #     self.ui.messageBox(msg, "ACDC4Robot", adsk.core.MessageBoxButtonTypes.OKButtonType)


def export_robot(robot_name: str, base_output_dir: str = None, *args, **kwargs):
    """Función helper llamada desde acdc4robot.py."""
    if robot_name is None and len(args) > 0:
        robot_name = args[0]
    if base_output_dir is None and len(args) > 1:
        base_output_dir = args[1]

    exporter = RobotExporter(robot_name, base_output_dir)
    exporter.export_all()
