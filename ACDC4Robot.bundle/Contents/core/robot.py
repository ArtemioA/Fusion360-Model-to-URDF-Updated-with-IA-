# -*- coding: utf-8 -*-
"""
Core export logic for ACDC4Robot.

Edita solo este archivo (Contents/core/robot.py).
"""

import adsk.core
import adsk.fusion
import os
import traceback

from .link import Link
from .joint import Joint
from . import urdf as urdf_utils


def _get_app_ui():
    app = adsk.core.Application.get()
    ui = app.userInterface if app else None
    return app, ui


def _sanitize(name: str) -> str:
    """Limpia nombres para archivos/links."""
    if not name:
        return "link"
    bad = '<>:"/\\|?* '
    for c in bad:
        name = name.replace(c, "_")
    return name


class RobotExporter:
    """
    Exportador robusto:

    - Crea un base_link virtual.
    - Genera links para:
        * Cada body sólido suelto en rootComponent.bRepBodies ("Cuerpos").
        * Cada Occurrence con sólidos.
        * Sub-ensamblajes (icono azul) con sólidos en descendientes.
    - Cada link basado en Occurrence:
        * usa Link + urdf_utils → pose, masas, etc correctas.
    - Joints:
        * reales: desde Joint + urdf_utils.
        * faltantes: fixed base_link→link con origin según la pose real del Link.
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
        self.base_link_name = "base_link"

        # Por defecto: Desktop
        if base_output_dir is None:
            home = os.path.expanduser("~")
            base_output_dir = os.path.join(home, "Desktop")

        # Carpeta final: <base>/<robot_name>_urdf/
        self.output_dir = os.path.join(base_output_dir, f"{self.robot_name}_urdf")
        self.meshes_dir = os.path.join(self.output_dir, "meshes")
        os.makedirs(self.meshes_dir, exist_ok=True)

        # Colecciones internas
        self.body_links = []          # [{name, mesh, body}]
        self.link_objs = []           # [Link(...)] para occurrences
        self.joint_objs = []          # [Joint(...)] joints reales
        self.extra_fixed_joints = []  # joints fixed sintéticos (usa pose real)

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
            err = traceback.format_exc()
            self._log("[ACDC4Robot] Error en RobotExporter.export_all():\n" + err)
            if self.ui:
                try:
                    self.ui.messageBox(
                        "Error en RobotExporter.export_all():\n\n" + err,
                        "ACDC4Robot"
                    )
                    self.ui.terminateActiveCommand()
                except:
                    pass
            # No relanzamos: evitamos matar el comando permanentemente.

    # =========================================================
    # Construcción de links/joints desde el diseño
    # =========================================================
    def _build_links_and_joints(self):
        root = self.design.rootComponent

        self.body_links = []
        self.link_objs = []
        self.joint_objs = []
        self.extra_fixed_joints = []

        idx = 0

        # ---------- 1) Cuerpos sueltos en el root ("Cuerpos") ----------
        for body in getattr(root, "bRepBodies", []):
            try:
                if not body or not body.isSolid:
                    continue
            except:
                continue

            name = _sanitize(f"body_{idx}_{body.name}")
            mesh = f"{name}.stl"

            self.body_links.append({
                "name": name,
                "mesh": mesh,
                "body": body,
            })

            # joint fijo simple a base_link (sin pose avanzada)
            self.extra_fixed_joints.append({
                "name": f"joint_{name}",
                "parent": self.base_link_name,
                "child": name,
                "origin_xyz": (0.0, 0.0, 0.0),
                "origin_rpy": (0.0, 0.0, 0.0),
                "axis": (0.0, 0.0, 1.0),
            })

            idx += 1

        # ---------- 2) Links desde todas las Occurrences ----------
        root_occs = getattr(root, "allOccurrences", None)
        seen_keys = set()

        if root_occs:
            for occ in root_occs:
                comp = getattr(occ, "component", None)
                if not comp:
                    continue

                # ¿Tiene cuerpos sólidos directos?
                has_direct = False
                for b in getattr(comp, "bRepBodies", []):
                    try:
                        if b and b.isSolid:
                            has_direct = True
                            break
                    except:
                        continue

                # ¿O tiene sólidos en descendientes? (sub-ensamblajes icono azul)
                has_desc = False
                if not has_direct and self._occ_has_descendant_solid(occ):
                    has_desc = True

                if not (has_direct or has_desc):
                    continue

                occ_key = getattr(occ, "fullPathName", None) or occ.name
                if occ_key in seen_keys:
                    continue

                try:
                    link_obj = Link(occ)
                    link_name = link_obj.get_name()
                except Exception:
                    self._log(f"[ACDC4Robot] Link() falló para occurrence '{occ.name}'")
                    self._log(traceback.format_exc())
                    continue

                if not link_name:
                    continue

                # Asegurar nombre único
                base_name = link_name
                suffix = 1
                while self._link_name_exists(link_name):
                    link_name = f"{base_name}_{suffix}"
                    suffix += 1
                link_obj.name = link_name

                self.link_objs.append(link_obj)
                seen_keys.add(occ_key)

        # ---------- 3) Joints reales desde Fusion ----------
        all_joints = getattr(root, "allJoints", None)
        if all_joints:
            for j in all_joints:
                try:
                    j_obj = Joint(j)
                    parent = j_obj.get_parent()
                    child = j_obj.get_child()
                except Exception:
                    self._log("[ACDC4Robot] Joint() falló para un joint de Fusion")
                    self._log(traceback.format_exc())
                    continue

                # Solo usamos joints donde ambos links existen (por nombre)
                if self._name_is_known_link(parent) and self._name_is_known_link(child):
                    self.joint_objs.append(j_obj)

        # ---------- 4) Joints fijos para links sin joint ----------
        linked_children = set()
        for j in self.joint_objs:
            try:
                linked_children.add(j.get_child())
            except:
                continue

        for link_obj in self.link_objs:
            name = link_obj.get_name()
            if name in linked_children:
                continue

            # Aqui está el FIX:
            # usamos la pose real del Link (w.r.t root) para el joint fixed.
            try:
                pose = link_obj.get_pose_sdf()  # [x, y, z, roll, pitch, yaw]
            except Exception:
                pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.extra_fixed_joints.append({
                "name": f"joint_{name}",
                "parent": self.base_link_name,
                "child": name,
                "origin_xyz": (pose[0], pose[1], pose[2]),
                "origin_rpy": (pose[3], pose[4], pose[5]),
                "axis": (0.0, 0.0, 1.0),
            })

        # ---------- 5) Fallback si no hay nada ----------
        if not self.link_objs and not self.body_links:
            # Exportar root como único link
            self.body_links.append({
                "name": self.base_link_name,
                "mesh": f"{self.base_link_name}.stl",
                "body": root,
            })

    def _occ_has_descendant_solid(self, occ) -> bool:
        """True si la occurrence o alguno de sus hijos tiene al menos un body sólido."""
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

    def _link_name_exists(self, name: str) -> bool:
        if name == self.base_link_name:
            return True
        for bl in self.body_links:
            if bl["name"] == name:
                return True
        for lo in self.link_objs:
            if lo.get_name() == name:
                return True
        return False

    def _name_is_known_link(self, name: str) -> bool:
        if name == self.base_link_name:
            return True
        for lo in self.link_objs:
            if lo.get_name() == name:
                return True
        # body_links no se usan en joints reales
        return False

    # =========================================================
    # Export de mallas STL
    # =========================================================
    def _export_meshes(self):
        export_mgr = self.design.exportManager

        # 1) bodies sueltos
        for bl in self.body_links:
            mesh = bl["mesh"]
            body = bl["body"]
            if not mesh or body is None:
                continue

            stl_path = os.path.join(self.meshes_dir, mesh)
            try:
                opts = export_mgr.createSTLExportOptions(body, stl_path)
                self._tune_stl_options(opts)
                if export_mgr.execute(opts):
                    self._log(f"[ACDC4Robot] STL generado (body): {stl_path}")
            except Exception:
                self._log(f"[ACDC4Robot] Error exportando STL para body-link {bl['name']}")
                self._log(traceback.format_exc())

        # 2) occurrences (incluye sub-ensamblajes icono azul)
        for link_obj in self.link_objs:
            name = link_obj.get_name()
            mesh = f"{name}.stl"
            occ = link_obj.get_link_occ()
            if not occ:
                continue

            stl_path = os.path.join(self.meshes_dir, mesh)
            try:
                opts = export_mgr.createSTLExportOptions(occ, stl_path)
                self._tune_stl_options(opts)
                if export_mgr.execute(opts):
                    self._log(f"[ACDC4Robot] STL generado (occ): {stl_path}")
            except Exception:
                self._log(f"[ACDC4Robot] Error exportando STL para link {name}")
                self._log(traceback.format_exc())

    def _tune_stl_options(self, stl_opts):
        """Configura opciones STL con compatibilidad hacia atrás."""
        try:
            stl_opts.meshRefinement = adsk.fusion.MeshRefinementOptions.High
        except AttributeError:
            try:
                stl_opts.meshRefinement = (
                    adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                )
            except AttributeError:
                pass

        if hasattr(stl_opts, "isBinaryFormat"):
            stl_opts.isBinaryFormat = True

    # =========================================================
    # Generación del archivo URDF
    # =========================================================
    def _write_urdf(self):
        from xml.etree.ElementTree import Element, ElementTree, SubElement

        urdf_path = os.path.join(self.output_dir, f"{self.robot_name}.urdf")

        robot_ele = Element("robot")
        robot_ele.attrib = {"name": self.robot_name}

        # --- base_link ---
        base_ele = Element("link")
        base_ele.attrib = {"name": self.base_link_name}
        inertial = SubElement(base_ele, "inertial")
        origin_i = SubElement(inertial, "origin")
        origin_i.attrib = {"xyz": "0 0 0", "rpy": "0 0 0"}
        mass = SubElement(inertial, "mass")
        mass.attrib = {"value": "1.0"}
        inertia = SubElement(inertial, "inertia")
        inertia.attrib = {
            "ixx": "1", "ixy": "0", "ixz": "0",
            "iyy": "1", "iyz": "0", "izz": "1",
        }
        robot_ele.append(base_ele)

        # --- links de bodies sueltos ---
        for bl in self.body_links:
            name = bl["name"]
            mesh = bl["mesh"]

            link_ele = Element("link")
            link_ele.attrib = {"name": name}

            visual = SubElement(link_ele, "visual")
            origin_v = SubElement(visual, "origin")
            origin_v.attrib = {"xyz": "0 0 0", "rpy": "0 0 0"}
            geometry_v = SubElement(visual, "geometry")
            mesh_v = SubElement(geometry_v, "mesh")
            mesh_v.attrib = {
                "filename": f"meshes/{mesh}",
                "scale": "0.001 0.001 0.001",
            }

            collision = SubElement(link_ele, "collision")
            origin_c = SubElement(collision, "origin")
            origin_c.attrib = {"xyz": "0 0 0", "rpy": "0 0 0"}
            geometry_c = SubElement(collision, "geometry")
            mesh_c = SubElement(geometry_c, "mesh")
            mesh_c.attrib = {
                "filename": f"meshes/{mesh}",
                "scale": "0.001 0.001 0.001",
            }

            inertial = SubElement(link_ele, "inertial")
            origin_i = SubElement(inertial, "origin")
            origin_i.attrib = {"xyz": "0 0 0", "rpy": "0 0 0"}
            mass = SubElement(inertial, "mass")
            mass.attrib = {"value": "1.0"}
            inertia = SubElement(inertial, "inertia")
            inertia.attrib = {
                "ixx": "1", "ixy": "0", "ixz": "0",
                "iyy": "1", "iyz": "0", "izz": "1",
            }

            robot_ele.append(link_ele)

        # --- links de occurrences usando utilidades URDF ---
        for link_obj in self.link_objs:
            try:
                link_ele = urdf_utils.get_link_element(link_obj)
                robot_ele.append(link_ele)
            except Exception:
                self._log(
                    f"[ACDC4Robot] Error generando link URDF para {link_obj.get_name()}"
                )
                self._log(traceback.format_exc())

        # --- joints reales ---
        for j_obj in self.joint_objs:
            try:
                joint_ele = urdf_utils.get_joint_element(j_obj)
                robot_ele.append(joint_ele)
            except Exception:
                self._log("[ACDC4Robot] Error generando joint URDF real")
                self._log(traceback.format_exc())

        # --- joints fijos sintéticos (usa pose real si existe) ---
        for j in self.extra_fixed_joints:
            joint_ele = Element("joint")
            joint_ele.attrib = {
                "name": j["name"],
                "type": "fixed",
            }

            ox, oy, oz = j["origin_xyz"]
            rr, rp, ry = j["origin_rpy"]

            origin = SubElement(joint_ele, "origin")
            origin.attrib = {
                "xyz": f"{ox} {oy} {oz}",
                "rpy": f"{rr} {rp} {ry}",
            }

            parent = SubElement(joint_ele, "parent")
            parent.attrib = {"link": j["parent"]}

            child = SubElement(joint_ele, "child")
            child.attrib = {"link": j["child"]}

            ax, ay, az = j["axis"]
            axis = SubElement(joint_ele, "axis")
            axis.attrib = {"xyz": f"{ax} {ay} {az}"}

            robot_ele.append(joint_ele)

        # Guardar
        tree = ElementTree(robot_ele)
        tree.write(urdf_path, encoding="utf-8", xml_declaration=False)

        self._log(f"[ACDC4Robot] URDF generado: {urdf_path}")

    # =========================================================
    # Logs
    # =========================================================
    def _log(self, msg: str):
        try:
            print(msg)
        except Exception:
            pass
        # Si quieres popups en Fusion, descomenta:
        # if self.ui:
        #     self.ui.messageBox(msg, "ACDC4Robot",
        #                        adsk.core.MessageBoxButtonTypes.OKButtonType)


# =========================================================
# Punto de entrada llamado desde acdc4robot.py
# =========================================================
def export_robot(robot_name: str, base_output_dir: str = None, *args, **kwargs):
    """
    Firma compatible con:
        export_robot(name, base_output_dir, use_joints=..., ...)
    Args extra se ignoran sin romper.
    """
    if robot_name is None and len(args) > 0:
        robot_name = args[0]
    if base_output_dir is None and len(args) > 1:
        base_output_dir = args[1]

    exporter = RobotExporter(robot_name, base_output_dir)
    exporter.export_all()
