import adsk.core
import adsk.fusion
import os
import math
import traceback
import struct
import zlib

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
        # Fusion internamente trabaja en cm, convertimos a m
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
# PNG helpers
# =========================================================

def _png_write_rgba(filepath: str, width: int, height: int, pixels_rgba: bytes):
    """
    Escribe un PNG RGBA simple (8 bits por canal) sin librerías externas.
    pixels_rgba: width * height * 4 bytes
    """
    raw = bytearray()
    stride = width * 4
    for y in range(height):
        raw.append(0)  # filter type 0
        start = y * stride
        raw.extend(pixels_rgba[start:start + stride])

    compressed = zlib.compress(bytes(raw), 9)

    def chunk(chunk_type: str, data: bytes) -> bytes:
        length = struct.pack(">I", len(data))
        ctype = chunk_type.encode("ascii")
        crc = zlib.crc32(ctype + data) & 0xffffffff
        crc_bytes = struct.pack(">I", crc)
        return length + ctype + data + crc_bytes

    ihdr_data = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)  # 8bit, RGBA
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk("IHDR", ihdr_data)
    png += chunk("IDAT", compressed)
    png += chunk("IEND", b"")

    with open(filepath, "wb") as f:
        f.write(png)


def _build_faces_atlas_png(png_path: str, face_colors: list, cell_px: int = 128):
    """
    face_colors: lista de (r,g,b,a) en [0..1] por cara.
    Genera un atlas en grid (celdas cell_px x cell_px) y devuelve:
      (width, height, uv_centers) donde
      uv_centers[i] = (u,v) centro del patch de la cara i.

    IMPORTANTE:
      - La imagen PNG se escribe fila 0 = arriba.
      - La mayoría de visores 3D interpretan v=0 = abajo.
      - Por eso invertimos V al calcular los UVs para que
        cada cara apunte al patch correcto (sin “colores cruzados”).
    """
    n = len(face_colors)
    if n <= 0:
        return None, None, []

    cols = max(1, int(math.ceil(math.sqrt(n))))
    rows = int(math.ceil(n / cols))

    width = cols * cell_px
    height = rows * cell_px

    pixels = bytearray(width * height * 4)

    # Pintamos cada celda
    for idx, col in enumerate(face_colors):
        if col is None:
            r = g = b = 0.7
            a = 1.0
        else:
            r, g, b, a = col

        ir = max(0, min(255, int(round(r * 255))))
        ig = max(0, min(255, int(round(g * 255))))
        ib = max(0, min(255, int(round(b * 255))))
        ia = max(0, min(255, int(round(a * 255))))

        c = idx % cols
        r_row = idx // cols

        x0 = c * cell_px
        y0 = r_row * cell_px

        for yy in range(y0, min(y0 + cell_px, height)):
            for xx in range(x0, min(x0 + cell_px, width)):
                off = (yy * width + xx) * 4
                pixels[off:off + 4] = bytes((ir, ig, ib, ia))

    _png_write_rgba(png_path, width, height, bytes(pixels))

    # UV centers con V invertida
    uv_centers = []
    for idx in range(n):
        c = idx % cols
        r_row = idx // cols

        # U tal cual (izquierda -> derecha)
        u0 = c / cols
        u1 = (c + 1) / cols
        u = 0.5 * (u0 + u1)

        # Coordenadas en espacio "imagen" (fila 0 = arriba)
        v0_img = r_row / rows
        v1_img = (r_row + 1) / rows
        v_img_center = 0.5 * (v0_img + v1_img)

        # Flip vertical a espacio UV (v=0 abajo, v=1 arriba)
        v = 1.0 - v_img_center

        uv_centers.append((u, v))

    return width, height, uv_centers


def _build_solid_png(png_path: str, color: tuple, size: int = 4):
    """
    Genera una textura pequeña sólida (size x size) con el color dado.
    color: (r,g,b,a) en [0..1]
    """
    if color is None:
        r = g = b = 0.7
        a = 1.0
    else:
        r, g, b, a = color

    ir = max(0, min(255, int(round(r * 255))))
    ig = max(0, min(255, int(round(g * 255))))
    ib = max(0, min(255, int(round(b * 255))))
    ia = max(0, min(255, int(round(a * 255))))

    pixels = bytes((ir, ig, ib, ia)) * (size * size)
    _png_write_rgba(png_path, size, size, pixels)


# =========================================================
# Recogida de cuerpos por occurrence
#  (idéntico sistema que tu exportador STL)
# =========================================================

def _collect_bodies_for_occurrence(occ: adsk.fusion.Occurrence, ui):
    comp = occ.component

    comp_brep_all = list(comp.bRepBodies)
    occ_brep_all = list(occ.bRepBodies)

    comp_brep_export = []
    occ_brep_export = []

    _log(ui, f"Analizando occurrence '{occ.name}'...")
    _log(ui, f"  [DEBUG] bRep(component) total={len(comp_brep_all)}, bRep(occurrence) total={len(occ_brep_all)}")

    # Primero bodies de la occurrence
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

    # Si no hay en occurrence, usamos los del componente
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

    # Primero meshBodies de la occurrence
    for m in occ_mesh_all:
        vis = getattr(m, "isVisible", None)
        export = True
        _log(ui, f"    [occ.mesh] body='{m.name}' visible={vis} -> export={export}")
        if export:
            occ_mesh_export.append(m)

    # Si no hay mesh en occurrence, usamos los del componente
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
    def __init__(self, robot_name: str, base_output_dir: str = None, mesh_quality: str = None):
        self.app, self.ui = _get_app_ui()
        if not self.app:
            raise RuntimeError("No se pudo obtener la aplicación de Fusion 360.")

        product = self.app.activeProduct
        self.design = adsk.fusion.Design.cast(product)
        if not self.design:
            raise RuntimeError("El producto activo no es un diseño de Fusion 360.")

        self.robot_name = _sanitize(robot_name) or "acdc_robot"

        # --- NUEVO: calidad de malla ---
        self.mesh_quality_label = mesh_quality if isinstance(mesh_quality, str) else "Medium quality"
        self.mesh_quality_mode = self._normalize_mesh_quality(self.mesh_quality_label)
        self._log(f"[ACDC4Robot] Mesh quality = '{self.mesh_quality_label}' → mode='{self.mesh_quality_mode}'")

        if base_output_dir is None:
            home = os.path.expanduser("~")
            base_output_dir = os.path.join(home, "Desktop")

        self.output_dir = os.path.join(base_output_dir, f"{self.robot_name}_urdf")
        # Carpeta "meshes" con .dae y .png
        self.meshes_dir = os.path.join(self.output_dir, "meshes")
        os.makedirs(self.meshes_dir, exist_ok=True)

        self.links = []          # [{name, mesh, occ, item, kind}]
        self.joints = []         # [{name,parent,child,type,origin_xyz,origin_rpy,axis,limit}]
        self.occ_to_link = {}    # occurrenceKey -> main link name
        self.base_link_name = None
        self.single_link_no_joints = False

    # ------------------------------
    # NUEVO: normalizar calidad
    # ------------------------------

    def _normalize_mesh_quality(self, label: str) -> str:
        """
        Convierte el texto del UI a un modo interno:
          very_low_optimized / very_low / low / medium / high
        Acepta:
          - "Very Low Quality Optimized"
          - "Very low quality"
          - "Low quality"
          - "Medium quality"
          - "Hight quality" (sic) / "High quality"
        """
        if not label:
            return "medium"
        q = label.strip().lower()

        # >>> VLO: modo ultra optimizado por velocidad
        if "very" in q and "low" in q and "optimized" in q:
            return "very_low_optimized"

        if "very" in q and "low" in q:
            return "very_low"
        if "low" in q:
            return "low"
        if "hight" in q or "high" in q:
            return "high"
        return "medium"

    # ------------------------------

    def _get_body_in_component_space(self, body):
        """
        Devuelve el cuerpo nativo del componente si 'body' es un proxy
        en contexto de ensamblaje (tiene nativeObject + assemblyContext).
        Esto evita doble transformación y mejora la precisión de malla.
        (NO cambia el sistema de poses, solo la base geométrica).
        """
        try:
            native = getattr(body, "nativeObject", None)
            ctx = getattr(body, "assemblyContext", None)
            if native and ctx:
                return native
        except:
            pass
        return body

    # ------------------------------

    def export_all(self):
        try:
            self._log(f"=== EXPORTANDO '{self.robot_name}' ===")
            self._build_links_and_joints()
            self._export_dae_meshes()
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
        """
        MISMO SISTEMA que tu exportador STL para obtener
        posición (xyz) y orientación (rpy) absolutas de la occurrence.
        """
        if not occ:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        try:
            if hasattr(occ, "transform2"):
                return _matrix_to_xyz_rpy(occ.transform2, self.design)
        except:
            pass
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    # ------------------------------
    # MISMO SISTEMA DE LINKS/Joints que el STL, pero con .dae
    # ------------------------------

    def _build_links_and_joints(self):
        root = self.design.rootComponent
        all_occs = list(root.allOccurrences)

        self._log(f"[DBG] _build_links_and_joints(): total occurrences = {len(all_occs)}")

        # 1) Links por cuerpos
        idx = 0
        for occ in all_occs:
            comp_brep, occ_brep, comp_mesh, occ_mesh = _collect_bodies_for_occurrence(occ, self.ui)

            # EXACTO: bodies = occ_brep primero, luego comp_brep
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
            # Aquí cambiamos solo la extensión a .dae
            main_name = _sanitize(f"link_{idx}_{occ.name}")
            main_mesh = f"{main_name}.dae"

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

            # Extras bodies de la misma occurrence/component -> fixed al main
            for i, extra in enumerate(exported_items[1:], start=1):
                extra_name = _sanitize(f"link_{idx}_{occ.name}_b{i}_{extra.name}")
                extra_mesh = f"{extra_name}.dae"

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
            mesh_name = f"{link_name}.dae"
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
    # (mismo sistema que STL)
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
    # DEBUG BREP + COLORES
    # -----------------------------------------------------

    def _log_brep_body_properties(self, body):
        body = self._get_body_in_component_space(body)

        try:
            cls = getattr(body, "classType", lambda: type(body))()
        except:
            cls = type(body)

        name = getattr(body, "name", "(sin nombre)")
        self._log(f"[DBG][BREP] body='{name}' classType={cls}")

        try:
            is_solid = getattr(body, "isSolid", None)
            is_visible = getattr(body, "isVisible", None)
            self._log(f"[DBG][BREP]   isSolid={is_solid}, isVisible={is_visible}")
        except:
            pass

        try:
            vol = getattr(body, "volume", None)
            area = getattr(body, "area", None)
            self._log(f"[DBG][BREP]   volume={vol}, area={area}")
        except:
            pass

        try:
            faces = getattr(body, "faces", None)
            edges = getattr(body, "edges", None)
            verts = getattr(body, "vertices", None)
            if faces:
                self._log(f"[DBG][BREP]   faces={faces.count}, edges={edges.count}, vertices={verts.count}")
            if faces and faces.count > 0:
                max_dump = min(3, faces.count)
                for i in range(max_dump):
                    f = faces.item(i)
                    f_name = getattr(f, "name", f"face_{i}")
                    f_cls = getattr(f, "classType", lambda: type(f))()
                    f_area = getattr(f, "area", None)
                    f_app = getattr(f, "appearance", None)
                    app_name = f_app.name if f_app else "(sin appearance)"
                    self._log(f"[DBG][BREP]   face[{i}] name='{f_name}' classType={f_cls}, area={f_area}, appearance={app_name}")
        except:
            pass

        try:
            app = getattr(body, "appearance", None)
            if app:
                self._log(f"[DBG][BREP]   body.appearance = '{app.name}'")
            mat = getattr(body, "material", None)
            if mat:
                self._log(f"[DBG][BREP]   body.material = '{mat.name}'")
        except:
            pass

    def _get_color_from_appearance(self, app):
        """
        Toma un Appearance y busca un ColorProperty.
        Devuelve (r,g,b,a) en [0..1] o None.
        """
        if not app:
            return None

        try:
            props = app.appearanceProperties
        except:
            return None

        # 1) propiedad "Color" explícita
        try:
            p = props.itemByName("Color")
            if p:
                colProp = adsk.core.ColorProperty.cast(p)
                if colProp and colProp.value:
                    c = colProp.value
                    r = c.red / 255.0
                    g = c.green / 255.0
                    b = c.blue / 255.0
                    a_raw = getattr(c, "opacity", 255)
                    a = a_raw / 255.0
                    self._log(f"[DBG][COLOR] '{app.name}' prop='Color' -> r={r:.3f}, g={g:.3f}, b={b:.3f}, a={a:.3f}")
                    return (r, g, b, a)
        except Exception as e:
            self._log(f"[DBG][COLOR] Error leyendo 'Color' en '{app.name}': {e}")

        # 2) cualquier ColorProperty
        try:
            for i in range(props.count):
                prop = props.item(i)
                colProp = adsk.core.ColorProperty.cast(prop)
                if colProp and colProp.value:
                    c = colProp.value
                    r = c.red / 255.0
                    g = c.green / 255.0
                    b = c.blue / 255.0
                    a_raw = getattr(c, "opacity", 255)
                    a = a_raw / 255.0
                    self._log(f"[DBG][COLOR] '{app.name}' prop='{prop.name}' -> r={r:.3f}, g={g:.3f}, b={b:.3f}, a={a:.3f}")
                    return (r, g, b, a)
        except Exception as e:
            self._log(f"[DBG][COLOR] Error recorriendo ColorProperty en '{app.name}': {e}")

        return None

    def _extract_color_from_body(self, body):
        """
        Color general del cuerpo (NO mira las caras).
        - body.appearance
        - body.material.appearance
        """
        if not body:
            return None

        body = self._get_body_in_component_space(body)

        # 1) body.appearance
        try:
            app = getattr(body, "appearance", None)
            if app:
                col = self._get_color_from_appearance(app)
                if col:
                    self._log(f"[DBG][COLOR] Color desde body.appearance '{app.name}' = {col}")
                    return col
        except:
            pass

        # 2) material.appearance
        try:
            mat = getattr(body, "material", None)
            if mat and getattr(mat, "appearance", None):
                app = mat.appearance
                col = self._get_color_from_appearance(app)
                if col:
                    self._log(f"[DBG][COLOR] Color desde body.material.appearance '{app.name}' = {col}")
                    return col
        except:
            pass

        return None

    def _extract_color_for_link(self, body_or_mesh, occ):
        """
        Fallback completo para un link (color "global" del cuerpo):
          1) body/mesh.appearance o material (sin mirar caras)
          2) occ.appearance
          3) occ.component.appearance
          4) occ.component.material.appearance
        """
        # 1) directo desde body/mesh
        col = self._extract_color_from_body(body_or_mesh)
        if col:
            return col

        # 2) appearance directo de la occurrence
        try:
            if occ and getattr(occ, "appearance", None):
                col = self._get_color_from_appearance(occ.appearance)
                if col:
                    self._log(f"[DBG][COLOR] Color desde occ.appearance '{occ.appearance.name}' = {col}")
                    return col
        except:
            pass

        # 3) appearance del componente
        try:
            if occ and getattr(occ, "component", None) and occ.component.appearance:
                col = self._get_color_from_appearance(occ.component.appearance)
                if col:
                    self._log(f"[DBG][COLOR] Color desde occ.component.appearance '{occ.component.appearance.name}' = {col}")
                    return col
        except:
            pass

        # 4) material del componente
        try:
            if occ and getattr(occ, "component", None):
                mat = getattr(occ.component, "material", None)
                if mat and getattr(mat, "appearance", None):
                    col = self._get_color_from_appearance(mat.appearance)
                    if col:
                        self._log(f"[DBG][COLOR] Color desde occ.component.material.appearance '{mat.appearance.name}' = {col}")
                        return col
        except:
            pass

        return None

    def _extract_color_for_face(self, face, body, occ):
        """
        Color para una cara específica:
          1) face.appearance
          2) color global del cuerpo / occ / componente
        """
        if face:
            try:
                app = getattr(face, "appearance", None)
                if app:
                    col = self._get_color_from_appearance(app)
                    if col:
                        self._log(f"[DBG][COLOR] Cara -> appearance '{app.name}' = {col}")
                        return col
            except:
                pass

        col = self._extract_color_for_link(body, occ)
        if col:
            return col

        return (0.7, 0.7, 0.7, 1.0)

    # -----------------------------------------------------
    # DAE export (BRep/Mesh -> Collada + PNG textura)
    # -----------------------------------------------------

    def _get_mesh_triangles_from_body_basic(self, body):
        """
        Versión básica: devuelve (vertices_m, indices) para un BRepBody o MeshBody.
        NO maneja UV ni textura, se usa como fallback.

        - Para BRep usamos TriangleMeshCalculator con calidad configurable
          (very_low_optimized / very_low / low / medium / high).
        - Siempre trabajamos en el cuerpo nativo del componente para evitar errores de posición.
        """
        body = self._get_body_in_component_space(body)

        is_brep = hasattr(body, "meshManager")
        is_mesh_body = hasattr(body, "mesh")

        coords = []
        indices = []

        try:
            if is_brep:
                self._log_brep_body_properties(body)

                mesh_mgr = body.meshManager
                mesh = None
                try:
                    calc = mesh_mgr.createMeshCalculator()

                    # --- NUEVO: calidad según self.mesh_quality_mode ---
                    try:
                        from adsk.fusion import TriangleMeshQualityOptions
                        mode = self.mesh_quality_mode

                        # intentamos varios nombres por modo, para ser robustos
                        candidates = []
                        if mode == "very_low_optimized":
                            # >>> VLO: lo más burdo posible
                            candidates = [
                                "VeryCoarseTriangleMesh",
                                "CoarseTriangleMesh",
                                "LowQualityTriangleMesh",
                            ]
                        elif mode == "very_low":
                            candidates = [
                                "VeryCoarseTriangleMesh",
                                "CoarseTriangleMesh",
                                "LowQualityTriangleMesh",
                            ]
                        elif mode == "low":
                            candidates = [
                                "LowQualityTriangleMesh",
                                "CoarseTriangleMesh",
                                "MediumQualityTriangleMesh",
                            ]
                        elif mode == "high":
                            candidates = [
                                "VeryFineTriangleMesh",
                                "FineTriangleMesh",
                                "HighQualityTriangleMesh",
                            ]
                        else:  # medium
                            candidates = ["MediumQualityTriangleMesh"]

                        chosen = None
                        for name in candidates:
                            if hasattr(TriangleMeshQualityOptions, name):
                                opt = getattr(TriangleMeshQualityOptions, name)
                                calc.setQuality(opt)
                                chosen = name
                                break

                        # fallback a Medium si nada anterior existe
                        if chosen is None and hasattr(TriangleMeshQualityOptions, "MediumQualityTriangleMesh"):
                            opt = getattr(TriangleMeshQualityOptions, "MediumQualityTriangleMesh")
                            calc.setQuality(opt)
                            chosen = "MediumQualityTriangleMesh"

                        self._log(f"[DBG][MESH] TriangleMeshCalculator.setQuality({chosen})")
                    except Exception:
                        # Fallback: ajustar tolerancia manualmente según calidad
                        try:
                            bbox = body.boundingBox
                            dx = bbox.maxPoint.x - bbox.minPoint.x
                            dy = bbox.maxPoint.y - bbox.minPoint.y
                            dz = bbox.maxPoint.z - bbox.maxPoint.z
                        except Exception:
                            bbox = None
                            dx = dy = dz = 1.0

                        diameter = math.sqrt(dx*dx + dy*dy + dz*dz) if bbox else 1.0

                        mode = self.mesh_quality_mode
                        # exponente: más grande ⇒ más fino
                        if mode == "very_low_optimized":
                            # >>> VLO: tolerancia muy gruesa
                            pow_ = 4
                        elif mode == "very_low":
                            pow_ = 6
                        elif mode == "low":
                            pow_ = 8
                        elif mode == "high":
                            pow_ = 12
                        else:  # medium
                            pow_ = 10

                        try:
                            calc.surfaceTolerance = diameter / (2.0 ** pow_)
                            self._log(f"[DBG][MESH] surfaceTolerance={calc.surfaceTolerance} (mode={mode})")
                        except Exception:
                            # fallback absoluto
                            if mode == "very_low_optimized":
                                calc.surfaceTolerance = 1.0
                            elif mode == "very_low":
                                calc.surfaceTolerance = 0.5
                            elif mode == "low":
                                calc.surfaceTolerance = 0.25
                            elif mode == "high":
                                calc.surfaceTolerance = 0.02
                            else:
                                calc.surfaceTolerance = 0.1
                            self._log(f"[DBG][MESH] surfaceTolerance fallback={calc.surfaceTolerance} (mode={mode})")

                    mesh = calc.calculate()
                except:
                    # fallback: displayMeshes
                    try:
                        if mesh_mgr.displayMeshes.count > 0:
                            mesh = mesh_mgr.displayMeshes.item(0)
                            self._log("[DBG][MESH] Usando displayMeshes como fallback.")
                    except:
                        pass

                if not mesh:
                    raise RuntimeError("No se pudo obtener el mesh del BRepBody.")

                try:
                    coords = list(mesh.nodeCoordinatesAsFloat)
                except:
                    coords = list(mesh.nodeCoordinates)

                indices = list(mesh.nodeIndices)

            elif is_mesh_body:
                mb = body
                self._log(f"[DBG][MESH] MeshBody name='{getattr(mb, 'name', '(sin nombre)')}'")

                mesh = None
                try:
                    mesh = mb.mesh
                except:
                    try:
                        mesh = mb.displayMesh
                    except:
                        pass

                if not mesh:
                    raise RuntimeError("No se pudo obtener el mesh de MeshBody.")

                try:
                    coords = list(mesh.nodeCoordinatesAsFloat)
                except:
                    coords = list(mesh.nodeCoordinates)

                indices = list(mesh.nodeIndices)

            else:
                raise RuntimeError("Tipo de body no soportado para extracción de malla.")

        except Exception as e:
            self._log(f"[ACDC4Robot] Error obteniendo malla: {e}")
            return [], []

        # cm -> m
        try:
            scale = self.design.unitsManager.convert(1.0, "cm", "m")
        except Exception:
            scale = 0.01

        verts_m = []
        for i in range(0, len(coords), 3):
            x = coords[i] * scale
            y = coords[i + 1] * scale
            z = coords[i + 2] * scale
            verts_m.extend([x, y, z])

        self._log(f"[DBG][MESH] vertices={len(verts_m)//3}, triangles={len(indices)//3}")
        return verts_m, indices

    def _build_brep_mesh_and_texture(self, body, occ, geom_id: str):
        """
        Malla cara por cara de un BRepBody:
          - genera atlas PNG con un patch por cara
          - cada cara tiene un UV (u,v) fijo (centro del patch)

        EXCEPCIÓN:
          - Si mesh_quality_mode == "very_low_optimized":
              * NO se triangula cara por cara
              * Se usa una única malla básica por cuerpo
              * Textura sólida 1 color por cuerpo
              * Muchísimo más rápido para ensamblajes grandes.
        Devuelve (vertices, indices, uvs, texture_filename)
        """
        body = self._get_body_in_component_space(body)

        # >>> VLO: camino ultra rápido, sacrificando colores por cara
        if self.mesh_quality_mode == "very_low_optimized":
            self._log(f"[ACDC4Robot] Very Low Quality Optimized: usando malla básica por cuerpo para '{getattr(body,'name','(sin nombre)')}'")
            verts, idxs = self._get_mesh_triangles_from_body_basic(body)
            if not verts or not idxs:
                return [], [], [], None

            tex_name = geom_id + ".png"
            tex_path = os.path.join(self.meshes_dir, tex_name)
            color = self._extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
            # textura mínima (2x2) para reducir aún más coste
            _build_solid_png(tex_path, color, size=2)

            nverts = len(verts) // 3
            uvs = [0.5, 0.5] * nverts
            return verts, idxs, uvs, tex_name

        # --- Camino normal (con atlas por cara) ---
        self._log(f"[ACDC4Robot] _build_brep_mesh_and_texture para '{getattr(body,'name','(sin nombre)')}'")

        faces = getattr(body, "faces", None)
        if not faces or faces.count == 0:
            self._log("[ACDC4Robot] BRepBody sin caras, fallback basic mesh.")
            verts, idxs = self._get_mesh_triangles_from_body_basic(body)
            if not verts or not idxs:
                return [], [], [], None

            tex_name = geom_id + ".png"
            tex_path = os.path.join(self.meshes_dir, tex_name)
            color = self._extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
            _build_solid_png(tex_path, color, size=4)

            nverts = len(verts) // 3
            uvs = [0.5, 0.5] * nverts
            return verts, idxs, uvs, tex_name

        # Colores por cara
        face_colors = []
        for i in range(faces.count):
            f = faces.item(i)
            col = self._extract_color_for_face(f, body, occ)
            face_colors.append(col)

        tex_name = geom_id + ".png"
        tex_path = os.path.join(self.meshes_dir, tex_name)

        # Atlas de mayor resolución
        w, h, uv_centers = _build_faces_atlas_png(tex_path, face_colors, cell_px=128)

        if not uv_centers:
            color = self._extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
            _build_solid_png(tex_path, color, size=4)
            verts, idxs = self._get_mesh_triangles_from_body_basic(body)
            nverts = len(verts) // 3
            uvs = [0.5, 0.5] * nverts
            return verts, idxs, uvs, tex_name

        verts_m = []
        indices = []
        uvs = []

        try:
            scale = self.design.unitsManager.convert(1.0, "cm", "m")
        except Exception:
            scale = 0.01

        faces_with_mesh = 0

        for i in range(faces.count):
            f = faces.item(i)
            uv_u, uv_v = uv_centers[i]

            try:
                mesh_mgr = f.meshManager
                calc = mesh_mgr.createMeshCalculator()

                # --- NUEVO: calidad según modo ---
                try:
                    from adsk.fusion import TriangleMeshQualityOptions
                    mode = self.mesh_quality_mode

                    candidates = []
                    if mode == "very_low":
                        candidates = [
                            "VeryCoarseTriangleMesh",
                            "CoarseTriangleMesh",
                            "LowQualityTriangleMesh",
                        ]
                    elif mode == "low":
                        candidates = [
                            "LowQualityTriangleMesh",
                            "CoarseTriangleMesh",
                            "MediumQualityTriangleMesh",
                        ]
                    elif mode == "high":
                        candidates = [
                            "VeryFineTriangleMesh",
                            "FineTriangleMesh",
                            "HighQualityTriangleMesh",
                        ]
                    else:  # medium
                        candidates = ["MediumQualityTriangleMesh"]

                    chosen = None
                    for name in candidates:
                        if hasattr(TriangleMeshQualityOptions, name):
                            opt = getattr(TriangleMeshQualityOptions, name)
                            calc.setQuality(opt)
                            chosen = name
                            break

                    if chosen is None and hasattr(TriangleMeshQualityOptions, "MediumQualityTriangleMesh"):
                        opt = getattr(TriangleMeshQualityOptions, "MediumQualityTriangleMesh")
                        calc.setQuality(opt)
                        chosen = "MediumQualityTriangleMesh"

                    self._log(f"[DBG][MESH_FACE] setQuality({chosen}) para cara {i}")
                except Exception:
                    try:
                        bbox = f.boundingBox
                        dx = bbox.maxPoint.x - bbox.minPoint.x
                        dy = bbox.maxPoint.y - bbox.minPoint.y
                        dz = bbox.maxPoint.z - bbox.maxPoint.z
                    except Exception:
                        bbox = None
                        dx = dy = dz = 1.0

                    diameter = math.sqrt(dx*dx + dy*dy + dz*dz) if bbox else 1.0

                    mode = self.mesh_quality_mode
                    if mode == "very_low":
                        pow_ = 6
                    elif mode == "low":
                        pow_ = 8
                    elif mode == "high":
                        pow_ = 12
                    else:
                        pow_ = 10

                    try:
                        calc.surfaceTolerance = diameter / (2.0 ** pow_)
                        self._log(f"[DBG][MESH_FACE] surfaceTolerance={calc.surfaceTolerance} (mode={mode}, cara={i})")
                    except Exception:
                        if mode == "very_low":
                            calc.surfaceTolerance = 0.5
                        elif mode == "low":
                            calc.surfaceTolerance = 0.25
                        elif mode == "high":
                            calc.surfaceTolerance = 0.02
                        else:
                            calc.surfaceTolerance = 0.1
                        self._log(f"[DBG][MESH_FACE] surfaceTolerance fallback={calc.surfaceTolerance} (mode={mode}, cara={i})")

                mesh = calc.calculate()
                if not mesh:
                    self._log(f"[ACDC4Robot] Cara {i}: mesh = None, se omite.")
                    continue
            except Exception as e:
                self._log(f"[ACDC4Robot] Error mesh en cara {i}: {e}")
                continue

            try:
                coords = list(mesh.nodeCoordinatesAsFloat)
            except:
                coords = list(mesh.nodeCoordinates)

            idxs_face = list(mesh.nodeIndices)

            if not coords or not idxs_face:
                self._log(f"[ACDC4Robot] Cara {i}: mesh vacío, se omite.")
                continue

            faces_with_mesh += 1
            base_index = len(verts_m) // 3

            for k in range(0, len(coords), 3):
                x = coords[k] * scale
                y = coords[k + 1] * scale
                z = coords[k + 2] * scale
                verts_m.extend([x, y, z])
                uvs.extend([uv_u, uv_v])

            for idx in idxs_face:
                indices.append(base_index + int(idx))

        if faces_with_mesh == 0 or not verts_m or not indices:
            self._log("[ACDC4Robot] WARNING: triangulación por caras falló, usando malla básica.")
            verts, idxs = self._get_mesh_triangles_from_body_basic(body)
            if not verts or not idxs:
                return [], [], [], None

            color = self._extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
            _build_solid_png(tex_path, color, size=4)
            nverts = len(verts) // 3
            uvs = [0.5, 0.5] * nverts
            return verts, idxs, uvs, tex_name

        self._log(f"[ACDC4Robot] BRepBody triangulado cara por cara: verts={len(verts_m)//3}, tris={len(indices)//3}")
        return verts_m, indices, uvs, tex_name

    def _build_meshbody_mesh_and_texture(self, body, occ, geom_id: str):
        """
        MeshBody completo con una textura sólida (1 color) por .dae.
        """
        body = self._get_body_in_component_space(body)

        verts, idxs = self._get_mesh_triangles_from_body_basic(body)
        if not verts or not idxs:
            return [], [], [], None

        tex_name = geom_id + ".png"
        tex_path = os.path.join(self.meshes_dir, tex_name)

        color = self._extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
        _build_solid_png(tex_path, color, size=4)

        nverts = len(verts) // 3
        uvs = [0.5, 0.5] * nverts

        return verts, idxs, uvs, tex_name

    def _write_dae_with_texture(self, filepath: str, geom_id: str,
                                vertices: list, indices: list,
                                uvs: list, texture_filename: str):
        """
        Escribe un Collada 1.4.1 con:
          - geometry con POSITION + TEXCOORD
          - material con textura PNG
        """
        if not vertices or not indices or not uvs:
            self._log(f"[ACDC4Robot] _write_dae_with_texture(): datos vacíos, se omite {filepath}")
            return

        pos_id = f"{geom_id}-positions"
        pos_array_id = f"{pos_id}-array"
        tex_id = f"{geom_id}-texcoords"
        tex_array_id = f"{tex_id}-array"

        verts_count = len(vertices)
        tris_count = len(indices) // 3
        vert_count = verts_count // 3
        uv_count = len(uvs) // 2

        if uv_count != vert_count:
            self._log(f"[ACDC4Robot] WARNING: uv_count({uv_count}) != vert_count({vert_count}), ajustando...")
            if uv_count < vert_count:
                if uv_count == 0:
                    uvs = [0.5, 0.5] * vert_count
                else:
                    last_u, last_v = uvs[-2], uvs[-1]
                    missing = vert_count - uv_count
                    uvs.extend([last_u, last_v] * missing)
            else:
                uvs = uvs[:vert_count * 2]

        verts_str = " ".join(f"{v:.9g}" for v in vertices)
        indices_str = " ".join(str(int(i)) for i in indices)
        uvs_str = " ".join(f"{c:.6f}" for c in uvs)

        dae = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor>
      <authoring_tool>ACDC4Robot DAE Exporter</authoring_tool>
    </contributor>
    <unit name="meter" meter="1.0"/>
    <up_axis>Z_UP</up_axis>
  </asset>

  <library_images>
    <image id="{geom_id}-image" name="{geom_id}-image">
      <init_from>{texture_filename}</init_from>
    </image>
  </library_images>

  <library_effects>
    <effect id="mat-effect">
      <profile_COMMON>
        <newparam sid="surface0">
          <surface type="2D">
            <init_from>{geom_id}-image</init_from>
          </surface>
        </newparam>
        <newparam sid="sampler0">
          <sampler2D>
            <source>surface0</source>
          </sampler2D>
        </newparam>
        <technique sid="common">
          <phong>
            <diffuse>
              <texture texture="sampler0" texcoord="TEX0"/>
            </diffuse>
          </phong>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>

  <library_materials>
    <material id="mat" name="mat">
      <instance_effect url="#mat-effect"/>
    </material>
  </library_materials>

  <library_geometries>
    <geometry id="{geom_id}" name="{geom_id}">
      <mesh>
        <source id="{pos_id}">
          <float_array id="{pos_array_id}" count="{verts_count}">{verts_str}</float_array>
          <technique_common>
            <accessor source="#{pos_array_id}" count="{vert_count}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>

        <source id="{tex_id}">
          <float_array id="{tex_array_id}" count="{2*vert_count}">{uvs_str}</float_array>
          <technique_common>
            <accessor source="#{tex_array_id}" count="{vert_count}" stride="2">
              <param name="S" type="float"/>
              <param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>

        <vertices id="{geom_id}-vertices">
          <input semantic="POSITION" source="#{pos_id}"/>
        </vertices>

        <triangles count="{tris_count}" material="mat">
          <input semantic="VERTEX" source="#{geom_id}-vertices" offset="0"/>
          <input semantic="TEXCOORD" source="#{tex_id}" offset="0" set="0"/>
          <p>{indices_str}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>

  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="{geom_id}-node" name="{geom_id}">
        <instance_geometry url="#{geom_id}">
          <bind_material>
            <technique_common>
              <instance_material symbol="mat" target="#mat">
                <bind_vertex_input semantic="TEX0" input_semantic="TEXCOORD" input_set="0"/>
              </instance_material>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>

  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
"""
        try:
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(dae)
            self._log(f"[ACDC4Robot] DAE generado: {filepath} (tex='{texture_filename}')")
        except Exception as e:
            self._log(f"[ACDC4Robot] Error escribiendo DAE '{filepath}': {e}")

    def _export_dae_meshes(self):
        """
        - Para BRepBody: atlas de textura PNG por cara
          (salvo en modo Very Low Quality Optimized, donde se usa malla básica).
        - Para MeshBody: textura sólida por .dae.
        En todos los casos: 1 .png por .dae en la misma carpeta.

        Sistema de links/joints y poses = el mismo que el STL, solo cambia el formato.
        """
        for link in self.links:
            mesh_name = link.get("mesh")
            if not mesh_name:
                continue

            item = link.get("item")
            occ = link.get("occ")

            export_target = item if item is not None else occ
            if export_target is None:
                continue

            dae_path = os.path.join(self.meshes_dir, mesh_name)
            geom_id = os.path.splitext(mesh_name)[0]

            try:
                item_native = self._get_body_in_component_space(item) if item is not None else None

                is_brep = hasattr(item_native, "faces") and hasattr(item_native, "meshManager")
                is_mesh_body = hasattr(item_native, "mesh") or hasattr(item_native, "displayMesh")

                if is_brep:
                    verts, idxs, uvs, tex_name = self._build_brep_mesh_and_texture(item_native, occ, geom_id)
                elif is_mesh_body:
                    verts, idxs, uvs, tex_name = self._build_meshbody_mesh_and_texture(item_native, occ, geom_id)
                else:
                    verts, idxs = self._get_mesh_triangles_from_body_basic(export_target)
                    if not verts or not idxs:
                        self._log(f"[ACDC4Robot] Sin malla para {link.get('name','?')}, DAE omitido.")
                        continue
                    tex_name = geom_id + ".png"
                    tex_path = os.path.join(self.meshes_dir, tex_name)
                    color = self._extract_color_for_link(item_native, occ) or (0.7, 0.7, 0.7, 1.0)
                    _build_solid_png(tex_path, color, size=4)
                    nverts = len(verts) // 3
                    uvs = [0.5, 0.5] * nverts

                if not verts or not idxs or not uvs or not tex_name:
                    self._log(f"[ACDC4Robot] Datos insuficientes para link '{link.get('name','?')}', DAE omitido.")
                    continue

                self._write_dae_with_texture(dae_path, geom_id, verts, idxs, uvs, tex_name)
            except Exception:
                self._log(
                    f"[ACDC4Robot] Error exportando DAE para {link.get('name', '?')}:\n"
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
        os.makedirs(self.output_dir, exist_ok=True)
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
                    f'scale="1 1 1"/></geometry>'
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
                    f'scale="1 1 1"/></geometry>'
                )
                lines.append('    </visual>')
                lines.append('    <collision>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(
                    f'      <geometry><mesh filename="meshes/{mesh}" '
                    f'scale="1 1 1"/></geometry>'
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

def export_robot(
    robot_name: str = "acdc_robot",
    base_output_dir: str = None,
    mesh_quality: str = None,
    *args,
    **kwargs
):
    """
    export_robot(robot_name, base_output_dir, mesh_quality)

    - mesh_quality: string como
        "Very Low Quality Optimized"  -> ultra rápido, 1 color por cuerpo
        "Very low quality"
        "Low quality"
        "Medium quality"
        "Hight quality" / "High quality"
      Si es None → medium.
    """
    # Compatibilidad con llamadas antiguas muy raras
    if robot_name is None and args:
        robot_name = args[0]
    if base_output_dir is None and len(args) > 1:
        base_output_dir = args[1]
    if mesh_quality is None and len(args) > 2:
        mesh_quality = args[2]
    if mesh_quality is None and "mesh_quality" in kwargs:
        mesh_quality = kwargs["mesh_quality"]

    exporter = RobotExporter(robot_name, base_output_dir, mesh_quality)
    exporter.export_all()
