import adsk.core
import adsk.fusion
import os
import math
import traceback

# =========================================================
# LOGGING UTIL
# =========================================================

def _get_app_ui():
    app = adsk.core.Application.get()
    ui = app.userInterface if app else None
    return app, ui

def _log(ui, msg):
    try:
        print("[URDF-DEBUG] " + msg)
        if ui:
            palette = ui.palettes.itemById("TextCommands")
            if palette:
                palette.writeText("[URDF-DEBUG] " + msg)
    except:
        pass

# =========================================================
# MATRIX HELPERS
# =========================================================

def _sanitize(name: str) -> str:
    if not name:
        return "link"
    bad = '<>:"/\\|?* ,.'
    for c in bad:
        name = name.replace(c, '_')
    if name[0].isdigit():
        name = "l_" + name
    return name

def _identity_matrix():
    m = adsk.core.Matrix3D.create()
    m.setToIdentity()
    return m

def _invert_matrix(m: adsk.core.Matrix3D):
    if not m:
        inv = adsk.core.Matrix3D.create()
        inv.setToIdentity()
        return inv
    inv = adsk.core.Matrix3D.create()
    try:
        inv.copy(m)
        inv.invert()
    except:
        try:
            inv.setToIdentity()
            inv.transformBy(m)
            inv.invert()
        except:
            inv.setToIdentity()
    return inv

def _matrix_det(m):
    a11,a12,a13 = m.getCell(0,0),m.getCell(0,1),m.getCell(0,2)
    a21,a22,a23 = m.getCell(1,0),m.getCell(1,1),m.getCell(1,2)
    a31,a32,a33 = m.getCell(2,0),m.getCell(2,1),m.getCell(2,2)
    return (
        a11*(a22*a33 - a23*a32)
        - a12*(a21*a33 - a23*a31)
        + a13*(a21*a32 - a22*a31)
    )

def _matrix_to_xyz_rpy(m: adsk.core.Matrix3D, design):
    if m is None:
        return (0,0,0), (0,0,0)

    p = m.translation
    try:
        scale = design.unitsManager.convert(1.0, design.unitsManager.defaultLengthUnits, "m")
    except:
        scale = 0.01

    x, y, z = p.x*scale, p.y*scale, p.z*scale

    r11,r12,r13 = m.getCell(0,0),m.getCell(0,1),m.getCell(0,2)
    r21,r22,r23 = m.getCell(1,0),m.getCell(1,1),m.getCell(1,2)
    r31,r32,r33 = m.getCell(2,0),m.getCell(2,1),m.getCell(2,2)

    if abs(r31) < 1.0:
        pitch = math.asin(-r31)
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)
    else:
        pitch = math.pi/2 if r31 <= -1 else -math.pi/2
        roll = 0
        yaw = math.atan2(-r12, r22)

    return (x,y,z), (roll,pitch,yaw)

def _matrix_to_str(m, design):
    xyz, rpy = _matrix_to_xyz_rpy(m, design)
    det = _matrix_det(m)
    return (
        f"XYZ=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})  "
        f"RPY=({rpy[0]:.3f},{rpy[1]:.3f},{rpy[2]:.3f})  "
        f"DET={det:.5f}"
    )

# =========================================================
# GLOBAL TRANSFORM (CORREGIDO) 
# =========================================================

def _compute_global_transform(occ, ui, design):
    """
    Devuelve la transformación GLOBAL del occurrence.

    NOTA IMPORTANTE:
    - En Fusion, para cada occurrence "proxy", la propiedad transform/transform2
      ya está dada respecto al rootComponent (global), usando assemblyContext 
      internamente.
    - Si intentamos multiplicar transform * assemblyContext.transform otra vez,
      desordenamos orientación y posición (doble aplicación).
    """
    if not occ:
        return _identity_matrix()

    try:
        m = occ.transform2 if hasattr(occ, "transform2") else occ.transform
    except:
        m = None

    if not m:
        m = _identity_matrix()

    _log(ui, f"[GLOBAL] {occ.name}: {_matrix_to_str(m, design)}")
    return m

# =========================================================
# EXPORTADOR PRINCIPAL
# =========================================================

class RobotExporter:
    def __init__(self, robot_name, base_output_dir=None):
        self.app, self.ui = _get_app_ui()

        product = self.app.activeProduct
        self.design = adsk.fusion.Design.cast(product)

        self.robot_name = _sanitize(robot_name)
        if base_output_dir is None:
            base_output_dir = os.path.join(os.path.expanduser("~"), "Desktop")

        self.output_dir = os.path.join(base_output_dir, self.robot_name + "_urdf")
        self.meshes_dir = os.path.join(self.output_dir, "meshes")
        os.makedirs(self.meshes_dir, exist_ok=True)

        # DATA STRUCTURES
        self.links = []             # [{name, meshes[], occ}]
        self.joints = []            # [{...}]
        self.occ_all = {}           # key -> occ
        self.occ_parent = {}        # key -> parent key
        self.occ_abs_mats = {}      # key -> GLOBAL transform
        self.occ_to_link = {}       # occ_key -> link_name
        self.link_to_occ = {}       # link_name -> occ

        self.base_link_name = None
        self.single_link_no_joints = False

    # -----------------------------------------------------
    # API principal
    # -----------------------------------------------------
    def export_all(self):
        try:
            _log(self.ui, f"=== EXPORTANDO {self.robot_name} ===")
            self._build_links_and_joints()
            self._export_meshes()
            self._write_urdf()
            _log(self.ui, f"✔ Exportación completa: {self.output_dir}")
        except Exception:
            msg = "ERROR en export_all():\n" + traceback.format_exc()
            _log(self.ui, msg,)
            raise

    # -----------------------------------------------------
    # OCCURRENCE KEY
    # -----------------------------------------------------
    def _occ_key(self, occ):
        if occ is None:
            return None
        try:
            tok = occ.entityToken
            if tok:
                return tok
        except:
            pass
        try:
            fp = occ.fullPathName
            if fp:
                return fp
        except:
            pass
        return f"{occ.name}_{id(occ)}"

    # -----------------------------------------------------
    # LISTAR CUERPOS DE UN OCCURRENCE
    # -----------------------------------------------------
    def _get_solid_bodies(self, occ):
        lst = []
        try:
            comp = occ.component
            if comp and comp.bRepBodies:
                for b in comp.bRepBodies:
                    if b.isSolid:
                        lst.append(b)
        except:
            pass
        return lst

    # -----------------------------------------------------
    # RECURSIVO PARA ENCONTRAR LINKS Y MATRICES
    # (USANDO GLOBAL DIRECTO)
    # -----------------------------------------------------
    def _collect_recursive(self, occs):
        for occ in occs:
            key = self._occ_key(occ)
            self.occ_all[key] = occ

            # padre lógico (solo jerarquía)
            ctx = None
            try:
                ctx = occ.assemblyContext
            except:
                ctx = None

            if ctx:
                parent_key = self._occ_key(ctx)
                if parent_key:
                    self.occ_parent[key] = parent_key

            # CALCULAR GLOBAL CORRECTO
            T_global = _compute_global_transform(occ, self.ui, self.design)
            self.occ_abs_mats[key] = T_global

            _log(self.ui, f"[ABS] {occ.name} = { _matrix_to_str(T_global, self.design) }")

            # Crear link si tiene cuerpos
            bodies = self._get_solid_bodies(occ)
            if bodies:
                # Nombre depende de la ruta completa (maneja folders/occurrences anidados)
                link_name = _sanitize(occ.fullPathName.replace(":", "_"))
                meshes = []
                for i,b in enumerate(bodies):
                    meshes.append({"filename": f"{link_name}_b{i}.stl", "body": b})

                self.links.append({
                    "name": link_name,
                    "meshes": meshes,
                    "occ": occ
                })

                self.occ_to_link[key] = link_name
                self.link_to_occ[link_name] = occ

                _log(self.ui, f"[LINK] Creado link '{link_name}' desde occ {occ.name}")

            # Recursión (esto baja por carpetas / subcomponentes / occurrences anidados)
            try:
                child = occ.childOccurrences
                if child and child.count > 0:
                    self._collect_recursive(child)
            except:
                pass

    # -----------------------------------------------------
    # CREAR WRAPPER LINKS PARA ASSEMBLY SIN CUERPOS
    # -----------------------------------------------------
    def _create_wrapper_links(self):
        for key, occ in list(self.occ_all.items()):
            if key in self.occ_to_link:
                continue

            # revisar si tiene hijos con link
            has_child_link = False
            try:
                for c in occ.childOccurrences:
                    if self._occ_key(c) in self.occ_to_link:
                        has_child_link = True
                        break
            except:
                pass

            if not has_child_link:
                continue

            # crear frame
            base = occ.fullPathName.replace(":", "_")
            ln = _sanitize(base + "_frame")

            # evitar repetidos
            existing = {l["name"] for l in self.links}
            orig = ln
            i = 1
            while ln in existing:
                ln = f"{orig}_{i}"
                i += 1

            self.links.append({
                "name": ln,
                "meshes": [],
                "occ": occ
            })
            self.occ_to_link[key] = ln
            self.link_to_occ[ln] = occ

            _log(self.ui, f"[WRAPPER] Creado wrapper link '{ln}' para {occ.name}")

    # -----------------------------------------------------
    # OBTENER JOINTS DE FUSION
    # -----------------------------------------------------
    def _get_fusion_joints(self):
        out = []
        d = self.design
        try:
            for j in d.allJoints:
                out.append(j)
        except:
            try:
                for j in d.rootComponent.joints:
                    out.append(j)
            except:
                pass

        try:
            for j in d.allAsBuiltJoints:
                out.append(j)
        except:
            try:
                for j in d.rootComponent.asBuiltJoints:
                    out.append(j)
            except:
                pass

        return out

    # -----------------------------------------------------
    # MAPEO DE TIPOS DE JOINT
    # -----------------------------------------------------
    def _map_joint(self, j, JointTypes):
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

        # EJE
        try:
            if motion and hasattr(motion, "rotationAxisVector"):
                av = motion.rotationAxisVector
                axis = (av.x, av.y, av.z)
            elif hasattr(j, "geometry") and j.geometry and \
                 hasattr(j.geometry, "primaryAxisVector"):
                av = j.geometry.primaryAxisVector
                axis = (av.x, av.y, av.z)
        except:
            pass

        # normalizar
        ln = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
        if ln > 1e-6:
            axis = (axis[0]/ln, axis[1]/ln, axis[2]/ln)

        # límites
        if jtype == "revolute":
            rl = getattr(motion, "rotationLimits", None)
            if rl:
                lo = rl.minimumValue if getattr(rl, "isMinimumValueEnabled", False) else None
                hi = rl.maximumValue if getattr(rl, "isMaximumValueEnabled", False) else None
                if lo is not None and hi is not None and hi > lo:
                    limit = (lo, hi)
            if limit is None:
                jtype = "continuous"

        elif jtype == "prismatic":
            sl = getattr(motion, "slideLimits", None)
            if sl:
                lo = sl.minimumValue if getattr(sl, "isMinimumValueEnabled", False) else None
                hi = sl.maximumValue if getattr(sl, "isMaximumValueEnabled", False) else None
                if lo is not None and hi is not None and hi > lo:
                    limit = (lo, hi)
            if limit is None:
                limit = (-0.1, 0.1)

        return jtype, axis, limit

    # -----------------------------------------------------
    # CREAR JOINTS DESDE FUSION → URDF
    # -----------------------------------------------------
    def _create_joints(self, joints):
        try:
            from adsk.fusion import JointTypes
        except:
            JointTypes = None

        used_children = set()

        for j in joints:
            try:
                occ1 = getattr(j, "occurrenceOne", None)
                occ2 = getattr(j, "occurrenceTwo", None)

                k1 = self._occ_key(occ1)
                k2 = self._occ_key(occ2)
                if k1 not in self.occ_to_link or k2 not in self.occ_to_link:
                    continue

                l1 = self.occ_to_link[k1]
                l2 = self.occ_to_link[k2]

                if l1 == l2:
                    continue

                # elegir quién es parent/child
                if l1 in used_children and l2 not in used_children:
                    parent, child = l2, l1
                elif l2 in used_children and l1 not in used_children:
                    parent, child = l1, l2
                else:
                    parent, child = (l1, l2) if l1 < l2 else (l2, l1)

                used_children.add(child)

                jtype, axis, limit = self._map_joint(j, JointTypes)

                # obtener matrices absolutas correctas
                p_abs = self._get_abs_matrix(parent)
                c_abs = self._get_abs_matrix(child)
                p_inv = _invert_matrix(p_abs)

                rel = adsk.core.Matrix3D.create()
                rel.setToIdentity()
                rel.transformBy(p_inv)
                rel.transformBy(c_abs)

                xyz, rpy = _matrix_to_xyz_rpy(rel, self.design)

                jname = _sanitize(getattr(j, "name", "") or f"{parent}_to_{child}")

                _log(self.ui, f"[JOINT] {jname} type={jtype}")
                _log(self.ui, f"   parent_abs: {_matrix_to_str(p_abs, self.design)}")
                _log(self.ui, f"   child_abs:  {_matrix_to_str(c_abs, self.design)}")
                _log(self.ui, f"   relative:   {_matrix_to_str(rel, self.design)}")

                self.joints.append({
                    "name": jname,
                    "parent": parent,
                    "child": child,
                    "type": jtype,
                    "origin_xyz": xyz,
                    "origin_rpy": rpy,
                    "axis": axis,
                    "limit": limit
                })
            except:
                _log(self.ui, "[ERROR] Creando joint:\n" + traceback.format_exc())

    # -----------------------------------------------------
    # OBTENER MATRIZ ABSOLUTA DE UN LINK
    # -----------------------------------------------------
    def _get_abs_matrix(self, link_name):
        if link_name == self.base_link_name or not link_name:
            return _identity_matrix()
        occ = self.link_to_occ.get(link_name)
        if not occ:
            return _identity_matrix()
        key = self._occ_key(occ)
        m = self.occ_abs_mats.get(key)
        if not m:
            _log(self.ui, f"[WARN] No abs matrix for link {link_name}, identity used")
            return _identity_matrix()
        return m

    # -----------------------------------------------------
    # SELECCIONAR PADRE EN ÁRBOL SI NO TIENE JOINT
    # -----------------------------------------------------
    def _find_parent_link(self, occ, child_link):
        if not occ:
            return self.base_link_name

        visited = set()
        key = self._occ_key(occ)

        while key and key not in visited:
            visited.add(key)
            parent = self.occ_parent.get(key)
            if not parent:
                break
            if parent in self.occ_to_link:
                pl = self.occ_to_link[parent]
                if pl != child_link:
                    return pl
            key = parent

        return self.base_link_name

    # -----------------------------------------------------
    # INICIO DE BUILD LINKS + JOINTS
    # -----------------------------------------------------
    def _build_links_and_joints(self):
        root = self.design.rootComponent

        _log(self.ui, "[DBG] === BUILD LINKS & JOINTS ===")

        self.links.clear()
        self.joints.clear()
        self.occ_all.clear()
        self.occ_parent.clear()
        self.occ_abs_mats.clear()
        self.occ_to_link.clear()
        self.link_to_occ.clear()

        # 1) Recorrer occurrences usando GLOBAL directo
        _log(self.ui, "[DBG] Recorrido recursivo de occurrences (todas las carpetas/subensambles)")
        self._collect_recursive(root.occurrences)

        # 1.b) CUERPOS DIRECTOS EN EL ROOT (sin occurrence)
        #     -> "normal parts" que antes se omitían.
        root_bodies = []
        try:
            for b in root.bRepBodies:
                if b.isSolid:
                    root_bodies.append(b)
        except:
            pass

        for i, b in enumerate(root_bodies):
            link_name = _sanitize(f"root_{b.name or 'body'}_{i}")
            meshes = [{"filename": f"{link_name}_b0.stl", "body": b}]
            self.links.append({
                "name": link_name,
                "meshes": meshes,
                "occ": None
            })
            _log(self.ui, f"[ROOT-LINK] Creado link '{link_name}' para body directo del root.")

        # 2) Crear wrapper links
        self._create_wrapper_links()

        if not self.links:
            raise RuntimeError("❌ No se encontraron cuerpos sólidos en el modelo.")

        # 3) base_link
        self.base_link_name = "base_link"
        self.links.insert(0, {"name": self.base_link_name, "meshes": [], "occ": None})
        _log(self.ui, "[URDF] base_link creado.")

        # 4) joints reales
        fjoints = self._get_fusion_joints()
        _log(self.ui, f"[DBG] Total Fusion Joints: {len(fjoints)}")

        if fjoints:
            self._create_joints(fjoints)

        # 5) Crear joints fixed para links sin joints
        used_child = {j["child"] for j in self.joints}
        used_parent = {j["parent"] for j in self.joints}

        for link in self.links:
            name = link["name"]
            if name == self.base_link_name:
                continue
            if name in used_child or name in used_parent:
                continue

            occ = link["occ"]
            parent_link = self._find_parent_link(occ, name)

            p_abs = self._get_abs_matrix(parent_link)
            c_abs = self._get_abs_matrix(name)
            p_inv = _invert_matrix(p_abs)

            rel = adsk.core.Matrix3D.create()
            rel.setToIdentity()
            rel.transformBy(p_inv)
            rel.transformBy(c_abs)

            xyz, rpy = _matrix_to_xyz_rpy(rel, self.design)
            jname = _sanitize(f"fix_{parent_link}_to_{name}")

            _log(self.ui, f"[FIXED] {jname}")
            _log(self.ui, f"   parent_abs: {_matrix_to_str(p_abs, self.design)}")
            _log(self.ui, f"   child_abs:  {_matrix_to_str(c_abs, self.design)}")
            _log(self.ui, f"   rel:        {_matrix_to_str(rel, self.design)}")

            self.joints.append({
                "name": jname,
                "parent": parent_link,
                "child": name,
                "type": "fixed",
                "origin_xyz": xyz,
                "origin_rpy": rpy,
                "axis": (0,0,1),
                "limit": None
            })

        _log(self.ui, "[DBG] === END LINKS & JOINTS ===")


    # -----------------------------------------------------
    # EXPORTAR MALLAS STL
    # -----------------------------------------------------
    def _export_meshes(self):
        export_mgr = self.design.exportManager

        for link in self.links:
            meshes = link.get("meshes", [])
            if not meshes:
                continue

            for m in meshes:
                body = m["body"]
                fn = m["filename"]
                path = os.path.join(self.meshes_dir, fn)

                try:
                    opts = export_mgr.createSTLExportOptions(body, path)
                    try:
                        opts.meshRefinement = adsk.fusion.MeshRefinementOptions.High
                    except:
                        pass
                    if hasattr(opts, "isBinaryFormat"):
                        opts.isBinaryFormat = True

                    export_mgr.execute(opts)
                    _log(self.ui, f"[STL] {fn} exportado correctamente.")
                except:
                    _log(self.ui, f"[ERROR] Exportando STL {fn}:\n" + traceback.format_exc())

    # -----------------------------------------------------
    # NORMALIZAR ÁNGULOS
    # -----------------------------------------------------
    def _normalize_limits(self, lo, hi):
        try:
            if abs(lo) > 2*math.pi or abs(hi) > 2*math.pi:
                lo = math.radians(lo)
                hi = math.radians(hi)
        except:
            pass
        return lo, hi

    # -----------------------------------------------------
    # EXPORTAR URDF
    # -----------------------------------------------------
    def _write_urdf(self):
        path = os.path.join(self.output_dir, f"{self.robot_name}.urdf")

        lines = [f'<robot name="{self.robot_name}">']

        # LINKS
        for link in self.links:
            name = link["name"]
            lines.append(f'  <link name="{name}">')

            for m in link["meshes"]:
                fn = m["filename"]
                for tag in ("visual", "collision"):
                    lines.append(f'    <{tag}>')
                    lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                    lines.append(
                        f'      <geometry><mesh filename="meshes/{fn}" '
                        f'scale="0.001 0.001 0.001"/></geometry>'
                    )
                    lines.append(f'    </{tag}>')

            # Inercia "dummy"
            lines.append('    <inertial>')
            lines.append('      <mass value="1.0"/>')
            lines.append('      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>')
            lines.append('    </inertial>')
            lines.append('  </link>')

        # JOINTS
        for j in self.joints:
            name = j["name"]
            parent = j["parent"]
            child = j["child"]
            jtype = j["type"]
            ox, oy, oz = j["origin_xyz"]
            rr, pp, yy = j["origin_rpy"]

            lines.append(f'  <joint name="{name}" type="{jtype}">')
            lines.append(f'    <origin xyz="{ox} {oy} {oz}" rpy="{rr} {pp} {yy}"/>')
            lines.append(f'    <parent link="{parent}"/>')
            lines.append(f'    <child link="{child}"/>')

            if jtype in ("revolute", "continuous", "prismatic"):
                ax, ay, az = j["axis"]
                lines.append(f'    <axis xyz="{ax} {ay} {az}"/>')

            if j["limit"] and jtype in ("revolute", "prismatic"):
                lo, hi = j["limit"]
                if jtype == "revolute":
                    lo, hi = self._normalize_limits(lo, hi)
                lines.append(
                    f'    <limit lower="{lo}" upper="{hi}" effort="10" velocity="1"/>'
                )

            lines.append('  </joint>')

        lines.append('</robot>')

        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        _log(self.ui, f"[URDF] Archivo generado correctamente: {path}")

# =========================================================
# FUNCIÓN PÚBLICA
# =========================================================

def export_robot(robot_name="fusion_robot", base_output_dir=None, *args, **kwargs):
    if robot_name is None and args:
        robot_name = args[0]
    if base_output_dir is None and len(args) > 1:
        base_output_dir = args[1]

    exp = RobotExporter(robot_name, base_output_dir)
    exp.export_all()

# =========================================================
# PARTE FINAL — UTILIDADES EXTRA (LOGS Y SEGURIDAD)
# =========================================================

def _debug_matrix(label, matrix, ui, design):
    """
    Log detallado de una matriz, usado para depurar posiciones incorrectas.
    """
    try:
        s = _matrix_to_str(matrix, design)
        _log(ui, f"[MATRIX-DBG] {label}: {s}")
    except:
        pass


def _debug_occ(occ, ui, design):
    """
    Log de un occurrence completo: nombre, fullPath, assemblyContext, transform.
    """
    try:
        name = occ.name
        fp = None
        try:
            fp = occ.fullPathName
        except:
            fp = "(no fullPathName)"

        ctx = None
        try:
            ctx = occ.assemblyContext
        except:
            ctx = None

        tr = None
        try:
            tr = occ.transform2 if hasattr(occ, "transform2") else occ.transform
        except:
            tr = None

        _log(ui, f"[OCC-DBG] === OCCURRENCE ===")
        _log(ui, f"[OCC-DBG] name={name}")
        _log(ui, f"[OCC-DBG] fullPath={fp}")
        _log(ui, f"[OCC-DBG] assemblyContext={ctx.name if ctx else None}")

        if tr:
            _debug_matrix("[OCC-DBG] local_transform", tr, ui, design)
        else:
            _log(ui, f"[OCC-DBG] transform=None")
    except:
        pass


def _trace_occ_chain(occ, ui, design):
    """
    Log completo de la cadena de assemblyContext hacia arriba.
    Es extremadamente útil para detectar dónde falla la posición.
    """
    _log(ui, "[CHAIN-DBG] ---- CADENA DE CONTEXTO ----")

    cur = occ
    depth = 0
    while cur:
        try:
            nm = cur.name
        except:
            nm = "(unnamed)"

        _log(ui, f"[CHAIN-DBG] Nivel {depth}: {nm}")

        tr = None
        try:
            tr = cur.transform2 if hasattr(cur, "transform2") else cur.transform
        except:
            tr = None

        if tr:
            _debug_matrix(f"[CHAIN-DBG] local at level {depth}", tr, ui, design)
        else:
            _log(ui, f"[CHAIN-DBG] Sin transform en nivel {depth}")

        try:
            cur = cur.assemblyContext
        except:
            cur = None

        depth += 1

    _log(ui, "[CHAIN-DBG] ---- FIN CADENA ----")
