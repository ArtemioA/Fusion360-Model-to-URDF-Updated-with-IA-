import adsk.core
import adsk.fusion
import traceback
import os
import sys

# =========================================================
# FIX PATH PARA IMPORTAR core/robot.py
# =========================================================

this_dir = os.path.dirname(os.path.abspath(__file__))
contents_dir = os.path.abspath(os.path.join(this_dir, "..", ".."))

if contents_dir not in sys.path:
    sys.path.insert(0, contents_dir)

try:
    from core import robot
    print("[ACDC4Robot] robot.py cargado desde:", robot.__file__)
except Exception:
    print("[ACDC4Robot] ERROR cargando robot.py:")
    print(traceback.format_exc())
    robot = None

_app = adsk.core.Application.get()
_ui = _app.userInterface if _app else None

# =========================================================
# LOG
# =========================================================

def _log(msg):
    """Escribe logs en consola y en TextCommands."""
    try:
        full = "[ACDC4Robot] " + msg
        print(full)

        if _ui:
            palette = _ui.palettes.itemById("TextCommands")
            if palette:
                palette.writeText(full)
    except:
        pass


# =========================================================
# EVENTOS: Input Changed
# =========================================================

def on_input_changed(args):
    """Fusion 360 llama esto cuando cambia algún input."""
    try:
        _log("on_input_changed() llamado (no hace nada).")
    except:
        pass


# =========================================================
# EJECUCIÓN REAL DEL EXPORTADOR
# =========================================================

def run_export(robot_name, output_folder):
    if robot is None:
        _log("ERROR: robot.py no se pudo cargar.")
        return

    try:
        _log(f"Iniciando exportación → robot='{robot_name}', carpeta='{output_folder}'")
        robot.export_robot(robot_name, output_folder)
        _log("✔ Exportación completada correctamente.")
    except Exception:
        _log("ERROR en run_export():\n" + traceback.format_exc())


# =========================================================
# on_execute (evento principal)
# =========================================================

def on_execute(args):
    """Evento principal cuando el usuario hace clic en OK."""
    try:
        cmd = args.firingEvent.sender
        inputs = cmd.commandInputs

        robot_name = inputs.itemById("robotName").value
        output_folder = inputs.itemById("outputFolder").value

        _log(f"Ejecutando on_execute(): robot={robot_name}, folder={output_folder}")
        run_export(robot_name, output_folder)

    except Exception:
        _log("ERROR en on_execute:\n" + traceback.format_exc())


# =========================================================
# WRAPPERS PARA ENTRY.PY
# (Fusion a veces llama execute() o command_execute())
# =========================================================

def execute(args):
    """Wrapper compatibilidad: Fusion llama acdc4robot.execute()"""
    try:
        _log("execute() llamado → redirigiendo a on_execute()")
        on_execute(args)
    except:
        _log("ERROR en execute():\n" + traceback.format_exc())


def command_execute(args):
    """Wrapper compatibilidad: Fusion llama command_execute()"""
    try:
        _log("command_execute() llamado → redirigiendo a on_execute()")
        on_execute(args)
    except:
        _log("ERROR en command_execute():\n" + traceback.format_exc())


# =========================================================
# UI DEL PANEL
# =========================================================

def build_ui(cmd: adsk.core.Command):
    """Construye los inputs del comando en Fusion."""
    try:
        inputs = cmd.commandInputs

        inputs.addStringValueInput("robotName", "Robot name", "fusion_robot")

        inputs.addStringValueInput(
            "outputFolder",
            "Output folder",
            os.path.join(os.path.expanduser("~"), "Desktop")
        )

        _log("UI creada correctamente.")

    except Exception:
        _log("ERROR en build_ui:\n" + traceback.format_exc())


# =========================================================
# ENTRY POINT DEL ADD-IN
# =========================================================

def entry():
    try:
        cmd_def = _ui.commandDefinitions.itemById("ACDC4Robot_CMD")

        if not cmd_def:
            cmd_def = _ui.commandDefinitions.addButtonDefinition(
                "ACDC4Robot_CMD",
                "ACDC4Robot Exporter",
                "Exporta el diseño actual a URDF + STLs usando robot.py"
            )

        # Registrar eventos
        cmd_def.commandCreated.add(lambda args: build_ui(args.command))
        cmd_def.inputChanged.add(on_input_changed)
        cmd_def.execute.add(on_execute)

        # Ejecutar comando
        cmd_def.execute()

        _log("entry() ejecutado → Comando lanzado correctamente.")

    except:
        _log("ERROR en entry:\n" + traceback.format_exc())
