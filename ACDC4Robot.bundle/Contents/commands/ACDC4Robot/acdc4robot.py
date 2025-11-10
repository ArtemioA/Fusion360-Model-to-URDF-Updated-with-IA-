import adsk.core
import adsk.fusion
import traceback
import os
import sys

# === FIX PATH ===
# Este archivo está en:
#   Contents/commands/ACDC4Robot/acdc4robot.py
# Queremos poder importar:
#   Contents/core/robot.py
this_dir = os.path.dirname(os.path.abspath(__file__))
contents_dir = os.path.abspath(os.path.join(this_dir, "..", ".."))
if contents_dir not in sys.path:
    sys.path.insert(0, contents_dir)

from core import robot  # import absoluto estable

_app = adsk.core.Application.get()
_ui = _app.userInterface if _app else None


def build_ui(cmd: adsk.core.Command):
    """Construye la interfaz de usuario del comando en Fusion 360."""
    try:
        inputs = cmd.commandInputs

        inputs.addStringValueInput(
            'robotName',
            'Robot name',
            'acdc_robot'
        )

        inputs.addStringValueInput(
            'outputFolder',
            'URDF base output folder',
            'C:/ACDC4Robot_URDF'
        )

        inputs.addBoolValueInput(
            'useAssemblyJoints',
            'Use existing joints (NO USADO EN ESTA VERSION)',
            True,
            '',
            True
        )

    except:
        msg = 'ACDC4Robot.build_ui() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)


def on_input_changed(args: adsk.core.InputChangedEventArgs):
    """Detecta cambios en los inputs del comando."""
    try:
        # Si quieres debug, descomenta:
        # changed = args.input
        # if _ui:
        #     _ui.messageBox(f"[DEBUG] Input cambiado: {changed.id}")
        pass
    except:
        msg = 'ACDC4Robot.on_input_changed() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)


def execute(args: adsk.core.CommandEventArgs):
    """
    Lógica principal:
    - Lee parámetros del panel.
    - Verifica diseño activo.
    - Llama a robot.export_robot().
    SI FALLA: muestra error y TERMINA el comando para que el botón siga funcionando.
    """
    try:
        cmd = args.command
        inputs = cmd.commandInputs

        robot_name = _get_str(inputs, 'robotName', 'acdc_robot')
        output_folder = _get_str(inputs, 'outputFolder', 'C:/ACDC4Robot_URDF')
        use_joints = _get_bool(inputs, 'useAssemblyJoints', True)  # no usado aún

        design = adsk.fusion.Design.cast(_app.activeProduct)
        if not design:
            raise RuntimeError('No active Fusion 360 design found.')

        root_comp = design.rootComponent
        if not root_comp:
            raise RuntimeError('No root component found.')

        # Crear carpeta base si no existe
        if not os.path.exists(output_folder):
            os.makedirs(output_folder, exist_ok=True)

        # Llamada al exportador real
        robot.export_robot(
            robot_name=robot_name,
            base_output_dir=output_folder
        )

        if _ui:
            _ui.messageBox(
                "Exportación URDF + STL completada.\n"
                f"Revisa la carpeta:\n{os.path.join(output_folder, robot_name + '_urdf')}",
                "ACDC4Robot"
            )

    except:
        # IMPORTANTE: no dejamos el comando colgado,
        # lo cerramos para que el botón se pueda usar de nuevo.
        msg = 'ACDC4Robot.execute() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg, "ACDC4Robot")
            try:
                _ui.terminateActiveCommand()
            except:
                # Si por alguna razón falla terminar el comando, al menos logueamos.
                print("Failed to terminate active command after error.")
        else:
            print(msg)


def _get_str(inputs: adsk.core.CommandInputs, input_id: str, default: str) -> str:
    """Obtiene un string desde los inputs con fallback seguro."""
    try:
        inp = inputs.itemById(input_id)
        if not inp:
            return default

        val = getattr(inp, 'value', None)
        if isinstance(val, str) and val.strip():
            return val.strip()

        text = getattr(inp, 'text', None)
        if isinstance(text, str) and text.strip():
            return text.strip()

        return default

    except:
        msg = 'ACDC4Robot._get_str() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)
        return default


def _get_bool(inputs: adsk.core.CommandInputs, input_id: str, default: bool) -> bool:
    """Obtiene un booleano desde los inputs con fallback seguro."""
    try:
        inp = inputs.itemById(input_id)
        if not inp:
            return default

        val = getattr(inp, 'value', None)
        if isinstance(val, bool):
            return val

        return default

    except:
        msg = 'ACDC4Robot._get_bool() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)
        return default
