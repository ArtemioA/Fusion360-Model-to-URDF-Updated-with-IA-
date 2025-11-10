# -*- coding: utf-8 -*-

import adsk.core
import traceback
import os
import sys

# === CONFIG PATH SEGURO ===
# Ruta actual: .../Contents/commands/ACDC4Robot/entry.py
this_dir = os.path.dirname(os.path.abspath(__file__))
contents_dir = os.path.abspath(os.path.join(this_dir, "..", ".."))
if contents_dir not in sys.path:
    sys.path.insert(0, contents_dir)

from core import futil
from . import acdc4robot  # import relativo dentro del mismo paquete

# Identificadores del comando
CMD_ID = 'ACDC4Robot_URDF_Exporter'
CMD_NAME = 'Export to URDF (ACDC4Robot)'
CMD_DESCRIPTION = 'Export the current Fusion 360 model/assembly to URDF format.'

WORKSPACE_ID = 'FusionSolidEnvironment'
PANEL_ID = 'SolidScriptsAddinsPanel'
CMD_RESOURCES = ''

_app = adsk.core.Application.get()
_ui = _app.userInterface if _app else None

local_handlers = []


def start():
    """Registra el comando y lo agrega al panel."""
    try:
        if not _ui:
            return

        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if not cmd_def:
            cmd_def = _ui.commandDefinitions.addButtonDefinition(
                CMD_ID, CMD_NAME, CMD_DESCRIPTION, CMD_RESOURCES
            )

        # usamos nuestro propio futil (no fusion360utils) para los handlers
        futil.add_handler(cmd_def.commandCreated, command_created, local_handlers)

        workspace = _ui.workspaces.itemById(WORKSPACE_ID)
        if not workspace:
            return
        panel = workspace.toolbarPanels.itemById(PANEL_ID)
        if not panel:
            return

        control = panel.controls.itemById(CMD_ID)
        if not control:
            control = panel.controls.addCommand(cmd_def)
            control.isPromoted = True
            control.isPromotedByDefault = True

    except:
        _error('ACDC4Robot.start() failed')


def stop():
    """Elimina el comando y limpia referencias."""
    try:
        if not _ui:
            return

        workspace = _ui.workspaces.itemById(WORKSPACE_ID)
        if workspace:
            panel = workspace.toolbarPanels.itemById(PANEL_ID)
            if panel:
                control = panel.controls.itemById(CMD_ID)
                if control:
                    control.deleteMe()

        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()

    except:
        _error('ACDC4Robot.stop() failed')


def command_created(args: adsk.core.CommandCreatedEventArgs):
    """Se llama cuando el usuario hace click en el botón."""
    try:
        cmd = args.command

        futil.add_handler(cmd.execute, command_execute, local_handlers)
        futil.add_handler(cmd.executePreview, command_execute_preview, local_handlers)
        futil.add_handler(cmd.inputChanged, command_input_changed, local_handlers)
        futil.add_handler(cmd.destroy, command_destroy, local_handlers)

        acdc4robot.build_ui(cmd)

    except:
        _error('ACDC4Robot.command_created() failed')


def command_execute(args: adsk.core.CommandEventArgs):
    """Ejecuta la lógica principal (exportación URDF)."""
    try:
        acdc4robot.execute(args)
    except:
        _error('ACDC4Robot.command_execute() failed')


def command_execute_preview(args: adsk.core.CommandEventArgs):
    """Preview opcional (sin lógica)."""
    pass


def command_input_changed(args: adsk.core.InputChangedEventArgs):
    """Propaga cambios a acdc4robot."""
    try:
        acdc4robot.on_input_changed(args)
    except:
        _error('ACDC4Robot.command_input_changed() failed')


def command_destroy(args: adsk.core.CommandEventArgs):
    """Limpia handlers al cerrar el comando."""
    try:
        global local_handlers
        local_handlers = []
    except:
        _error('ACDC4Robot.command_destroy() failed')


def _error(prefix: str):
    msg = prefix + ':\n' + traceback.format_exc()
    if _ui:
        _ui.messageBox(msg, "ACDC4Robot")
    else:
        print(msg)
