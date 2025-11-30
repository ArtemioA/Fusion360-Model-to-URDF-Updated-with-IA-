import adsk.core
import traceback

from .commands.ACDC4Robot import entry as acdc_entry


def run(context):
    try:
        acdc_entry.start()
    except:
        app = adsk.core.Application.get()
        ui = app.userInterface if app else None
        msg = 'ACDC4Robot.run() failed:\n{}'.format(traceback.format_exc())
        if ui:
            ui.messageBox(msg)
        else:
            print(msg)


def stop(context):
    try:
        acdc_entry.stop()
    except:
        app = adsk.core.Application.get()
        ui = app.userInterface if app else None
        msg = 'ACDC4Robot.stop() failed:\n{}'.format(traceback.format_exc())
        if ui:
            ui.messageBox(msg)
        else:
            print(msg)
