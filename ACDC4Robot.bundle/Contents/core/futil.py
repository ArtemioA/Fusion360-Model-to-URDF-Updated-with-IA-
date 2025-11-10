import adsk.core
import traceback


def _make_handler(event_obj, callback):
    """
    Crea un handler correcto según el tipo de evento de Fusion 360.
    El callback debe aceptar un único parámetro: args.
    """

    core = adsk.core

    # CommandCreated
    if isinstance(event_obj, core.CommandCreatedEvent):
        class _Handler(core.CommandCreatedEventHandler):
            def __init__(self):
                super().__init__()

            def notify(self, args: core.CommandCreatedEventArgs):
                try:
                    callback(args)
                except:
                    _report_error("CommandCreatedEventHandler", traceback.format_exc())
        return _Handler()

    # Execute, Destroy, ExecutePreview, etc. (CommandEvent)
    if isinstance(event_obj, core.CommandEvent):
        class _Handler(core.CommandEventHandler):
            def __init__(self):
                super().__init__()

            def notify(self, args: core.CommandEventArgs):
                try:
                    callback(args)
                except:
                    _report_error("CommandEventHandler", traceback.format_exc())
        return _Handler()

    # InputChanged
    if isinstance(event_obj, core.InputChangedEvent):
        class _Handler(core.InputChangedEventHandler):
            def __init__(self):
                super().__init__()

            def notify(self, args: core.InputChangedEventArgs):
                try:
                    callback(args)
                except:
                    _report_error("InputChangedEventHandler", traceback.format_exc())
        return _Handler()

    # ValidateInputs
    if isinstance(event_obj, core.ValidateInputsEvent):
        class _Handler(core.ValidateInputsEventHandler):
            def __init__(self):
                super().__init__()

            def notify(self, args: core.ValidateInputsEventArgs):
                try:
                    callback(args)
                except:
                    _report_error("ValidateInputsEventHandler", traceback.format_exc())
        return _Handler()

    # Fallback genérico usando CommandEventHandler (último recurso)
    class _Generic(core.CommandEventHandler):
        def __init__(self):
            super().__init__()

        def notify(self, args):
            try:
                callback(args)
            except:
                _report_error("GenericEventHandler", traceback.format_exc())

    return _Generic()


def _report_error(handler_name, details):
    app = adsk.core.Application.get()
    ui = app.userInterface if app else None
    msg = f"[futil] Error in {handler_name}:\n{details}"
    if ui:
        ui.messageBox(msg)
    else:
        print(msg)


def add_handler(event_obj, callback, local_handlers):
    """
    Uso esperado (tal como lo tienes en entry.py):

        futil.add_handler(cmd_def.commandCreated, command_created, local_handlers)
        futil.add_handler(cmd.execute, command_execute, local_handlers)
        futil.add_handler(cmd.executePreview, command_execute_preview, local_handlers)
        futil.add_handler(cmd.inputChanged, command_input_changed, local_handlers)
        futil.add_handler(cmd.destroy, command_destroy, local_handlers)

    - event_obj: objeto .commandCreated, .execute, .inputChanged, .destroy, etc.
    - callback: función Python que recibe args.
    - local_handlers: lista donde se guarda la referencia para evitar GC.
    """
    if event_obj is None or callback is None:
        return

    try:
        handler = _make_handler(event_obj, callback)
        event_obj.add(handler)

        if isinstance(local_handlers, list):
            local_handlers.append(handler)

    except:
        _report_error("add_handler", traceback.format_exc())
