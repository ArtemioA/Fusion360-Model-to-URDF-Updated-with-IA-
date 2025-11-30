# -*- coding: utf-8 -*-
"""
Shared configuration/state for the ACDC4Robot command.
Mantiene las variables globales necesarias para comunicar los módulos del Add-In.
"""

# --- Paths and state ---
SDF_FILE_DIR = ""
ROBOT_NAME = ""
TEXT_PALETTE = None  # adsk.core.Palette

# --- Metadata ---
AUTHOR_NAME = ""
MODEL_DESCRIPTION = ""
ROBOT_DESCRIPTION_FORMAT = ""   # "URDF", "SDFormat", "MJCF", "URDF+", "None"
SIMULATION_ENVIRONMENT = ""     # "Gazebo", "PyBullet", "MuJoCo", "None"


# ---------------------------------------------------------------------------
# Setters & Getters
# ---------------------------------------------------------------------------

def set_sdf_file_dir(sdf_file_dir: str):
    """Guarda la ruta de salida del archivo SDF."""
    global SDF_FILE_DIR
    SDF_FILE_DIR = sdf_file_dir


def get_sdf_file_dir() -> str:
    """Devuelve la ruta actual de export SDF."""
    return SDF_FILE_DIR


def set_robot_name(robot_name: str):
    """Guarda el nombre del robot."""
    global ROBOT_NAME
    ROBOT_NAME = robot_name


def get_robot_name() -> str:
    """Devuelve el nombre actual del robot."""
    return ROBOT_NAME


def set_text_palette(text_palette):
    """Guarda la referencia a la paleta de TextCommands para logs."""
    global TEXT_PALETTE
    TEXT_PALETTE = text_palette


def get_text_palette():
    """Devuelve la paleta TextCommands si existe."""
    return TEXT_PALETTE


def set_author_name(author_name: str):
    """Guarda el nombre del autor del modelo."""
    global AUTHOR_NAME
    AUTHOR_NAME = author_name


def get_author_name() -> str:
    """Devuelve el nombre del autor."""
    return AUTHOR_NAME


def set_model_description(model_description: str):
    """Guarda la descripción del modelo."""
    global MODEL_DESCRIPTION
    MODEL_DESCRIPTION = model_description


def get_model_description() -> str:
    """Devuelve la descripción del modelo."""
    return MODEL_DESCRIPTION


def set_rdf(robot_description_format: str):
    """Define el formato de descripción de robot (URDF, SDF, etc.)."""
    global ROBOT_DESCRIPTION_FORMAT
    ROBOT_DESCRIPTION_FORMAT = robot_description_format


def get_rdf() -> str:
    """Devuelve el formato de descripción de robot seleccionado."""
    return ROBOT_DESCRIPTION_FORMAT


def set_sim_env(sim_env: str):
    """Define el entorno de simulación (Gazebo, MuJoCo, PyBullet, etc.)."""
    global SIMULATION_ENVIRONMENT
    SIMULATION_ENVIRONMENT = sim_env


def get_sim_env() -> str:
    """Devuelve el entorno de simulación seleccionado."""
    return SIMULATION_ENVIRONMENT
