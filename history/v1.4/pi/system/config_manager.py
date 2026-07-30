# =============================================================================
# config_manager.py — ConfigManager (Singleton YAML Configuration Loader)
# =============================================================================
# This module provides a singleton ConfigManager that loads all robot
# configuration from a YAML file (config/pi_config.yaml by default) and
# exposes it via a nested-key .get() method.
#
# Singleton behavior:
#   ConfigManager() always returns the same instance (see __new__). This
#   means that wherever it is instantiated in the codebase (boot.py,
#   main.py, manager.py, etc.), all code shares the same configuration
#   dictionary. If any part of the code calls .set() to modify a value,
#   every other consumer sees the change.
#
# Why a singleton?
#   The config file is read from disk once (when .load() is first called).
#   Subsequent uses just read from the in-memory dict. This is fast and
#   avoids redundant file I/O.
#
# Connection to other files:
#   - manager.py:    self.config = ConfigManager(); self.config.load()
#   - boot.py:       config = ConfigManager(); config.load()
#   - main.py:       config = mgr.config (already loaded by SystemManager)
#   - Any module that needs a config value can import and call:
#       ConfigManager().get(...)
#
# The YAML file (config/pi_config.yaml) defines keys like:
#   sensors:
#     camera:
#       device: 0
#       width: 640
#       height: 480
#       fps: 60
#     vl53l0x_left:
#       xshut_pin: 17
#     vl53l0x_right:
#       xshut_pin: 27
#     vl53l1x_front:
#       xshut_pin: 22
#   hardware:
#     leds:
#       green_pin: 23
#       red_pin: 24
#     switch:
#       pin: 25
# =============================================================================

import yaml
from pathlib import Path


class ConfigManager:
    # Static/class-level singleton state.
    # _instance holds the single ConfigManager object.
    # _config holds the parsed YAML dictionary (shared across all users).
    _instance = None
    _config = {}

    # -------------------------------------------------------------------------
    # __new__(cls)
    # -------------------------------------------------------------------------
    # Overrides object creation to implement the Singleton pattern.
    # Only the first call to ConfigManager() creates a new instance.
    # All subsequent calls return the existing _instance.
    # This means ConfigManager() is idempotent and lightweight.
    # -------------------------------------------------------------------------
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    # -------------------------------------------------------------------------
    # load(path="config/pi_config.yaml")
    # -------------------------------------------------------------------------
    # Reads the YAML file at the given path (relative to the working
    # directory) and stores the parsed dictionary in self._config.
    #
    # If the file does not exist, _config remains {} and .get() will
    # return defaults for every key. This allows running without a config
    # file during development, but sensors may fail if required pins are not
    # specified.
    #
    # Called by:
    #   - SystemManager.__init__()  (manager.py)
    #   - power_on_self_test()     (boot.py)
    #   - boot_sequence()          (boot.py)
    #
    # Config path: The default is "config/pi_config.yaml" relative to
    # the working directory (usually the project root). If the robot's
    # startup script runs from a different directory, this path will be
    # wrong and config will not load.
    # -------------------------------------------------------------------------
    def load(self, path: str = "config/pi_config.yaml"):
        p = Path(path)
        if p.exists():
            with open(p) as f:
                self._config = yaml.safe_load(f)
        return self._config

    # -------------------------------------------------------------------------
    # get(*keys, default=None)
    # -------------------------------------------------------------------------
    # Navigates the nested config dictionary using positional key arguments.
    #
    # Example:
    #   config.get("sensors", "camera", "width", default=640)
    #   → returns _config["sensors"]["camera"]["width"] or 640 if missing.
    #
    # If any intermediate key is missing or if a key traverses a non-dict,
    # the default is returned immediately.
    #
    # This is the primary way every module reads configuration.
    # Changing the YAML file changes the behavior of every module that
    # reads a value via .get().
    #
    # Default values in the code:
    #   Each .get() call includes a sensible default so the system can
    #   operate even without a config file. However, some values (like
    #   GPIO pins) MUST match the actual wiring — so relying on defaults
    #   is only safe during testing on a known setup.
    # -------------------------------------------------------------------------
    def get(self, *keys, default=None):
        val = self._config
        for k in keys:
            if isinstance(val, dict):
                val = val.get(k)
                if val is None:
                    return default
            else:
                return default
        return val

    # -------------------------------------------------------------------------
    # set(*keys, value)
    # -------------------------------------------------------------------------
    # Sets a value deep inside the nested config dictionary, creating
    # intermediate dicts as needed.
    #
    # Example:
    #   config.set("sensors", "camera", "width", value=800)
    #   → _config["sensors"]["camera"]["width"] = 800
    #
    # Used rarely — mostly for runtime calibration overrides or test
    # fixtures. Changes are in-memory only; they do NOT persist to the
    # YAML file.
    # -------------------------------------------------------------------------
    def set(self, *keys, value):
        val = self._config
        for k in keys[:-1]:
            if k not in val:
                val[k] = {}
            val = val[k]
        val[keys[-1]] = value

    # -------------------------------------------------------------------------
    # all (property)
    # -------------------------------------------------------------------------
    # Returns the entire config dictionary (for debugging / inspection).
    # -------------------------------------------------------------------------
    @property
    def all(self):
        return self._config
