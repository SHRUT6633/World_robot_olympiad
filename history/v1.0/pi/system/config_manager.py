import yaml
from pathlib import Path

class ConfigManager:
    _instance = None
    _config = {}

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def load(self, path: str = "config/pi_config.yaml"):
        p = Path(path)
        if p.exists():
            with open(p) as f:
                self._config = yaml.safe_load(f)
        return self._config

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

    def set(self, *keys, value):
        val = self._config
        for k in keys[:-1]:
            if k not in val:
                val[k] = {}
            val = val[k]
        val[keys[-1]] = value

    @property
    def all(self):
        return self._config
