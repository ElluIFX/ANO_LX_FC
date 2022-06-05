import configparser
import os

OPTION_NOT_FOUND = None

class ConfigManager:
    def __init__(
        self,
        file: str = "config.ini",
        default_setting: dict = {},
        sction: str = "SETTINGS",
    ):
        self._config_file = file
        self._config = configparser.ConfigParser()
        if os.path.exists(file):
            self._config.read(file)
        else:
            open(file, "w").close()
        self._sction_name = sction
        self._init_file_(default_setting.copy())

    def get(self, option: str):
        if self._config.has_option(self._sction_name, option):
            return self._config.get(self._sction_name, option)
        else:
            return OPTION_NOT_FOUND

    def getbool(self, option: str) -> bool:
        return self._config.getboolean(self._sction_name, option)
    
    def getint(self, option: str) -> int:
        return self._config.getint(self._sction_name, option)
    
    def getfloat(self, option: str) -> float:
        return self._config.getfloat(self._sction_name, option)
    
    def geteval(self, option: str) -> str:
        return eval(self.get(option))

    def set(self, option: str, value: any):
        value = str(value)
        self._config.set(self._sction_name, option, value)
        with open(self._config_file, "w") as f:
            self._config.write(f)

    def remove(self, option: str):
        self._config.remove_option(self._sction_name, option)
        with open(self._config_file, "w") as f:
            self._config.write(f)

    def _init_file_(self, default_setting: dict):

        if self._sction_name not in self._config.sections():
            self._config.add_section(self._sction_name)

        for key, value in default_setting.items():
            if self._config.has_option(self._sction_name, key):
                default_setting[key] = self._config.get(self._sction_name, key)
            else:
                self.set(key, value)

    def set_from_dict(self, setting_dict: dict):
        for key, value in setting_dict.items():
            self.set(key, value)

    def dict(self):
        items = self._config.items(self._sction_name)
        return {i[0]: i[1] for i in items}

