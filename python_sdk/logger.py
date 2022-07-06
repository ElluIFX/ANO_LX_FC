import logging
import os
import sys

import colorlog  # 控制台日志输入颜色


class log:
    def __init__(self):
        log_colors_config = {
            "DEBUG": "cyan",
            "INFO": "bold_green",
            "WARNING": "bold_yellow",
            "ERROR": "bold_red",
            "CRITICAL": "bold_purple",
        }
        text_colors_config = {
            "DEBUG": "cyan",
            "INFO": "white",
            "WARNING": "bold_yellow",
            "ERROR": "bold_red",
            "CRITICAL": "bold_purple",
        }
        self.logger = logging.getLogger()

        path = os.path.split(os.path.realpath(__file__))[0]
        logFile = os.path.join(path, "log.log")
        self.logger.setLevel(logging.DEBUG)

        fmtConsole = colorlog.ColoredFormatter(
            "\n%(log_color)s[%(levelname)s] %(asctime)s %(0_log_color)s%(message)s",
            "%Y-%b-%d %H:%M:%S",
            log_colors=log_colors_config,
            secondary_log_colors={0: text_colors_config},
        )
        fmtFile = logging.Formatter(
            "[%(levelname)s] %(asctime)s\n%(message)s",
            "(%Y-%b-%d %H:%M:%S)",
        )
        self.logger.fileHdl = logging.FileHandler(logFile, encoding="utf-8")
        self.logger.consoleHdl = logging.StreamHandler()
        self.logger.fileHdl.setLevel(logging.DEBUG)
        self.logger.consoleHdl.setLevel(logging.INFO)
        self.logger.fileHdl.setFormatter(fmtFile)
        self.logger.consoleHdl.setFormatter(fmtConsole)
        self.logger.addHandler(self.logger.fileHdl)
        self.logger.addHandler(self.logger.consoleHdl)

    def debug(self, msg, *args, **kwargs):
        self.logger.debug(msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        self.logger.info(msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self.logger.warning(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self.logger.error(msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        self.logger.critical(msg, *args, **kwargs)

    def set_level(self, level):
        levelList = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        if level in levelList:
            self.logger.setLevel(level)


logger = log()


def print(msg, *args, **kwargs):
    logger.info(msg, *args, **kwargs)


def Exception_Catcher(func):
    def wrapper(*args, **kwargs):
        import traceback

        try:
            return func(*args, **kwargs)
        except Exception as e:
            logger.error(e)
            logger.debug(traceback.format_exc())
            raise e

    return wrapper


if __name__ == "__main__":
    if len(sys.argv) == 2:
        logger.info(str(sys.argv[1]))
    if len(sys.argv) > 2:
        typeDict = {
            "-d": logger.debug,
            "-i": logger.info,
            "-w": logger.warning,
            "-e": logger.error,
            "-c": logger.critical,
        }
        line = ""
        try:
            for i in sys.argv[2:]:
                line += str(i) + " "
            typeDict[sys.argv[1].lower()](line)
        except:
            line = str(sys.argv[1]) + " " + line
            logger.info(line)
