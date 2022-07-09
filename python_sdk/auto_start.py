## this script is used to auto start the python_sdk as linux start up

import os
import sys
from time import sleep

from logger import logger

python_files_list = ["server.test.py", "mission_manager.py"]

base_path = "/home/pi/prj/python_sdk"

start_interval = 4

for file in python_files_list:
    file_path = os.path.join(base_path, file)
    os.system(f"nohup python3 {file_path} &")
    sleep(start_interval)
    logger.info(f"[AUTO_START] {file_path} started")
