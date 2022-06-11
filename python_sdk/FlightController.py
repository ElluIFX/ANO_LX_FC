# vscode-fold=2
import copy
import re
import threading
import time
import traceback

from logger import Exception_Catcher, logger
from SerCom32 import SerCom32

##### small functions
bytes_to_str = lambda data: " ".join([f"{b:02X}" for b in data])
float_to_byte = lambda value: int(value * 100).to_bytes(2, "little", signed=True)[::-1]
uint16_to_byte = lambda value: int(value).to_bytes(2, "little", signed=False)[::-1]
int16_to_byte = lambda value: int(value).to_bytes(2, "little", signed=True)[::-1]
uint8_to_byte = lambda value: int(value).to_bytes(1, "little", signed=False)
b_int = lambda b: int.from_bytes(b, "little", signed=True)
i_byte = lambda i, length: i.to_bytes(length, "little", signed=True)


class Byte_Var:
    value = 0

    def __init__(self, ctype="u8", var_type=int, multiplier=1):
        self._var_type = var_type
        self._multiplier = multiplier
        ctype_word_part = ctype[0]
        ctype_number_part = ctype[1:]
        if ctype_word_part.lower() == "u":
            self._signed = False
        elif ctype_word_part.lower() == "s":
            self._signed = True
        else:
            raise ValueError(f"Invalid ctype: {ctype}")
        if int(ctype_number_part) % 8 != 0:
            raise ValueError(f"Invalid ctype: {ctype}")
        self._byte_length = int(int(ctype_number_part) // 8)

    def get(self):
        return self.value

    def update(self, value):
        self.value = (
            int.from_bytes(value, "little", signed=self._signed) * self._multiplier
        )
        if self._var_type is int:
            self.value = int(self.value)
        elif self._var_type is float:
            pass
        elif self._var_type is bool:
            self.value = bool(self.value)

    def bytes(self):
        return int(self.value / self._multiplier).to_bytes(
            self._byte_length, "little", signed=self._signed
        )

    def byte_length(self):
        return self._byte_length


class FlightController:
    def __init__(self, serial_port) -> None:
        self._start_bit = [0xAA, 0x22]
        self._stop_bit = []
        self.running = False
        self.connected = False
        self.ser = SerCom32(serial_port, 115200)
        logger.info("FC: Serial port opened")
        self.set_option(0)
        self.ser.readConfig(readByteStartBit=[0xAA, 0x55], byteDataCheck="sum")
        self.read_thread = None
        self.state_update_callback = None
        self.drone_state = {
            "rol": Byte_Var("s16", float, 0.01),  # deg
            "pit": Byte_Var("s16", float, 0.01),  # deg
            "yaw": Byte_Var("s16", float, 0.01),  # deg
            "alt": Byte_Var("s32", int),  # cm
            "vel_x": Byte_Var("s16", int),  # cm/s
            "vel_y": Byte_Var("s16", int),  # cm/s
            "vel_z": Byte_Var("s16", int),  # cm/s
            "pos_x": Byte_Var("s32", int),  # cm
            "pos_y": Byte_Var("s32", int),  # cm
            "bat": Byte_Var("u16", float, 0.01),  # V
            "mode": Byte_Var("u8", int),
            "unlock": Byte_Var("u8", bool),
            "CID": Byte_Var("u8", int),
        }

    def start_listen_serial(self, callback=None):
        self.running = True
        self.state_update_callback = callback
        self.read_thread = threading.Thread(target=self._read_serial_task)
        self.read_thread.start()

    def quit(self) -> None:
        self.running = False
        if self.read_thread:
            self.read_thread.join()
        self.ser.close()
        logger.info("FC: Threads closed, FC offline")

    def set_option(self, option: int) -> None:
        self.ser.sendConfig(
            startBit=self._start_bit,
            optionBit=[option],
            stopBit=self._stop_bit,
            useSumCheck=True,
        )

    def send_form_data(self, data: list, option: int) -> None:
        self.set_option(option)
        _ = self.ser.send_form_data(data)
        logger.debug(f"FC: Send: {bytes_to_str(_)}")

    def _read_serial_task(self):
        logger.info("FC: Read thread started")
        last_heartbeat_time = time.time()
        while self.running:
            try:
                if self.ser.read_byte_serial():
                    data = self.ser.rx_byte_data
                    # logger.debug(f"FC: Read: {bytes_to_str(data)}")
                    self.update_state(data)
            except Exception as e:
                logger.error(f"FC: Read thread exception: {traceback.format_exc()}")
            if time.time() - last_heartbeat_time > 0.5:
                self.send_form_data([0x01], 0)  # 心跳包

    def update_state(self, data):
        try:
            index = 0
            for key, value in self.drone_state.items():
                length = value.byte_length()
                self.drone_state[key].update(data[index : index + length])
                index += length
            if not self.connected:
                self.connected = True
                logger.info("FC: Connected")
            # if self.state_update_callback:
            #     self.state_update_callback(self.drone_state)
            self.show_state()
        except Exception as e:
            logger.error(f"FC: Update state exception: {traceback.format_exc()}")

    def show_state(self, *args, **kwargs):
        RED = "\033[1;31m"
        # GREEN = "\033[1;32m"
        YELLOW = "\033[1;33m"
        # BLUE = "\033[1;34m"
        CYAN = "\033[1;36m"
        # PURPLE = "\033[1;35m"
        RESET = "\033[0m"
        text = ""
        text += (
            " ".join(
                [
                    f"{YELLOW}{((key[:2]+'_'+key[-1]).capitalize())}: {f'{CYAN}√' if value.get() else f'{RED}x{RESET}'}"
                    if type(value.get()) == bool
                    else f"{YELLOW}{((key[:2]+'_'+key[-1]).capitalize())}: {CYAN}{value.get()}{RESET}"
                    for key, value in self.drone_state.items()
                    if type(value.get()) != float
                ]
            )
            + " "
            + " ".join(
                [
                    f"{YELLOW}{((key[:2]+'_'+key[-1]).capitalize())}:{CYAN}{value.get():^7.02f}{RESET}"
                    for key, value in self.drone_state.items()
                    if type(value.get()) == float
                ]
            )
        )
        print(f"\r {text}\r", end="")

    def keyboard_control(self):
        horizontal_speed = 10.0
        vertical_speed = 10.0
        turn_speed = 15.0
        action_horizonal = 200
        action_vertical = 100
        action_degree = 30
        while True:
            try:
                k = input()[0]
            except:
                return
            logger.info(f"FC: Key: {k}")
            if k == "q":
                return


if __name__ == "__main__":
    fc = FlightController("COM23")
    try:
        fc.start_listen_serial(callback=fc.show_state())
        fc.keyboard_control()
    except Exception as e:
        logger.error(f"FC: Main loop exception: {traceback.format_exc()}")
    finally:
        fc.quit()
