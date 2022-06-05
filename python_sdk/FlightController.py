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
b_int = lambda b: int.from_bytes(b, "big", signed=True)
i_byte = lambda i, length: i.to_bytes(length, "little", signed=True)


class FlightController:
    def __init__(self, serial_port) -> None:
        self._start_bit = [0xAA, 0x22]
        self._stop_bit = []
        self.running = True
        self.connected = False
        self.ser = SerCom32(serial_port, 115200)
        logger.info("FC: Serial port opened")
        self.set_option(0)
        self.ser.readConfig(readByteStartBit=[0xAA, 0x55], byteDataCheck="sum")
        self.read_thread = None
        self.state_update_callback = None
        self.drone_state = {
            "armed": False,  # u8
            "flying": False,  # u8
            "alt_hold": False,  # u8
            "height": 0.0,  # s32
            "heading": 0.0,  # s16
            "spd_x": 0.0,  # s16
            "spd_y": 0.0,  # s16
            "spd_z": 0.0,  # s16
            "spd_rad": 0.0,  # s16
            "voltage": 0.0,  # s16
            "rc_mask": False,  # u8
            "pc_stat": False,  # u8
        }

    def start_listen_serial(self, callback=None):
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

    def wait_for_connect(self, timeout=-1) -> bool:
        t0 = time.time()
        logger.info("FC: Waiting for FC connection")
        while True:
            if timeout > 0 and time.time() - t0 > timeout:
                logger.error("FC: Wait for connect timeout")
                return False
            self.send_form_data([0x01], 0)
            time.sleep(0.5)
            if self.connected:
                logger.info("FC: Connected")
                return True
            time.sleep(0.5)

    def _read_serial_task(self):
        logger.info("FC: Read thread started")
        while self.running:
            try:
                if self.ser.read_byte_serial():
                    data = self.ser.rx_byte_data
                    # logger.debug(f"FC: Read: {bytes_to_str(data)}")
                    self.update_state(data)
            except Exception as e:
                logger.error(f"FC: Read thread exception: {traceback.format_exc()}")

    def update_state(self, data):
        try:
            self.drone_state["armed"] = data[0] != 0
            self.drone_state["flying"] = data[1] != 0
            self.drone_state["alt_hold"] = data[2] != 0
            height = b_int(data[3:7]) / 1000.0
            if height > 10 * 100 or height < -10 * 100:  # limit height
                height = 0.0
            self.drone_state["height"] = height
            self.drone_state["heading"] = b_int(data[7:9]) / 100.0
            self.drone_state["spd_x"] = b_int(data[9:11]) / 100.0
            self.drone_state["spd_y"] = b_int(data[11:13]) / 100.0
            self.drone_state["spd_z"] = b_int(data[13:15]) / 100.0
            self.drone_state["spd_rad"] = b_int(data[15:17]) / 100.0
            self.drone_state["voltage"] = b_int(data[17:19]) / 100.0
            self.drone_state["rc_mask"] = data[19] == 1
            self.drone_state["pc_stat"] = data[20] != 0
            self.connected = True
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
                    f"{YELLOW}{((key[:2]+'_'+key[-1]).capitalize())}: {f'{CYAN}√' if value else f'{RED}x{RESET}'}"
                    if type(value) == bool
                    else f"{YELLOW}{((key[:2]+'_'+key[-1]).capitalize())}: {CYAN}{value}{RESET}"
                    for key, value in self.drone_state.items()
                    if type(value) != float
                ]
            )
            + " "
            + " ".join(
                [
                    f"{YELLOW}{((key[:2]+'_'+key[-1]).capitalize())}:{CYAN}{value:^7.02f}{RESET}"
                    for key, value in self.drone_state.items()
                    if type(value) == float
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
                k = input()
            except:
                return
            logger.debug(f"FC: Key: {k}")
            if k == "t":
                self.quit()
                return
            elif k == "x":
                self.rt_stop()
            elif k == "w":
                self.rt_set_XY_speed(horizontal_speed, 0)
                # self.flow_go_X(action_horizonal, horizontal_speed)
            elif k == "s":
                self.rt_set_XY_speed(-horizontal_speed, 0)
                # self.flow_go_X(-action_horizonal, horizontal_speed)
            elif k == "a":
                self.rt_set_XY_speed(0, horizontal_speed)
                # self.flow_go_Y(action_horizonal, horizontal_speed)
            elif k == "d":
                self.rt_set_XY_speed(0, -horizontal_speed)
                # self.flow_go_Y(-action_horizonal, horizontal_speed)
            elif k == "q":
                self.rt_set_Yaw_speed(-turn_speed)
                # self.flow_turn(-action_degree, turn_speed)
            elif k == "e":
                self.rt_set_Yaw_speed(turn_speed)
                # self.flow_turn(action_degree, turn_speed)
            elif k == "z":
                self.rt_set_Z_speed(vertical_speed)
                # self.flow_go_Z(action_vertical, vertical_speed)
            elif k == "c":
                self.rt_set_Z_speed(-vertical_speed)
                # self.flow_go_Z(-action_vertical, vertical_speed)
            elif k.isdigit():
                val = int(k) * 20
                logger.info(f"FC: Set height to {val}")
                self.rt_set_Height(val)
            elif k == "f":
                if self.drone_state["flying"]:
                    self.land()
                else:
                    self.takeoff()
            elif k == "r":
                self.emergency()

    def send_flow_control_command(self, cmd: int, value: int, speed: int):
        """
        cmd   Action     Value  Speed
        0x00: Reset      None   None
        0x01: TakeOff    None   None
        0x02: Land       None   None
        0x03: Up         cm     cm/s
        0x04: Down       cm     cm/s
        0x05: Forward    cm     cm/s
        0x06: Backward   cm     cm/s
        0x07: Left       cm     cm/s
        0x08: Right      cm     cm/s
        0x09: LeftTurn   cm     cm/s
        0x0A: RightTurn  cm     cm/s
        0xA0: Emergency  None   None
        Note: All values must be positive!
        Note2: All horizontal movements are in world coordinates!(Axis inited at every takeoff)
        """
        self.set_option(0x01)
        data = (
            b"\x10"
            + uint16_to_byte(cmd)
            + uint16_to_byte(abs(value))
            + uint16_to_byte(abs(speed))
        )
        self.ser.send_form_data(data)

    def send_realtime_control_command(self, cmd: int, value_1: float, value_2: float):
        """
        cmd   Action                Value_1     Value_2
        0x01: Set Horizonal Speed   X cm/s      Y cm/s
        0x02: Set Vertical Speed    Z cm/s      None
        0x03: Set Yaw Speed         Z Rad/s     None
        0x04: Set Height            Z cm        None
        0x05: All Stop              None        None
        X     Positive to the front
        Y     Positive to the left
        Z     Positive to the up
        Yaw   Positive to the clockwise
        """
        self.set_option(0x02)
        data = uint8_to_byte(cmd) + float_to_byte(value_1) + float_to_byte(value_2)
        self.ser.send_form_data(data)

    def rt_set_XY_speed(self, x: float, y: float):
        """
        x positive to the front / y positive to the left
        """
        self.send_realtime_control_command(0x01, x, y)

    def rt_set_Z_speed(self, z: float):
        """
        positive to the up
        """
        self.send_realtime_control_command(0x02, z, 0)

    def rt_set_Yaw_speed(self, yaw: float):
        """
        positive to the clockwise
        """
        self.send_realtime_control_command(0x03, yaw, 0)

    def rt_set_Height(self, height: float):
        """
        Notice: Set value will be limit to +-200cm based on current height inside FC!!
        """
        if height > 160:
            height = 160  # 防止撞实验室天花板
        self.send_realtime_control_command(0x04, height, 0)

    def rt_stop(self):
        self.send_realtime_control_command(0x05, 0, 0)

    def flow_go_X(self, distance: float, speed: float):
        """
        positive to the front
        """
        if distance > 0:
            self.send_flow_control_command(0x05, distance, speed)
        elif distance < 0:
            self.send_flow_control_command(0x06, -distance, speed)

    def flow_go_Y(self, distance: float, speed: float):
        """
        positive to the left
        """
        if distance > 0:
            self.send_flow_control_command(0x07, distance, speed)
        elif distance < 0:
            self.send_flow_control_command(0x08, -distance, speed)

    def flow_go_Z(self, distance: float, speed: float):
        """
        positive to the up
        """
        if distance > 0:
            self.send_flow_control_command(0x03, distance, speed)
        elif distance < 0:
            self.send_flow_control_command(0x04, -distance, speed)

    def flow_turn(self, degree: float, speed: float):
        """
        positive to the clockwise
        """
        if degree > 0:
            self.send_flow_control_command(0x0A, degree, speed)
        elif degree < 0:
            self.send_flow_control_command(0x09, -degree, speed)

    def takeoff(self):
        self.send_flow_control_command(0x01, 0, 0)

    def land(self):
        self.send_flow_control_command(0x02, 0, 0)

    def emergency(self):
        self.send_flow_control_command(0xA0, 0, 0)


if __name__ == "__main__":
    fc = FlightController("COM23")
    fc.start_listen_serial(callback=fc.show_state())
    fc.wait_for_connect(2)
    fc.keyboard_control()
    fc.quit()
