import threading
import time

import numpy as np

from .Logger import logger
from .Protocal import FC_Protocol


class FC_Application(FC_Protocol):
    """
    应用层, 基于协议层进行开发, 不触及底层通信
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    def reset_position_prediction(self):
        """
        复位位置融合预测(伪造通用位置传感器)
        """
        self.send_general_position(0, 0, 0)
        self._action_log("reset position prediction")

    def cordinate_move(self, x: int, y: int, speed: int) -> None:
        """
        匿名坐标系下的水平移动: (程控模式下有效)
        移动半径在0-10000 cm
        x,y: 匿名坐标系下的坐标
        speed: 移动速度:10-300 cm/s
        """
        div = y / x if x != 0 else np.inf
        target_deg = -np.arctan(div) / np.pi * 180
        distance = np.sqrt(x**2 + y**2)
        if target_deg < 0:
            target_deg += 360
        self._action_log("cordinate move", f"{x}, {y}, {speed}")
        self.horizontal_move(int(distance), speed, int(target_deg))

    def wait_for_connection(self, timeout_s=-1) -> bool:
        """
        等待飞控连接
        """
        t0 = time.time()
        while not self.connected:
            time.sleep(0.1)
            if timeout_s != -1 and time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for fc connection timeout")
                return False
        self._action_log("wait ok", "fc connection")
        return True

    def wait_for_last_command_done(self, timeout_s=10) -> bool:
        """
        等待最后一次指令完成
        """
        t0 = time.time()
        time.sleep(0.5)  # 等待数据回传
        while not self.last_command_done:
            time.sleep(0.1)
            if time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for last command done timeout")
                return False
        self._action_log("wait ok", "last cmd done")
        return True

    def wait_for_stabilizing(self, timeout_s=10) -> bool:
        """
        等待进入悬停状态
        """
        t0 = time.time()
        time.sleep(0.5)  # 等待数据回传
        while not self.hovering:
            time.sleep(0.1)
            if time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for stabilizing timeout")
                return False
        self._action_log("wait ok", "stabilizing")
        return True

    def wait_for_lock(self, timeout_s=10) -> bool:
        """
        等待锁定
        """
        t0 = time.time()
        while self.state.unlock.value:
            time.sleep(0.1)
            if time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for lock timeout")
                return False
        self._action_log("wait ok", "locked")
        return True

    def wait_for_takeoff_done(self, z_speed_threshold=4, timeout_s=5) -> bool:
        """
        等待起飞完成
        """
        t0 = time.time()
        time.sleep(1)  # 等待加速完成
        while self.state.vel_z.value < z_speed_threshold:
            time.sleep(0.1)
            if time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for takeoff done timeout")
                return False
        if self.state.alt_add.value < 20:
            logger.warning("[FC] takeoff failed, low altitude")
            return False
        time.sleep(1)  # 等待机身高度稳定
        self._action_log("wait ok", "takeoff done")
        return True

    def manual_takeoff(self, height: int, speed: int) -> None:
        """
        程控起飞模式,使用前先解锁 (危险,当回传频率过低时容易冲顶,应使用较小的安全速度)
        """
        self._action_log("manual takeoff start", f"{height}cm")
        last_mode = self.state.mode.value
        hgt = self.state.height
        timeout_value = height / speed  # 安全时间
        self.set_flight_mode(self.HOLD_POS_MODE)
        time.sleep(0.1)  # 等待模式设置完成
        self.send_realtime_control_data(vel_z=speed)
        takeoff_time = time.time()
        while hgt.value < height - 20:
            time.sleep(0.1)
            if time.time() - takeoff_time > timeout_value:  # 超时, 终止
                logger.warning("[FC] takeoff timeout, force stop")
                self.stablize()
                return
            else:
                self.send_realtime_control_data(vel_z=speed)  # 持续发送
        self.send_realtime_control_data(vel_z=10)
        while hgt.value < height - 2:
            time.sleep(0.1)
        self.stablize()
        self.set_flight_mode(last_mode)  # 还原模式
        self._action_log("manual takeoff ok")
