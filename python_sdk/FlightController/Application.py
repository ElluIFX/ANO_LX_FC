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
        self._realtime_control_thread = None
        self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
        self._realtime_control_running = False

    def reset_position_prediction(self):
        """
        复位位置融合预测(伪造通用位置传感器)
        """
        self.send_general_position(0, 0, 0)
        self._action_log("reset position prediction")

    def rectangular_move(self, x: int, y: int, speed: int) -> None:
        """
        匿名坐标系下的水平移动 (程控模式下有效)
        x,y: 匿名坐标系下的坐标
        speed: 移动速度:10-300 cm/s

        移动半径在0-10000 cm
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
            if timeout_s > 0 and time.time() - t0 > timeout_s:
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
            if timeout_s > 0 and time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for last command done timeout")
                return False
        self._action_log("wait ok", "last cmd done")
        return True

    def wait_for_hovering(self, timeout_s=10) -> bool:
        """
        等待进入悬停状态
        """
        t0 = time.time()
        time.sleep(0.5)  # 等待数据回传
        while not self.hovering:
            time.sleep(0.1)
            if timeout_s > 0 and time.time() - t0 > timeout_s:
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
            if timeout_s > 0 and time.time() - t0 > timeout_s:
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
            if timeout_s > 0 and time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for takeoff done timeout")
                return False
        if self.state.alt_add.value < 20:
            logger.warning("[FC] takeoff failed, low altitude")
            return False
        time.sleep(1)  # 等待机身高度稳定
        self._action_log("wait ok", "takeoff done")
        return True

    def programmed_takeoff(self, height: int, speed: int) -> None:
        """
        程控起飞模式,使用前先解锁 (危险,当回传频率过低时容易冲顶,应使用较小的安全速度)
        """
        self._action_log("manual takeoff start", f"{height}cm")
        last_mode = self.state.mode.value
        hgt = self.state.alt_add
        timeout_value = height / speed * 2  # 安全时间
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

    def _realtime_control_task(self, freq):
        logger.info("[FC] realtime control task started")
        last_send_time = time.time()
        pauesed = False
        while self._realtime_control_running:
            while time.time() - last_send_time < 1 / freq:
                time.sleep(0.01)
            last_send_time = time.time()
            if self.state.mode.value != self.HOLD_POS_MODE:
                if not pauesed:
                    self._action_log("realtime control", "paused")
                pauesed = True
                continue
            if pauesed:  # 取消暂停时先清空数据避免失控
                pauesed = False
                self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
                self._action_log("realtime control", "resumed")
            try:
                self.send_realtime_control_data(*self._realtime_control_data_in_xyzYaw)
            except Exception as e:
                logger.warning(f"[FC] realtime control task error: {e}")
        logger.info("[FC] realtime control task stopped")

    def start_realtime_control(self, freq: float = 15) -> None:
        """
        开始自动发送实时控制, 仅在定点模式下有效
        freq: 后台线程自动发送控制帧的频率

        警告: 除非特别需要, 否则不建议使用该方法, 而是直接调用 send_realtime_control_data,
        虽然在主线程崩溃的情况下, 子线程会因daemon自动退出, 但这一操作的延时是不可预知的

        本操作不会强行锁定控制模式于所需的定点模式, 因此可以通过切换到程控来暂停实时控制
        """
        if self._realtime_control_running:
            self.stop_realtime_control()
        self._realtime_control_running = True
        self._realtime_control_thread = threading.Thread(
            target=self._realtime_control_task, args=(freq,), daemon=True
        )
        self._thread_list.append(self._realtime_control_thread)
        self._realtime_control_thread.start()

    def stop_realtime_control(self) -> None:
        """
        停止自动发送实时控制
        """
        self._realtime_control_running = False
        if self._realtime_control_thread:
            self._realtime_control_thread.join()
            self._thread_list.remove(self._realtime_control_thread)
            self._realtime_control_thread = None
        self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]

    def update_realtime_control(
        self, vel_x: int = None, vel_y: int = None, vel_z: int = None, yaw: int = None
    ) -> None:
        """
        更新自动发送实时控制的目标值
        vel_x,vel_y,vel_z: cm/s 匿名坐标系
        yaw: deg/s 顺时针为正

        注意默认参数为None, 代表不更新对应的值, 若不需要建议置为0而不是留空
        """
        for n, target in enumerate([vel_x, vel_y, vel_z, yaw]):
            if target is not None:
                self._realtime_control_data_in_xyzYaw[n] = target

    def set_action_log(self, output: bool) -> None:
        """
        设置动作日志输出
        """
        self.settings.action_log_output = output
