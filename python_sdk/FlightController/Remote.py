import socket
import sys
import time
from multiprocessing.managers import BaseManager, EventProxy, ListProxy
from threading import Event, Thread

from .Application import FC_Application
from .Logger import logger


def get_ip():
    try:
        st = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        st.connect(("10.255.255.255", 1))
        IP = st.getsockname()[0]
    except Exception:
        IP = "127.0.0.1"
    finally:
        st.close()
    return IP


class func_proxy(object):
    def __init__(self, fc: FC_Application):
        self.fc = fc

    def send_data_to_fc(
        self,
        data: bytes,
        option: int,
        need_ack: bool = False,
        _ack_retry_count: int = None,
    ):
        # logger.debug(
        #     f"[FC_Server] Transmited: option={option} data={bytes_to_str(data)}"
        # )
        return self.fc.send_data_to_fc(data, option, need_ack, _ack_retry_count)


class FC_Server(FC_Application):
    """
    飞控服务器, 负责转发指令到飞控
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._manager = None
        self._proxy = func_proxy(self)
        self._serial_callback = None
        self._proxy_state_action_id = 0
        self._proxy_state_action_target = 0
        self._proxy_state_action_op = 0
        self._proxy_state_list = [0 for var in self.state.RECV_ORDER]
        self._proxy_state_list_start_len = len(self._proxy_state_list)
        self._proxy_state_list.extend(
            [
                self.connected,
                self._proxy_state_action_id,
                self._proxy_state_action_target,
                self._proxy_state_action_op,
            ]
        )
        self._proxy_state_event = Event()
        self._set_event_callback(self._update_event_state)

    def _update_proxy_state(self, state):
        _len = self._proxy_state_list_start_len
        self._proxy_state_event.clear()
        for n, var in enumerate(self.state.RECV_ORDER):
            self._proxy_state_list[n] = var.value
        self._proxy_state_list[_len] = self.connected
        self._proxy_state_list[_len + 1] = self._proxy_state_action_id
        self._proxy_state_list[_len + 2] = self._proxy_state_action_target
        self._proxy_state_list[_len + 3] = self._proxy_state_action_op
        self._proxy_state_event.set()
        if callable(self._serial_callback):
            self._serial_callback(state)

    def _update_event_state(self, event_code, event_operator):
        self._proxy_state_action_id += 1
        self._proxy_state_action_target = event_code
        self._proxy_state_action_op = event_operator

    def start_listen_serial(
        self,
        serial_port: str,
        bit_rate: int = 500000,
        print_state=True,
        callback=None,
    ):
        self._serial_callback = callback
        return super().start_listen_serial(
            serial_port,
            bit_rate,
            print_state,
            self._update_proxy_state,
        )

    def init(self, port=5654, authkey=b"fc"):
        """
        初始化服务器
        """
        if not self.running:
            raise Exception("Serial listening must be started before manager_init")

        class FC_Manager(BaseManager):
            pass

        FC_Manager.register("get_proxy", callable=lambda: self._proxy)
        FC_Manager.register(
            "get_proxy_state_list",
            callable=lambda: self._proxy_state_list,
            proxytype=ListProxy,
        )
        FC_Manager.register(
            "get_proxy_state_event",
            callable=lambda: self._proxy_state_event,
            proxytype=EventProxy,
        )
        self._manager = FC_Manager(address=("", port), authkey=authkey)
        self._port = port
        logger.info("[FC_Server] Manager initialized")

    def run(self):
        """
        启动服务器(永久阻塞)
        由于python多进程间不能共享数据, 故服务器不可能脱离飞控的主类(this one)运行,
        新创建的进程所有的通信都是虚假的,数据被发送到一个复制出的的"伪类"中(参见py多进程的实现)
        因此, manager的start方法是不可用的,只能使用阻塞本进程的serve_forever方法.
        (如果需要本飞控服务器启动后同时干别的事情,请在启动服务器前使用threading创建线程)
        """
        if self._manager is None:
            raise Exception("Manager must be initialized before run")
        address = get_ip() + ":" + str(self._port)
        local_address = f"127.0.0.1:{self._port}"
        logger.info(f"[FC_Server] Manager serving on {address}, {local_address}")
        self._manager.get_server().serve_forever()


class FC_Client(FC_Application):
    """
    飞控客户端, 与服务器通信
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._manager = None
        self._func_proxy = None
        self._proxy_state_list = None
        self._proxy_state_event = None

    def start_listen_serial(
        self,
        serial_port: str,
        bit_rate: int = 500000,
        print_state=True,
        callback=None,
    ):
        """
        客户端无需监听串口, 调用start_sync_state替代
        """
        logger.warning(
            "[FC_Client] do not need to start serial listening, auto calling start_sync_state instead"
        )
        return self.start_sync_state(print_state, callback)

    def start_sync_state(self, print_state=True, callback=None):
        """
        与服务器同步状态变量
        """
        self._state_update_callback = callback
        self._print_state_flag = print_state
        self._listen_thread = Thread(target=self._sync_state_task)
        self._listen_thread.daemon = True
        self._listen_thread.start()
        logger.info("[FC_Client] State sync started")

    def _sync_state_task(self):
        _len = len(self.state.RECV_ORDER)
        last_id = 0
        while self.running:
            time.sleep(0.01)
            try:
                self._proxy_state_event.wait()
                for n, var in enumerate(self.state.RECV_ORDER):
                    var.value = self._proxy_state_list[n]
                self.connected = self._proxy_state_list[_len]
                id = self._proxy_state_list[_len + 1]
                if id != last_id:
                    last_id = id
                    if id != 0:
                        event_code = self._proxy_state_list[_len + 2]
                        event_operator = self._proxy_state_list[_len + 3]
                        if event_operator == 0x01:  # set
                            self.event.EVENT_CODE[event_code].set()
                        elif event_operator == 0x02:  # clear
                            self.event.EVENT_CODE[event_code].clear()
                self._proxy_state_event.clear()
                if callable(self._state_update_callback):
                    self._state_update_callback(self.state)
                if self._print_state_flag:
                    self._print_state()
            except Exception as e:
                logger.error(f"[FC_Client] State sync error: {e}")
                if "WinError" in str(e) or "Broken pipe" in str(e):
                    logger.warning("[FC_Client] Connection lost, trying to reconnect")
                    self.running = False
                    for i in range(3):
                        logger.warning(f"[FC_Client] Trying to reconnect {i+1}/3")
                        try:
                            self._manager = None  # 先析构
                            self.connect(*self._last_connection_args)
                        except:
                            logger.warning(f"[FC_Client] Reconnect failed {i+1}/3")
                            time.sleep(1)
                        else:
                            logger.info("[FC_Client] Successfully reconnected")
                            time.sleep(0.1)
                            break
                    else:
                        logger.error("[FC_Client] All reconnect failed, closing")
        logger.warning("[FC_Client] State sync thread stopped")

    def connect(self, host="127.0.0.1", port=5654, authkey=b"fc"):
        """
        连接服务器
        """

        class FC_Manager(BaseManager):
            pass

        self._last_connection_args = (host, port, authkey)
        FC_Manager.register("get_proxy")
        FC_Manager.register("get_proxy_state_list")
        FC_Manager.register("get_proxy_state_event")
        self._manager = FC_Manager(address=(host, port), authkey=authkey)
        self._manager.connect()
        logger.info("[FC_Client] Manager connected to %s:%d" % (host, port))
        self._func_proxy = self._manager.get_proxy()
        self._proxy_state_list = self._manager.get_proxy_state_list()
        self._proxy_state_event = self._manager.get_proxy_state_event()
        logger.info("[FC_Client] Methods registered")
        self.running = True

    def send_data_to_fc(
        self,
        data: bytes,
        option: int,
        need_ack: bool = False,
        _ack_retry_count: int = None,
    ):
        if not self._func_proxy:
            raise Exception("FC_Client not connected")
        return self._func_proxy.send_data_to_fc(
            data, option, need_ack, _ack_retry_count
        )
