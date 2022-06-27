import socket
import sys
import time
from multiprocessing.managers import BaseManager, EventProxy, ListProxy
from threading import Event, Thread

from .Logger import logger
from .Protocal import FC_Protocol


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
    def __init__(self, fc: FC_Protocol):
        self.fc = fc

    def send_data_to_fc(
        self,
        data: bytes,
        option: int,
        need_ack: bool = False,
        __ack_retry_count: int = None,
    ):
        # logger.debug(
        #     f"[FC_Server] Transmited: option={option} data={bytes_to_str(data)}"
        # )
        return self.fc.send_data_to_fc(data, option, need_ack, __ack_retry_count)


class FC_Server(FC_Protocol):
    """
    飞控服务器, 负责转发指令到飞控
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.__manager = None
        self.__proxy = func_proxy(self)
        self.__serial_callback = None
        self.__proxy_state_action_id = 0
        self.__proxy_state_action_target = 0
        self.__proxy_state_action_op = 0
        self.__proxy_state_list = [0 for var in self.state.RECV_ORDER]
        self.__proxy_state_list_start_len = len(self.__proxy_state_list)
        self.__proxy_state_list.extend(
            [
                self.connected,
                self.__proxy_state_action_id,
                self.__proxy_state_action_target,
                self.__proxy_state_action_op,
            ]
        )
        self.__proxy_state_event = Event()
        self.__set_event_callback__(self.__update_event_state)

    def __update_proxy_state(self, state):
        _len = self.__proxy_state_list_start_len
        self.__proxy_state_event.clear()
        for n, var in enumerate(self.state.RECV_ORDER):
            self.__proxy_state_list[n] = var.value
        self.__proxy_state_list[_len] = self.connected
        self.__proxy_state_list[_len + 1] = self.__proxy_state_action_id
        self.__proxy_state_list[_len + 2] = self.__proxy_state_action_target
        self.__proxy_state_list[_len + 3] = self.__proxy_state_action_op
        self.__proxy_state_event.set()
        if callable(self.__serial_callback):
            self.__serial_callback(state)

    def __update_event_state(self, event_code, event_operator):
        self.__proxy_state_action_id += 1
        self.__proxy_state_action_target = event_code
        self.__proxy_state_action_op = event_operator

    def start_listen_serial(
        self,
        serial_port: str,
        bit_rate: int = 500000,
        print_state=True,
        callback=None,
        daemon=True,
    ):
        self.__serial_callback = callback
        return super().start_listen_serial(
            serial_port,
            bit_rate,
            print_state,
            self.__update_proxy_state,
            daemon,
        )

    def init(self, port=5654, authkey=b"fc"):
        """
        初始化服务器
        """
        if not self.running:
            raise Exception("Serial listening must be started before manager_init")

        class FC_Manager(BaseManager):
            pass

        FC_Manager.register("get_proxy", callable=lambda: self.__proxy)
        FC_Manager.register(
            "get_proxy_state_list",
            callable=lambda: self.__proxy_state_list,
            proxytype=ListProxy,
        )
        FC_Manager.register(
            "get_proxy_state_event",
            callable=lambda: self.__proxy_state_event,
            proxytype=EventProxy,
        )
        self.__manager = FC_Manager(address=("", port), authkey=authkey)
        self.__port = port
        logger.info("[FC_Server] Manager initialized")

    def run(self):
        """
        启动服务器(永久阻塞)
        由于python多进程间不能共享数据, 故服务器不可能脱离飞控的主类(this one)运行,
        新创建的进程所有的通信都是虚假的,数据被发送到一个复制出的的"伪类"中(参见py多进程的实现)
        因此, manager的start方法是不可用的,只能使用阻塞本进程的serve_forever方法.
        (如果需要本飞控服务器启动后同时干别的事情,请在启动服务器前使用threading创建线程)
        """
        if self.__manager is None:
            raise Exception("Manager must be initialized before run")
        address = get_ip() + ":" + str(self.__port)
        local_address = f"127.0.0.1:{self.__port}"
        logger.info(f"[FC_Server] Manager serving on {address}, {local_address}")
        self.__manager.get_server().serve_forever()


class FC_Client(FC_Protocol):
    """
    飞控客户端, 与服务器通信
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.__manager = None
        self.__func_proxy = None
        self.__proxy_state_list = None
        self.__proxy_state_event = None

    def start_listen_serial(
        self,
        serial_port: str,
        bit_rate: int = 500000,
        print_state=True,
        callback=None,
        daemon=True,
    ):
        """
        客户端无需监听串口, 调用start_sync_state替代
        """
        logger.warning(
            "[FC_Client] do not need to start serial listening, auto calling start_sync_state instead"
        )
        return self.start_sync_state(print_state, callback, daemon)

    def start_sync_state(self, print_state=True, callback=None, daemon=True):
        """
        与服务器同步状态变量
        """
        self.__state_update_callback = callback
        self.__print_state_flag = print_state
        self.__listen_thread = Thread(target=self.__sync_state_task)
        self.__listen_thread.setDaemon(daemon)
        self.__listen_thread.start()
        logger.info("[FC_Client] State sync started")

    def __sync_state_task(self):
        _len = len(self.state.RECV_ORDER)
        last_id = 0
        while self.running:
            time.sleep(0.01)
            try:
                self.__proxy_state_event.wait()
                self.__proxy_state_event.clear()
                for n, var in enumerate(self.state.RECV_ORDER):
                    var.value = self.__proxy_state_list[n]
                self.connected = self.__proxy_state_list[_len]
                id = self.__proxy_state_list[_len + 1]
                if id != last_id:
                    last_id = id
                    if id != 0:
                        event_code = self.__proxy_state_list[_len + 2]
                        event_operator = self.__proxy_state_list[_len + 3]
                        if event_operator == 0x01:  # set
                            self.event.EVENT_CODE[event_code].set()
                        elif event_operator == 0x02:  # clear
                            self.event.EVENT_CODE[event_code].clear()
                        logger.info(f"[FC_Client] Event {event_code} {event_operator}")
                if callable(self.__state_update_callback):
                    self.__state_update_callback(self.state)
                if self.__print_state_flag:
                    self._FC_Base_Uart_Comunication__print_state()
            except Exception as e:
                logger.error(f"[FC_Client] State sync error: {e}")
                if "WinError" in str(e):
                    logger.warning("[FC_Client] Connection lost, trying to reconnect")
                    self.running = False
                    for i in range(3):
                        logger.warning(f"[FC_Client] Trying to reconnect {i+1}/3")
                        try:
                            self.__manager = None  # 先析构
                            self.connect(*self.__last_connection_args)
                        except:
                            logger.warning(f"[FC_Client] Reconnect failed {i+1}/3")
                            time.sleep(1)
                        else:
                            logger.info("[FC_Client] Successfully reconnected")
                            time.sleep(0.1)
                            break
                    else:
                        logger.error("[FC_Client] All reconnect failed, closing")
                        sys.exit(1)
        logger.warning("[FC_Client] State sync thread stopped")

    def connect(self, host="127.0.0.1", port=5654, authkey=b"fc"):
        """
        连接服务器
        """

        class FC_Manager(BaseManager):
            pass

        self.__last_connection_args = (host, port, authkey)
        FC_Manager.register("get_proxy")
        FC_Manager.register("get_proxy_state_list")
        FC_Manager.register("get_proxy_state_event")
        self.__manager = FC_Manager(address=(host, port), authkey=authkey)
        self.__manager.connect()
        logger.info("[FC_Client] Manager connected to %s:%d" % (host, port))
        self.__func_proxy = self.__manager.get_proxy()
        self.__proxy_state_list = self.__manager.get_proxy_state_list()
        self.__proxy_state_event = self.__manager.get_proxy_state_event()
        logger.info("[FC_Client] Methods registered")
        self.running = True

    def send_data_to_fc(
        self,
        data: bytes,
        option: int,
        need_ack: bool = False,
        __ack_retry_count: int = None,
    ):
        if not self.__func_proxy:
            raise Exception("FC_Client not connected")
        return self.__func_proxy.send_data_to_fc(
            data, option, need_ack, __ack_retry_count
        )
