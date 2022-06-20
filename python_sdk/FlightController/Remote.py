from .Protocal import FC_Protocol


class FC_Server(FC_Protocol):
    """
    飞控服务器, 负责转发指令到飞控 #TODO
    """


class FC_Client(FC_Protocol):
    """
    飞控客户端, 与服务器通信 #TODO
    """
