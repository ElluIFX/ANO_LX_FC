from .Logger import logger
from .Protocal import FC_Protocol as __FC_Protocol
from .Remote import FC_Client, FC_Server


class FC_Controller(__FC_Protocol):
    """
    本地飞控
    """

    pass  # 只是个别名
