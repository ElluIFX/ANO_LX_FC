from .Application import FC_Application as __FC_without_remote_layer__
from .Logger import logger
from .Remote import FC_Client, FC_Server


class FC_Controller(__FC_without_remote_layer__):
    """
    本地飞控
    """

    pass  # 只是个别名
