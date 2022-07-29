import time

from FlightController import FC_Client


# fc = FC_Server()
# try:
#     fc.start_listen_serial("COM25")
# except:
#     fc.start_listen_serial("/dev/serial0")
# fc.init()
# fc.run()

fc = FC_Client()
fc.connect("192.168.137.23")
fc.start_sync_state(False)
fc.wait_for_connection(5)

while True:
    time.sleep(0.1)
    i = int(input("i="))
    j = int(input("j="))
    if i == 1:
        # 收线
        fc.set_pod(i,j)
    elif i == 2:
        # 放线
        fc.set_pod(i,j)
    elif i == 3:
        break
