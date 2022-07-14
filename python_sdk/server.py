from FlightController import FC_Server

fc = FC_Server()
try:
    fc.start_listen_serial("COM25")
except:
    fc.start_listen_serial("/dev/serial0")
fc.init()
fc.run()
