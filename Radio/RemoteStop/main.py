# Default template for XBee MicroPython projects


# TODO
# - Add line passing
# - Receive heartbeat packet from Rover radio
# - Loops
# - LED's,Switches
# IMPORTS
import xbee
import time

# DEFINES
RADIOMODE_UNKNOWN = 0
RADIOMODE_REMOTE = 1
RADIOMODE_ROVER = 2
AVAILABLE_ROVERS = ['XBEE_C']

radio = xbee.XBee()
radioMode = RADIOMODE_UNKNOWN
myID = radio.atcmd('NI')


def find_device(node_id):
    """
    Finds the XBee device with the given node identifier in the network and
    returns it.

    :param node_id: Node identifier of the XBee device to find.

    :return: The XBee device with the given node identifier or ``None`` if it
        could not be found.
    """
    for dev in xbee.discover():
        if dev['node_id'] == node_id:
            return dev
    return None


def map_radiomode_tostring(mode):
    if mode == RADIOMODE_UNKNOWN:
        return "UNKNOWN"
    elif mode == RADIOMODE_REMOTE:
        return "REMOTE"
    elif mode == RADIOMODE_ROVER:
        return "ROVER"
    else:
        return "UNKNOWN"


def run_fastloop():
    a = 1


def run_mediumloop():
    if radioMode == RADIOMODE_ROVER:
        # send heartbeat
        a = 1
    a = 1


def run_slowloop():
    a = 1


def run_veryslowloop():
    a = 1

# INIT


print("My ID: " + myID)
device_is_rover = False
for i in range(0, len(AVAILABLE_ROVERS)):
    if AVAILABLE_ROVERS[i] == myID:
        device_is_rover = True
if device_is_rover == True:
    radioMode = RADIOMODE_ROVER
else:
    radioMode = RADIOMODE_REMOTE
print("Radio Mode: " + map_radiomode_tostring(radioMode))



# EXEC
while True:
    time.sleep(1)
    run_slowloop()