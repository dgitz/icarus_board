# Default template for XBee MicroPython projects
import time

# TODO
"""
 - Add line passing
 - Receive/Transmit heartbeat
 - Loops (DONE)
 - LED's,Switches
 - Firmware Version
"""
# IMPORTS
ARCHITECTURE_XBEE = False
if ARCHITECTURE_XBEE:
    import xbee
    from machine import Pin


# CONFIG
AVAILABLE_ROVERS = ['XBEE_C']
FASTLOOP_RATE = 10.0
MEDIUMLOOP_RATE = 2.0
SLOWLOOP_RATE = 0.5
VERYSLOWLOOP_RATE = 0.1
LOGGER_VERBOSITY = 1
FIRMWARE_MAJORVERSION = 0
FIRMWARE_MINORVERSION = 0
FIRMWARE_BUILDVERSION = 0

# DEFINES
LEDPIN_STATUS_ID = -1
RADIOMODE_UNKNOWN = 0
RADIOMODE_REMOTE = 1
RADIOMODE_ROVER = 2

DIAGNOSTICLEVEL_DEBUG = 0  # This Level is solely for development/debugging only.
DIAGNOSTICLEVEL_INFO = 1  # This Level is purely for informational use only.
DIAGNOSTICLEVEL_NOTICE = 2  # This Level is a higher form of information and does not imply that anything is wrong.
DIAGNOSTICLEVEL_WARN = 3  # This Level implies that a program is not running as expected, but may continue to operate in a diminished capacity.
DIAGNOSTICLEVEL_ERROR = 4  # This Level implies that a program will not initialize or some other kind of crash.
DIAGNOSTICLEVEL_FATAL = 5  # This Level implies that a program has failed so bad it can cause injury to itself or others.

# VARIABLES
fastloop_counter = 0
mediumloop_counter = 0
slowloop_counter = 0
veryslowloop_counter = 0
start_time = time.time()
radio = []
myID = 'NOT AVAILABLE'
if ARCHITECTURE_XBEE:
    radio = xbee.XBee()
    myID = radio.atcmd('NI')
radioMode = RADIOMODE_UNKNOWN


def logger(verbosity, message):
    if verbosity >= LOGGER_VERBOSITY:
        print("[" + str(time.time()) + " " + map_diagnosticlevel_tostring(verbosity) + "] " + message)


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


def map_diagnosticlevel_tostring(level):
    if level == DIAGNOSTICLEVEL_DEBUG:
        return "DEBUG"
    elif level == DIAGNOSTICLEVEL_INFO:
        return "INFO"
    elif level == DIAGNOSTICLEVEL_NOTICE:
        return "NOTICE"
    elif level == DIAGNOSTICLEVEL_WARN:
        return "WARN"
    elif level == DIAGNOSTICLEVEL_ERROR:
        return "ERROR"
    elif level == DIAGNOSTICLEVEL_FATAL:
        return "FATAL"


def run_fastloop():
    logger(DIAGNOSTICLEVEL_DEBUG, "FastLoop Execute")


def run_mediumloop():
    logger(DIAGNOSTICLEVEL_DEBUG, "MediumLoop Execute")
    if radioMode == RADIOMODE_ROVER:
        # send heartbeat
        a = 1


def run_slowloop():
    logger(DIAGNOSTICLEVEL_DEBUG, "SlowLoop Execute")


def run_veryslowloop():
    logger(DIAGNOSTICLEVEL_DEBUG, "VerySlowLoop Execute")
    logger(DIAGNOSTICLEVEL_INFO,
           "FastLoop Count: " + str(fastloop_counter) +
           " MediumLoop Count: " + str(mediumloop_counter) +
           " SlowLoop Count: " + str(slowloop_counter) +
           " VerySlowLoop Count: " + str(veryslowloop_counter))


def blink_firmware():
    """
    Blinks Status Pin with Firmware Version Numbers
    :return: Nothing
    """
    if ARCHITECTURE_XBEE:
        ledpin_status.value(0)
        for v in range(0, FIRMWARE_MAJORVERSION):
            ledpin_status.value(1)
            time.sleep(1)
            ledpin_status.value(0)
            time.sleep(1)
        time.sleep(2)
        for v in range(0, FIRMWARE_MINORVERSION):
            ledpin_status.value(1)
            time.sleep(0.5)
            ledpin_status.value(0)
            time.sleep(0.5)
        time.sleep(2)
        for v in range(0, FIRMWARE_BUILDVERSION):
            ledpin_status.value(1)
            time.sleep(0.1)
            ledpin_status.value(0)
            time.sleep(0.1)
        time.sleep(2)
        ledpin_status.value(0)


# INIT

logger(DIAGNOSTICLEVEL_NOTICE, "Firmware: " +
       "Major: " + str(FIRMWARE_MAJORVERSION) +
       " Minor: " + str(FIRMWARE_MINORVERSION) +
       " Build: " + str(FIRMWARE_BUILDVERSION))
if ARCHITECTURE_XBEE == False:
    logger(DIAGNOSTICLEVEL_WARN, "Current Device Architecture is Not Supported!")
logger(DIAGNOSTICLEVEL_INFO, "My ID: " + str(myID))
device_is_rover = False
for i in range(0, len(AVAILABLE_ROVERS)):
    if AVAILABLE_ROVERS[i] == myID:
        device_is_rover = True
if device_is_rover:
    radioMode = RADIOMODE_ROVER
else:
    radioMode = RADIOMODE_REMOTE
logger(DIAGNOSTICLEVEL_INFO, "Radio Mode: " + map_radiomode_tostring(radioMode))
time_lastfastloop = start_time
time_lastmediumloop = start_time
time_lastslowloop = start_time
time_lastveryslowloop = start_time
fastloop_time = 1.0 / FASTLOOP_RATE
mediumloop_time = 1.0 / MEDIUMLOOP_RATE
slowloop_time = 1.0 / SLOWLOOP_RATE
veryslowloop_time = 1.0 / VERYSLOWLOOP_RATE
ledpin_status = 0
if ARCHITECTURE_XBEE:
    ledpin_status = Pin(LEDPIN_STATUS_ID, Pin.OUT, value=0)
# EXEC
while True:
    time.sleep(0.01)
    if (time.time() - time_lastfastloop) >= fastloop_time:
        time_lastfastloop = time.time()
        fastloop_counter += 1
        run_fastloop()
    if (time.time() - time_lastmediumloop) >= mediumloop_time:
        time_lastmediumloop = time.time()
        mediumloop_counter += 1
        run_mediumloop()
    if (time.time() - time_lastslowloop) >= slowloop_time:
        time_lastslowloop = time.time()
        slowloop_counter += 1
        run_slowloop()
    if (time.time() - time_lastveryslowloop) >= veryslowloop_time:
        time_lastveryslowloop = time.time()
        veryslowloop_counter += 1
        run_veryslowloop()
