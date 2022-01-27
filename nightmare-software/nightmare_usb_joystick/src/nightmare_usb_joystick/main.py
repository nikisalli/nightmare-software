import os
import struct
import array
from fcntl import ioctl
from threading import Thread
import time
import numpy as np

# ros imports
from std_msgs.msg import Header
import rospy

# module imports
from nightmare_usb_joystick.msg import command  # noqa

# joystick vars #

axis_states = {}
button_states = {}
prev_axis_states = {}
prev_button_states = {}

mode = 'stand'  # joystick command mode

# robot command vars #

gait = 'tripod'
state = 'idle'  # string containing the robot's global state e.g. walking sitting etc

body_trasl = np.array([0, 0, 0])
body_rot = np.array([0, 0, 0])
walk_trasl = np.array([0, 0, 0])
walk_rot = np.array([0, 0, 0])

# GAITS
GAIT = {'tripod': np.array([[0, 2, 4], [1, 3, 5]]),
        'ripple': np.array([[0, 4], [1, 3], [2, 5]]),
        'wave': np.array([[0], [1], [2], [3], [4], [5]])}

header = Header()

axis_names = {
    0x00: 'jlx',
    0x01: 'jly',
    0x02: 'jrx',
    0x03: 'rz',
    0x04: 'z',
    0x05: 'jry',
    0x06: 'trottle',
    0x07: 'rudder',
    0x08: 'wheel',
    0x09: 'gas',
    0x0a: 'brake',
    0x10: 'tx',
    0x11: 'ty',
    0x12: 'hat1x',
    0x13: 'hat1y',
    0x14: 'hat2x',
    0x15: 'hat2y',
    0x16: 'hat3x',
    0x17: 'hat3y',
    0x18: 'pressure',
    0x19: 'distance',
    0x1a: 'tilt_x',
    0x1b: 'tilt_y',
    0x1c: 'tool_width',
    0x20: 'volume',
    0x28: 'misc',
}

button_names = {
    0x120: 'by',
    0x121: 'bb',
    0x122: 'ba',
    0x123: 'bx',
    0x124: 'ltrigger',
    0x125: 'rtrigger',
    0x126: 'lbase',
    0x127: 'rbase',
    0x128: 'select',
    0x129: 'start',
    0x12a: 'jlb',
    0x12b: 'jrb',
    0x12f: 'dead',
    0x130: 'a',
    0x131: 'b',
    0x132: 'c',
    0x133: 'x',
    0x134: 'y',
    0x135: 'z',
    0x136: 'tl',
    0x137: 'tr',
    0x138: 'tl2',
    0x139: 'tr2',
    0x13a: 'select',
    0x13b: 'start',
    0x13c: 'mode',
    0x13d: 'thumbl',
    0x13e: 'thumbr',

    0x220: 'dpad_up',
    0x221: 'dpad_down',
    0x222: 'dpad_left',
    0x223: 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0: 'dpad_left',
    0x2c1: 'dpad_right',
    0x2c2: 'dpad_up',
    0x2c3: 'dpad_down',
}

axis_map = []
button_map = []

EPSILON = 0.001  # for float comparisons


def feq(a, b):  # floating point equal
    return abs(a - b) < EPSILON


def handle_js():
    global jsdev
    while not rospy.is_shutdown():
        evbus = None
        try:
            evbuf = jsdev.read(8)
        except OSError:
            rospy.loginfo("joystick missing, waiting...")
            while True:
                for fn in os.listdir('/dev/input'):
                    if fn.startswith('js'):
                        device = '/dev/input/%s' % (fn)
                        break
                else:
                    time.sleep(0.8)
                    # rospy.loginfo("connect valid joystick to host")
                    continue
                break
            fn = device
            rospy.loginfo('Opening %s...' % fn)
            jsdev = open(fn, 'rb')
            continue  # continue to next iteration of outer loop
        if evbuf:
            _, value, _type, number = struct.unpack('IhBB', evbuf)
            if _type & 0x01:
                button = button_map[number]
                if button:
                    button_states[button] = bool(value)
            if _type & 0x02:
                _axis = axis_map[number]
                if _axis:
                    fvalue = value / 32767.0
                    axis_states[_axis] = fvalue


def publisher():
    # joystick
    global mode
    global prev_button_states
    global prev_axis_states

    # robot
    global state
    global walk_trasl
    global walk_rot
    global body_trasl
    global body_rot
    global gait

    height_change_timer = 0
    height_displacement = 0

    gait_change_timer = 0
    gait_num = 0

    prev_button_states = button_states.copy()
    prev_axis_states = axis_states.copy()
    rate = rospy.Rate(50)
    pub = rospy.Publisher("/control/command", command, queue_size=1)

    while not rospy.is_shutdown():
        if button_states['bb'] and prev_button_states['bb'] is False:  # joystick mode slection
            mode = 'walk'
            rospy.loginfo('mode set to walk')
        elif button_states['ba'] and prev_button_states['ba'] is False:
            mode = 'stand'
            rospy.loginfo('mode set to stand')
        elif button_states['bx'] and prev_button_states['bx'] is False:
            mode = 'leg_control'
            rospy.loginfo('mode set to leg control')
        prev_button_states['ba'] = button_states['ba']
        prev_button_states['bb'] = button_states['bb']
        prev_button_states['bx'] = button_states['bx']
        prev_button_states['by'] = button_states['by']

        if button_states['start'] and state == 'idle' and prev_button_states['start'] is False:
            state = 'awake'
            rospy.loginfo('state set to awake')
            height_displacement = 0
        elif button_states['start'] and state == 'awake' and prev_button_states['start'] is False:
            state = 'idle'
            rospy.loginfo('state set to idle')
            height_displacement = 0
        prev_button_states['start'] = button_states['start']

        if (feq(axis_states['ty'], -1) or feq(axis_states['ty'], 1)) and (time.time() - height_change_timer) > 0.05:
            height_change_timer = time.time()
            height_displacement += 0.05 * -axis_states['ty']  # increment by 0.5cm every 0.5s
            if abs(height_displacement) > 1:  # if limit exceeded set to limit
                height_displacement = -axis_states['ty']
            rospy.loginfo(f"height set to {height_displacement}")

        if (feq(axis_states['tx'], -1) or feq(axis_states['tx'], 1)) and (time.time() - gait_change_timer) > 0.5:
            gait_change_timer = time.time()
            gait_num += int(-axis_states['tx'])
            if gait_num < 0:  # if limit exceeded set to limit
                gait_num = len(GAIT) - 1
            elif gait_num > len(GAIT) - 1:
                gait_num = 0
            gait = list(GAIT.keys())[gait_num]
            rospy.loginfo(f"gait set to {gait}")

        if mode == 'walk':
            walk_trasl = [- axis_states['jly'], - axis_states['jlx'], 0]
            walk_rot = [0, 0, - axis_states['jrx']]

            body_trasl = [0, 0, height_displacement]
            body_rot = [0, 0, 0]

        elif mode == 'stand':
            walk_trasl = [0, 0, 0]
            walk_rot = [0, 0, 0]

            body_trasl = [- axis_states['jly'], - axis_states['jlx'], height_displacement]
            body_rot = [axis_states['jrx'], - axis_states['jry'], 0]

        # If None is used as the header value, rospy will automatically
        # fill it in.
        header.stamp = rospy.Time.now()
        pub.publish(command(header, body_trasl, body_rot, walk_trasl, walk_rot, mode, state, gait))
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("usb_joystick")  # blocks until registered with master

    # Iterate over the joystick devices.
    device = ""

    while True:
        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                device = '/dev/input/%s' % (fn)
                break
        else:
            time.sleep(0.8)
            rospy.loginfo("connect valid joystick to host")
            continue
        break

    # Open the joystick device.
    fn = device
    rospy.loginfo('Opening %s...' % fn)
    jsdev = open(fn, 'rb')

    # Get the device name.
    # buf = bytearray(63)
    buf = array.array('B', [0] * 64)
    ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
    js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
    rospy.loginfo('Device name: %s' % js_name)

    # Get number of axes and buttons.
    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a11, buf)  # JSIOCGAXES
    num_axes = buf[0]

    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
    num_buttons = buf[0]

    # Get the axis map.
    buf = array.array('B', [0] * 0x40)
    ioctl(jsdev, 0x80406a32, buf)  # JSIOCGAXMAP

    for axis in buf[:num_axes]:
        axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
        axis_map.append(axis_name)
        axis_states[axis_name] = 0.0

    # Get the button map.
    buf = array.array('H', [0] * 200)
    ioctl(jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP

    for btn in buf[:num_buttons]:
        btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
        button_map.append(btn_name)
        button_states[btn_name] = 0

    rospy.loginfo('%d axes found: %s' % (num_axes, ', '.join(axis_map)))
    rospy.loginfo('%d buttons found: %s' % (num_buttons, ', '.join(button_map)))

    t = Thread(target=handle_js)
    t.start()

    publisher()

    rospy.loginfo("exiting")
    os._exit(0)
