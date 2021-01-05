import os
from threading import Thread
from pynput.keyboard import Key, Listener, KeyCode

from nightmare_keyboard.msg import command  # noqa
from std_msgs.msg import Header
import rospy

key_states = {KeyCode.from_char('w'): False,
              KeyCode.from_char('a'): False,
              KeyCode.from_char('s'): False,
              KeyCode.from_char('d'): False,
              KeyCode.from_char('q'): False,
              KeyCode.from_char('e'): False,
              KeyCode.from_char('i'): False,
              KeyCode.from_char('j'): False,
              KeyCode.from_char('k'): False,
              KeyCode.from_char('l'): False,
              KeyCode.from_char('u'): False,
              KeyCode.from_char('o'): False,
              KeyCode.from_char('m'): False,
              Key.space: False,
              Key.shift: False,
              Key.ctrl: False}

keys = [KeyCode.from_char('w'),
        KeyCode.from_char('a'),
        KeyCode.from_char('s'),
        KeyCode.from_char('d'),
        KeyCode.from_char('q'),
        KeyCode.from_char('e'),
        KeyCode.from_char('i'),
        KeyCode.from_char('j'),
        KeyCode.from_char('k'),
        KeyCode.from_char('l'),
        KeyCode.from_char('u'),
        KeyCode.from_char('o'),
        KeyCode.from_char('m'),
        Key.space,
        Key.shift,
        Key.ctrl]


header = Header()


def handle_keyboard():
    with Listener(on_press=handle_press, on_release=handle_release) as listener:
        listener.join()


def handle_press(key):
    if key in keys:
        key_states[key] = True


def handle_release(key):
    if key in keys:
        key_states[key] = False


def publisher():
    rate = rospy.Rate(50)
    pub = rospy.Publisher('/control/keyboard', command, queue_size=1)
    state = 'sleep'
    mode = 'stand'
    gait = 'tripod'
    prev_spacebar = key_states[Key.space]
    prev_m = key_states[KeyCode.from_char('m')]

    walk_direction = [0] * 3
    body_displacement = [0] * 3

    while not rospy.is_shutdown():
        # If None is used as the header value, rospy will automatically fill it in.
        header.stamp = rospy.Time.now()

        walk_direction = [0] * 3
        body_displacement = [0] * 6

        if key_states[Key.space] and state == 'sleep' and prev_spacebar is False:
            state = 'stand'
            rospy.loginfo('state set to stand')
        elif key_states[Key.space] and state == 'stand' and prev_spacebar is False:
            state = 'sleep'
            rospy.loginfo('state set to sleep')
        prev_spacebar = key_states[Key.space]

        if key_states[KeyCode.from_char('m')] and mode == 'stand' and prev_m is False:
            mode = 'walk'
            rospy.loginfo('mode set to walk')
        elif key_states[KeyCode.from_char('m')] and mode == 'walk' and prev_m is False:
            mode = 'stand'
            rospy.loginfo('mode set to stand')
        prev_m = key_states[KeyCode.from_char('m')]

        if state == 'stand':
            if mode == 'walk':
                if key_states[KeyCode.from_char('w')]:
                    walk_direction[1] -= 1
                if key_states[KeyCode.from_char('s')]:
                    walk_direction[1] += 1
                if key_states[KeyCode.from_char('a')]:
                    walk_direction[0] -= 1
                if key_states[KeyCode.from_char('d')]:
                    walk_direction[0] += 1
                if key_states[KeyCode.from_char('q')]:
                    walk_direction[2] -= 1
                if key_states[KeyCode.from_char('e')]:
                    walk_direction[2] += 1
            elif mode == 'stand':
                if key_states[KeyCode.from_char('w')]:
                    body_displacement[1] -= 1
                if key_states[KeyCode.from_char('s')]:
                    body_displacement[1] += 1
                if key_states[KeyCode.from_char('a')]:
                    body_displacement[0] -= 1
                if key_states[KeyCode.from_char('d')]:
                    body_displacement[0] += 1
                if key_states[KeyCode.from_char('q')]:
                    body_displacement[5] -= 1
                if key_states[KeyCode.from_char('e')]:
                    body_displacement[5] += 1
                if key_states[KeyCode.from_char('i')]:
                    body_displacement[4] -= 1
                if key_states[KeyCode.from_char('k')]:
                    body_displacement[4] += 1
                if key_states[KeyCode.from_char('j')]:
                    body_displacement[3] += 1
                if key_states[KeyCode.from_char('l')]:
                    body_displacement[3] -= 1
                if key_states[KeyCode.from_char('u')]:
                    body_displacement[5] -= 1
                if key_states[KeyCode.from_char('o')]:
                    body_displacement[5] += 1
            if key_states[Key.shift]:
                body_displacement[2] += 1
            if key_states[Key.ctrl]:
                body_displacement[2] -= 1

        pub.publish(command(header,
                            body_displacement,
                            walk_direction,
                            state,
                            gait))

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('keyboard')  # blocks until registered with master

    t = Thread(target=handle_keyboard)
    t.start()

    publisher()

    rospy.loginfo('exiting')
    os._exit(0)
