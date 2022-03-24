# Be sure to use python2.7
#
# Author: Jonathan Shulgach
# Last updated: 10/29/21

import sys
import Leap
import threading
import Queue as queue
import time


class LeapListener(threading.Thread):
    def __init__(self, parent, stop_event, rate=10, name='LeapListener'):
        """Thread that handles incoming messages like a producer/consumer system and executes the case

        :param parent: Flexi object
        :param stop_event: Event type object to handle stopping queue
        :param name: name of thread looping rate in Hz
        :param rate: integer value of thread looping rate in Hz
        """
        threading.Thread.__init__(self)
        self.t_rate = 1/float(rate)  # Hz
        self.name = name
        self.parent = parent
        self.stop_event = stop_event
        print("Leap Listener Object Initialized")

    def run(self):
        # execute these commands as soon as the thread starts
        self.monitor_leap()

    def monitor_leap(self):
        # Get the most recent frame and report some basic information
        while not self.stop_event.is_set():
            frame = self.parent.controller.frame()
            if not frame.hands.is_empty:
                self.parent.msg_queue.put(frame)
            time.sleep(self.t_rate)


class QueuedMessageHandler(threading.Thread):
    def __init__(self, parent, stop_event, rate=10, name='Queued Message Handler'):
        """Thread that handles incoming messages like a producer/consumer system and executes the case

        :param parent: Flexi object
        :param stop_event: Event type object to handle stopping queue
        :param name: name of thread looping rate in Hz
        :param rate: integer value of thread looping rate in Hz
        """
        threading.Thread.__init__(self)
        self.t_rate = 1/float(rate)  # Hz
        self.i = 0  # internal counter
        self.name = name
        self.parent = parent
        self.stop_event = stop_event
        print("Queued Message Handler Object Initialized")

    def run(self):
        # run the message handler with the built in 'run' threading method
        self.msg_handler()

    def msg_handler(self):
        # This continuously loops looking for commands sent to the msg_queue
        # not using right now
        while not self.stop_event.is_set():
            # do nothing if the asyncio thread is dead and no more data in the queue
            time.sleep(self.t_rate)
            self.i += 1
            if not self.parent.leap_listener.is_alive():
                del self
                sys.exit()

            if not self.parent.msg_queue.empty():
                # If we actually have something, get it from the queue and send it to the state machine!
                msg = self.parent.msg_queue.get()
                self.state_machine(msg)

    def state_machine(self, msg):
        """Handles incoming message from queue system and executes the appropriate event case

        :param msg: Can be any data type (variant)
        :return:
        """
        if type(msg) == Leap.Frame:
            # Get hand position and orientation from frame
            temp = self.get_frame_data(msg)
            if len(temp) == 1:
                # Only one hand
                x, y, z, roll, pitch, yaw = temp[0]

                palm_state = self.check_palm_state(roll)
                print(palm_state)
        else:
            pass

    def check_palm_state(self, roll):
        # state whether palm is facing up or down
        if roll < 30 and roll > -30:
            return "palm down"
        elif roll > 100 and roll < 180 or roll < -90 and roll > -180:
            return "palm up"
        else:
            return "none"

    def get_frame_data(self, frame, verbose=False):
        # Get hands
        hand_data = []
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"

            print("grab stength: "+str(hand.grab_strength))

            # Get hand position
            x = hand.palm_position.x
            y = hand.palm_position.y
            z = hand.palm_position.z

            if verbose:
                print("  %s, id %d, position: %s" % (handType, hand.id, hand.palm_position))

            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction

            # Calculate the hand's pitch, roll, and yaw angles
            pitch = direction.pitch * Leap.RAD_TO_DEG
            roll = normal.roll * Leap.RAD_TO_DEG
            yaw = direction.yaw * Leap.RAD_TO_DEG

            if verbose:
                print("  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (pitch, roll, yaw))
            hand_data.append([x, y, z, roll, pitch, yaw])
        return hand_data


class Flexi(LeapListener):
    def __init__(self):
        """This Flexi object initializes a leap controller device and constantly listens to presence of data
        TO-DO: make this its own thread so the process can be interrupted from the terminal

        """
        self.controller = Leap.Controller()
        self.msg_queue = queue.Queue()

        self.finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        self.bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']

        self.leap_listener_event = threading.Event()
        self.leap_listener = LeapListener(self, self.leap_listener_event)
        self.leap_listener.start()

        self.queue_msg_handler_event = threading.Event()
        self.queue_msg_handler = QueuedMessageHandler(self, self.queue_msg_handler_event)
        self.queue_msg_handler.start()

    def destroy(self):
        print("Stopping "+str(self.leap_listener.name))
        self.leap_listener_event.set()
        print("Stopping "+str(self.queue_msg_handler.name))
        self.queue_msg_handler_event.set()

        # missing something to delete object instance and save memory, will figure out later


def main():

    flexi = Flexi()
    print("palm-test\n\nThis demo tracks the hand position and determines whether the palm is facing\nup or down.\n\n")
    print("Press Enter to quit...")
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        flexi.destroy()


# For manual debugging
if __name__ == "__main__":
    main()

