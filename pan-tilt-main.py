# Be sure to use python2.7
#
# Author: Jonathan Shulgach
# Last updated: 12/5/21

import sys
import Leap
import threading
import Queue as queue
import time
import math
from datetime import datetime as dt
import numpy as np
import serial

# library for inverse kinematics
#from ikpy.chain import Chain
#from ikpy.link import OriginLink, URDFLink


class ArduinoComm(threading.Thread):
    def __init__(self, parent, stop_event, rate=20, name='Arduino Comm handler',
                 port='COM6', baud=115200, publish_rate=1, verbose=False):
        """ Thread that handles all serial communication to the arduino

        :param parent:
        :param stop_event:
        :param rate:
        :param name:
        :param port: (str) COM port number for arduino
        :param baud: (int) baud rate for arduino serial comm
        :param publish_rate: (int) specific publishing rate to send data to arduino
        :param verbose:
        """
        threading.Thread.__init__(self)
        self.t_rate = 1 / float(rate)  # Hz
        self.publish_rate = 1/float(publish_rate)  # Hz (separate rate to send data to Arduino through serial
        self.i = 0  # internal counter
        self.name = name
        self.parent = parent
        self.stop_event = stop_event
        self.verbose = verbose
        self.arduino = None
        self.port = port
        self.baud = baud
        self.connected = False
        self.msg = None
        self.t_now = None
        self.t_prev = dt.now()
        print(str(self.name)+": Object Initialized")

        self.connect()  # try connecting to arduino if it's plugged in

    def run(self):
        # execute these commands as soon as the thread starts
        while not self.stop_event.is_set() and self.connected:
            # do nothing if the asyncio thread is dead and no data in the queue
            time.sleep(self.t_rate)
            self.i += 1
            if not self.parent.arduino_queue.empty():
                self.msg = self.parent.arduino_queue.get()
                #print("updated value: "+str(self.msg)+"\n")
                # Just using time different to execute arduino publishing
                self.t_now = dt.now()
                delta = self.t_now - self.t_prev

                if delta.total_seconds() > self.publish_rate:
                    print("Send")

                    self.t_prev = dt.now()
                    #if self.verbose: print(str(self.name) + ": Sending to Arduino: "+str(self.msg)+"\n")
                    result = self.write_read(self.msg)
                    #if self.verbose: print(str(self.name) + ": Response from Arduino: "+str(result)+"\n")

                    if result:
                        # Update parent joint state
                        self.parent.cur_eef_pos = self.parent.goal_eef_pos


        self.destroy()

    def destroy(self):
        print(str(self.name)+" destroyed")

    def connect(self):
        try:
            self.arduino = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.1)
            self.connected = True
            print("[Arduino Communicator]: Arduino connection success!")
            # Send initial position
            self.write_read("35,112")
        except:
            self.connected = False
            #self.destroy()
            print("Arduino connection failed")

    def write_read(self, msg):
        if self.arduino:
            #self.arduino.write(bytes(msg))
            self.arduino.write(msg)

            #data = self.arduino.readlines()
            #print(data)
            data = False
            return data


class LeapListener(threading.Thread):
    def __init__(self, parent, stop_event, rate=1, name='LeapListener'):
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
        print(str(self.name)+" Object Initialized")

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
    def __init__(self, parent, stop_event, rate=20, name='Queued Message Handler', verbose=False):
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
        self.verbose = verbose
        print(str(self.name)+" Object Initialized")

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

            # Check for listen command
            if self.listen_command_heard(msg):
                try:
                    # Calculate new position from gradient
                    delta, new_eef_pos, hand_pos = self.position_gradient(msg)
                    if self.verbose: print("hand delta: "+str(delta[0:3]))


                    # IK stuff.... convert pose to joint state
                    #joint_states = self.parent.my_chain.inverse_kinematics(new_eef_pos)
                    #j_deg = []
                    #for j_rad in joint_states:
                    #    j_deg.append(str(np.rad2deg(j_rad)))
                    #msg = ",".join(j_deg)

                    # Map displacement to joint angles for gimbal box servos
                    #joint_states = self.set_head_angle(delta)
                    #j_deg = []
                    #for j_rad in joint_states:
                    #    j_deg.append(str(j_rad))
                    #msg = ",".join(j_deg)

                    # Keep hand in center of vision
                    joint_states, delta_angles = self.set_min_disp_head(hand_pos)
                    print("d_pan:" + str(delta_angles[0]) + "| d_tilt:" + str(delta_angles[1]))
                    print("pan:"+str(joint_states[0])+"| tilt:"+str(joint_states[1]))
                    j_deg = []
                    for j_rad in joint_states:
                       j_deg.append(str(j_rad))
                    msg = ",".join(j_deg)

                    # Send to Arduino communicator queue
                    self.parent.arduino_queue.put(msg)
                except:
                    if self.verbose: print(str(self.name)+": Error converting goal state")
        else:
            # Shouldn't expect other messages for now...
            pass

    def set_head_angle(self, displacement):
        # Calculates the angle of the gimbal box servos based on the displacement of the hand
        # When hand moves left/right, x-axis rotation scales 1 deg for 10 mm movement
        # Assumes the Leap is on the table
        gain = 1

        # gimbal box origin 35 and 112
        # Just output x and y movements, height not passed
        joint_states = (np.array([displacement[0]+35, displacement[2]+112]))*gain
        #joint_states = (np.array([displacement[0], displacement[2])+90)*gain
        if self.verbose: print("New angles: " + str(joint_states[0:2]))
        return joint_states

    def set_min_disp_head(self, pos):
        # Calculates the angle of the gimbal box servos based on the displacement of the hand
        # from the center field of view on the leap. When hand moves left/right, x-axis rotation
        # Assumes the Leap is attached to the head
        gain = 0.5

        # Calculate the arctan from x and z to get pan and tilt angles
        d_pan = np.rad2deg(math.atan2(float(pos[0]), float(pos[1])))*gain
        d_tilt = np.rad2deg(math.atan2(float(pos[2]), float(pos[1])))*gain

        # Update delta to current eef position
        self.parent.cur_eff_pos[0] += d_pan
        self.parent.cur_eff_pos[1] += d_tilt

        return self.parent.cur_eff_pos, (d_pan, d_tilt)

    def position_gradient(self, data, use_rot=False):
        # Calculates the delta of position and orientation
        # The hand operates like a joystick, move hand towards direction and the light will move there
        hand_pos, finger_pos = self.get_hand_pos(data, True)
        delta = list(np.subtract(self.parent.leap_origin, hand_pos[0]))

        # Use hand delta scaled to 10mm as max step movement in any direction
        for i, d in enumerate(delta):
            lim = 45
            if d > lim:
                delta[i] = lim
            if d < -lim:
                delta[i] = -lim

        # Get current pose of end effector and calculate new end effector pos
        eff_cur_pos = None
        #if self.parent.cur_eff_pos:
        #    eff_cur_pos = self.parent.my_chain.forward_kinematics([0] * self.parent.n_joints)[:3, 3]  # Get current joint state
        #    self.parent.goal_eff_pos = list(np.add(eff_cur_pos, delta[0:3]))

        return delta, eff_cur_pos, hand_pos[0]

    def listen_command_heard(self, data):
        """ Function that analyzes frame data and determines if the "listen" command is active

        :param: data (Leap.Frame)
        :return: result (bool) True or False whether listening command was heard
        """

        result = False
        # Get hand position and orientation from frame
        hand_pos, finger_pos = self.get_hand_pos(data)

        if len(hand_pos) >= 1:
            # Only one hand
            palm_x, palm_y, palm_z, palm_roll, palm_pitch, palm_yaw = hand_pos[0]
            palm = np.array([palm_x, palm_y, palm_z])

            # Calculate Euclidean distance between palm and fingertips
            thumb, index, middle, ring, pinky = finger_pos[0]
            palm_to_thumb = np.linalg.norm(palm - thumb)
            palm_to_index = np.linalg.norm(palm - index)
            palm_to_middle = np.linalg.norm(palm - middle)
            palm_to_ring = np.linalg.norm(palm - ring)
            palm_to_pinky = np.linalg.norm(palm - pinky)
            thumb_to_pinky = np.linalg.norm(thumb - pinky)

            #if self.verbose: print('Euclidean distance: '+str(palm_to_thumb), str(palm_to_index), str(palm_to_middle), str(palm_to_ring),
            #     str(palm_to_pinky))

            # ==== Listen command below ======
            if (palm_to_pinky > self.parent.finger_threshold['Pinky'] and
                palm_to_ring < self.parent.finger_threshold['Ring'] and
                palm_to_middle < self.parent.finger_threshold['Middle'] and
                palm_to_index < self.parent.finger_threshold['Index'] and
                palm_to_thumb < self.parent.finger_threshold['Thumb']
            ):
                if self.verbose: print("=============Listen!==================")
                result = True
                if self.parent.refresh_origin:
                    if self.verbose: print("origin refreshed")
                    self.parent.leap_origin = hand_pos[0]
                    self.parent.refresh_origin = False
            else:
                self.parent.refresh_origin = True

            # ================================

            return result

    def get_fingertip_pos(self, hand):
        """Get fingertip positions for each finger

        :param hand: (Hand) Leap object containing finger data
        :return: finger_data (list) A list containing the position of each fingertip
        """
        finger_data = []
        for finger in hand.fingers:
            # Getting just the finger tip of the 4th (distal) bone per finger
            bone = finger.bone(3)
            #if self.verbose: print("  %s finger, tip: %s" % (self.parent.finger_names[finger.type], bone.next_joint))
            finger_data.append(np.array([bone.next_joint.x, bone.next_joint.y, bone.next_joint.z]))

        return finger_data

    def check_palm_state(self, roll):
        # state whether palm is facing up or down
        if roll < 30 and roll > -30:
            return "palm down"
        elif roll > 100 and roll < 180 or roll < -90 and roll > -180:
            return "palm up"
        else:
            return "none"

    def get_hand_pos(self, frame, verbose=False):
        """ Function that collects the transform of the hand palm with position and orientation

        :param frame: Leap.Frame object
        :param verbose:
        :return: (list) list of lists with pos and orient data elements, for each hand detected
        """
        hand_data = []
        finger_data = []
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"
            if verbose: print("grab stength: "+str(hand.grab_strength))

            # Get hand position
            x = hand.palm_position.x
            y = hand.palm_position.y
            z = hand.palm_position.z

            if verbose: print("  %s, id %d, position: %s" % (handType, hand.id, hand.palm_position))

            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction

            # Calculate the hand's pitch, roll, and yaw angles
            pitch = direction.pitch * Leap.RAD_TO_DEG
            roll = normal.roll * Leap.RAD_TO_DEG
            yaw = direction.yaw * Leap.RAD_TO_DEG

            if verbose: print("  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (pitch, roll, yaw))
            hand_data.append([x, y, z, roll, pitch, yaw])

            finger_data.append(self.get_fingertip_pos(hand))

        return hand_data, finger_data


class Flexi(LeapListener):
    def __init__(self, verbose=False):
        """This Flexi object initializes a leap controller device and constantly listens to presence of data

        """
        self.verbose = verbose
        self.controller = Leap.Controller()
        self.msg_queue = queue.Queue()  # main message handler queue for handling leap data
        self.arduino_queue = queue.Queue()  # separate queue for sending data to arduino

        self.finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        self.bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']

        # Threshold values for euclidean distances between fingertips and palm
        self.finger_threshold = {'Thumb': 70, 'Index': 70, 'Middle': 70, 'Ring': 70, 'Pinky': 80}

        # Set up robot kinematics
        #self.my_chain = flexi_chain
        #self.n_joints = len(self.my_chain.links)
        self.cur_eff_pos = np.array([35, 112, 0])  # x, z, y , origin for gimbal box
        #self.cur_eef_pos = self.my_chain.forward_kinematics([0] * self.n_joints)[:3, 3]  # Get current joint state
        #print(self.cur_eef_pos)
        self.goal_eef_pos = None
        #self.joint_state = self.my_chain.inverse_kinematics(self.cur_eef_pos)
        #print(self.joint_state)
        self.leap_origin = []
        self.refresh_origin = True
        # [90, 90, 90, 90, 90, 90, 90]  # Initial joint states

        # self.my_chain = ikpy.chain.Chain.from_urdf_file("../resources/poppy_ergo.URDF")  # Add kinematic chain for light
        # TO-DO : self.my_chain = ikpy.chain.Chain.from_urdf_file("../resources/flexi.URDF")  # <-- eventually!

        # Add threads for parallel processes
        self.arduino_comm_event = threading.Event()
        self.arduino_comm = ArduinoComm(self, self.arduino_comm_event, port='COM6', verbose=self.verbose)
        self.arduino_comm.start()

        self.leap_listener_event = threading.Event()
        self.leap_listener = LeapListener(self, self.leap_listener_event)
        self.leap_listener.start()

        self.queue_msg_handler_event = threading.Event()
        self.queue_msg_handler = QueuedMessageHandler(self, self.queue_msg_handler_event, verbose=self.verbose)
        self.queue_msg_handler.start()

    def destroy(self):
        print("Stopping " + str(self.arduino_comm.name))
        self.arduino_comm_event.set()
        print("Stopping "+str(self.leap_listener.name))
        self.leap_listener_event.set()
        print("Stopping "+str(self.queue_msg_handler.name))
        self.queue_msg_handler_event.set()

        # missing something to delete object instance and save memory, will figure out later


def main():

    flexi = Flexi(True)
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

