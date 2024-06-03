from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface 
from rtde_io import RTDEIOInterface as RTDEIO
import robotiq_gripper
from spnav import spnav_open, spnav_poll_event, spnav_close, SpnavMotionEvent, SpnavButtonEvent
from threading import Thread, Event
from collections import defaultdict
import numpy as np
import time

class Spacemouse(Thread):
    def __init__(self, max_value=500, deadzone=(0,0,0,0,0,0), dtype=np.float32):
        """
        Continuously listen to 3D connection space naviagtor events
        and update the latest state.

        max_value: {300, 500} 300 for wired version and 500 for wireless
        deadzone: [0,1], number or tuple, axis with value lower than this value will stay at 0
        
        front
        z
        ^   _
        |  (O) space mouse
        |
        *----->x right
        y
        """
        if np.issubdtype(type(deadzone), np.number):
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()

        super().__init__()
        self.stop_event = Event()
        self.max_value = max_value
        self.dtype = dtype
        self.deadzone = deadzone
        self.motion_event = SpnavMotionEvent([0,0,0], [0,0,0], 0)
        self.button_state = defaultdict(lambda: False)
        self.tx_zup_spnav = np.array([
            [0,0,-1],
            [1,0,0],
            [0,1,0]
        ], dtype=dtype)

    def get_motion_state(self): #this method gets the movement of the mouse 
        me = self.motion_event
        state = np.array(me.translation + me.rotation, 
            dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state
    
    def get_motion_state_transformed(self): #transforms get_motion_state 
        """
        Return in right-handed coordinate
        z
        *------>y right
        |   _
        |  (O) space mouse
        v
        x
        back

        """
        state = self.get_motion_state()
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_spnav @ state[:3]
        tf_state[3:] = self.tx_zup_spnav @ state[3:]

        # Set values lesser than 0.3 to 0 for better control
        tf_state[np.abs(tf_state) < 0.3] = 0
        tf_state = tf_state * SCALE_FACTOR

        return tf_state

    def is_button_pressed(self, button_id):
        return self.button_state[button_id]

    def stop(self):
        self.stop_event.set()
        self.join()

    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def run(self):
        spnav_open()
        try:
            while not self.stop_event.is_set():
                event = spnav_poll_event()
                if isinstance(event, SpnavMotionEvent):
                    self.motion_event = event
                elif isinstance(event, SpnavButtonEvent):
                    self.button_state[event.bnum] = event.press
                else:
                    time.sleep(1/200)
        finally:
            spnav_close()

# Define robot parameters
ROBOT_HOST = "192.168.20.124"  # IP address of the robot controller
SCALE_FACTOR = 0.3 # Scale factor for velocity command

def main():
    sm = Spacemouse()
    sm.start()
    # Initialize RTDEControlInterface
    rtde_c = RTDEControlInterface(ROBOT_HOST)
    rtde_r = RTDEReceiveInterface(ROBOT_HOST)
    rtde_io = RTDEIO(ROBOT_HOST)
    
    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(ROBOT_HOST, 63352)
    print("Activating gripper...")
    gripper.activate()
    try:
        while True:
            if rtde_r.getRobotMode() == 7:
                # Read motion state from SpaceMouse
                motion_state = sm.get_motion_state_transformed()
                
                #send command to robot 
                rtde_c.speedL(motion_state, acceleration = 1.5, time = 0.1) #adjust the acceleration if required 

                #get TCP velocity of robot
                actual_velocity = rtde_r.getActualTCPSpeed()
                actual_velocity = [0 if abs(x) < 0.01 else x for x in actual_velocity] #filter out extremely small numbers
                print(actual_velocity)

                #get TCP pose of robot
                #actual_pose = rtde_r.getActualTCPPose()
                #print(actual_pose)
                if sm.is_button_pressed(0):
                    gripper.move(gripper.get_open_position(), 255, 255)
                
                if sm.is_button_pressed(1):
                    gripper.move(gripper.get_closed_position(), 255, 255)

                #wait awhile before proceeding 
                time.sleep(1/100)

            else:
                print("Robot is not ready.")
                time.sleep(1)  # Wait longer if robot is not ready

    except KeyboardInterrupt:
        # Handle graceful shutdown here
        print("Stopping robot")
        rtde_c.stopScript()
        sm.stop()

if __name__ == "__main__":
    main()
