from controller import Robot, Motor, Gyro, GPS, Camera, Compass, Keyboard, LED, InertialUnit, DistanceSensor
import math
import cv2
import numpy as np


SIGN = lambda x: int(x>0) - int(x<0)
CLAMP = lambda value, low, high : min(high, max(value, low))




class Drone:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)
        # front_left_led = robot.getDevice("front left led");
        # front_right_led = robot.getDevice("front right led");
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        # self.ds_front=self.robot.getDevice("ds_front")
        # self.ds_front.enable(self.timestep)
        # self.ds_right=self.robot.getDevice("ds_right")
        # self.ds_right.enable(self.timestep)
        # self.ds_left=self.robot.getDevice("ds_left")
        # self.ds_left.enable(self.timestep)
        # self.ds_bottom=self.robot.getDevice("ds_bottom")
        # self.ds_bottom.enable(self.timestep)
       
        # keyboard = Keyboard();
        # keyboard.enable(timestep)
        self.camera_roll_motor = self.robot.getDevice('camera roll')
        self.camera_pitch_motor = self.robot.getDevice('camera pitch')

        self.front_left_motor = self.robot.getDevice("front left propeller")
        self.front_right_motor = self.robot.getDevice("front right propeller")
        self.rear_left_motor = self.robot.getDevice("rear left propeller")
        self.rear_right_motor = self.robot.getDevice("rear right propeller")
        self.motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]

        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        self.k_vertical_thrust = 68.5
        self.k_vertical_offset = 0.6 
        self.k_vertical_p = 3.0
        self.k_roll_p = 50.0
        self.k_pitch_p = 30.0

        self.target_altitude = 1.0

    def move(self,command,intensity):
        roll = self.imu.getRollPitchYaw()[0] #+ math.pi / 2.0
        pitch = self.imu.getRollPitchYaw()[1]
        altitude = self.gps.getValues()[2]
        roll_acceleration = self.gyro.getValues()[0]
        pitch_acceleration = self.gyro.getValues()[1]

        # led_state = int(time) % 2
        # front_left_led.set(led_state)
        # front_right_led.set(int(not led_state))
        
        # self.camera_roll_motor.setPosition(-0.115 * roll_acceleration)
        # self.camera_pitch_motor.setPosition(-0.1 * pitch_acceleration)
        self.camera_roll_motor.setPosition(0)
        self.camera_pitch_motor.setPosition(math.pi/2)
        
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0

        if(command=='forward'):
            pitch_disturbance = -intensity  #2.0
        elif(command=='backward'):
            pitch_disturbance = intensity #-2.0
        elif(command=='right'):
            yaw_disturbance = -intensity  #1.3
        elif(command=='left'):
            yaw_disturbance = intensity  #-1.3
        elif(command=='sRight'):
            roll_disturbance = -intensity  #-1.0
        elif(command=='sLeft'):
            roll_disturbance = intensity  #1.0
        elif(command=='up'):
            self.target_altitude += intensity  #0.05
        elif(command=='down'):
            self.target_altitude -= intensity  #0.05

        roll_input = self.k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
        pitch_input = self.k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = CLAMP(self.target_altitude - altitude + self.k_vertical_offset, -1.0, 1.0)
        vertical_input = self.k_vertical_p * pow(clamped_difference_altitude, 3.0)

        
        front_left_motor_input = self.k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
        front_right_motor_input = self.k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
        rear_left_motor_input = self.k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
        rear_right_motor_input = self.k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
        self.front_left_motor.setVelocity(front_left_motor_input)
        self.front_right_motor.setVelocity(-front_right_motor_input)
        self.rear_left_motor.setVelocity(-rear_left_motor_input)
        self.rear_right_motor.setVelocity(rear_right_motor_input)
        
    def get_image(self):
        image=self.camera.getImageArray()
        image=np.array(image,dtype=np.uint8)
        image = np.flip(image,axis=2)
        image = cv2.rotate(image, cv2.cv2.ROTATE_90_CLOCKWISE)
        return image
        






drone=Drone()


i=0

while drone.robot.step(drone.timestep) != -1:
    i=i+1
    if(i<100):
        drone.move('up',0.1)
    
    
    else:    
        drone.move('forward',1.5)
    # print(drone.ds_front.getValue())
    # print(drone.imu.getRollPitchYaw())
    # i+=1
    if i%300==0:
        image=drone.get_image()
        
        
        cv2.imshow("img",image)
        cv2.waitKey(1)
        cv2.destroyAllWindows()

    
    # image=drone.get_image()
    
   
    # cv2.imshow("img",image)
    # if cv2.waitKey() & 0xFF == ord('q'):
        # break

cv2.destroyAllWindows()

    


   


