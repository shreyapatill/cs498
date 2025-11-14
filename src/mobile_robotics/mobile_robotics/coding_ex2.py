# Student name: 

import math
import numpy as np
from numpy import linalg as LA
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Accel
from tf2_ros import TransformBroadcaster

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import time
from mobile_robotics.utils import quaternion_from_euler, lonlat2xyz, quat2euler


class ExtendedKalmanFilter(Node):

    
    def __init__(self):
        super().__init__('ExtendedKalmanFilter')
        
        
        #array to save the sensor measurements from the rosbag file
        #measure = [p, q, r, fx, fy, fz, x, y, z, vx, vy, vz] 
        self.measure = np.zeros(12)
        
        #Initialization of the variables used to generate the plots.
        self.PHI = []  
        self.PSI = []
        self.THETA = []
        self.P_R = []
        self.P_R1 = []
        self.P_R2 = []
        self.Pos = []
        self.Vel = []
        self.Quater = []
        self.measure_PosX = []
        self.measure_PosY = []
        self.measure_PosZ = []
        self.P_angular = []
        self.Q_angular = []
        self.R_angular = []
        self.P_raw_angular = []
        self.Q_raw_angular = []
        self.R_raw_angular = []
        self.Bias =[]
        
        self.POS_X = []
        self.POS_Y = []
        
        
        #Initialization of the variables used in the EKF
        
        # initial bias values, these are gyroscope and accelerometer biases
        self.bp= 0.0
        self.bq= 0.0
        self.br= 0.0
        self.bfx = 0.0
        self.bfy = 0.0
        self.bfz = 0.0
        # initial rotation
        self.q2, self.q3, self.q4, self.q1 = quaternion_from_euler(0.0, 0.0, np.pi/2) #[qx,qy,qz,qw]

        #initialize the state vector [x y z vx vy vz          quat    gyro-bias accl-bias]
        self.xhat = np.array([[0, 0, 0, 0, 0, 0, self.q1, self.q2, self.q3, self.q4, self.bp, self.bq, self.br, self.bfx, self.bfy, self.bfz]]).T

        self.rgps=np.array([-0.15, 0 ,0]) #This is the location of the GPS wrt CG, this is very important
        
        #noise params process noise (my gift to you :))
        self.Q = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.5, 0.5, 0.5, 0.5, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
        #measurement noise
        #GPS position and velocity
        self.R = np.diag([10, 10, 10, 2, 2, 2])
        
       
        #Initialize P, the covariance matrix
        self.P = np.diag([30, 30, 30, 3, 3, 3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.Pdot=self.P*0.0
        
        self.time = []
        self.loop_t = 0

        # You might find these blocks useful when assembling the transition matrices
        self.Z = np.zeros((3,3))
        self.I = np.eye(3,3)
        self.Z34 = np.zeros((3,4))
        self.Z43 = np.zeros((4,3))
        self.Z36 = np.zeros((3,6))

        self.lat = 0
        self.lon = 0
        self.lat0 = 0
        self.lon0 = 0
        self.flag_lat = False
        self.flag_lon = False
        self.dt = 0.0125 #set sample time

        # Ros subscribers and publishers
        self.subscription_imu = self.create_subscription(Imu, 'terrasentia/imu', self.callback_imu, 10)
        self.subscription_gps_lat = self.create_subscription(Float32, 'gps_latitude', self.callback_gps_lat, 10)
        self.subscription_gps_lon = self.create_subscription(Float32, 'gps_longitude', self.callback_gps_lon, 10)
        self.subscription_gps_speed_north = self.create_subscription(Float32, 'gps_speed_north', self.callback_gps_speed_north, 10)
        self.subscription_gps_speend_east = self.create_subscription(Float32, 'gps_speed_east', self.callback_gps_speed_east, 10)
        
        self.timer_ekf = self.create_timer(self.dt, self.ekf_callback)
        self.timer_plot = self.create_timer(1, self.plot_data_callback)

    
    def callback_imu(self,msg):
        #measurement vector = [p, q, r, fx, fy, fz, x, y, z, vx, vy, vz]
        # In practice, the IMU measurements should be filtered. In this coding exercise, we are just going to clip
        # the values of velocity and acceleration to keep them in physically possible intervals.
        self.measure[0] = np.clip(msg.angular_velocity.x,-5,5) #(-5,5)
        self.measure[1] = np.clip(msg.angular_velocity.y,-5,5) #(-5,5)
        self.measure[2] = np.clip(msg.angular_velocity.z,-5,5) #(-5,5)
        self.measure[3] = np.clip(msg.linear_acceleration.x,-6,6) #(-6,6)
        self.measure[4] = np.clip(msg.linear_acceleration.y,-6,6) #(-6,6)
        self.measure[5] = np.clip(msg.linear_acceleration.z,-16,-4) #(-16,-4) 
 
    def callback_gps_lat(self, msg):
        self.lat = msg.data
        if (self.flag_lat == False): #just a trick to recover the initial value of latitude
            self.lat0 = msg.data
            self.flag_lat = True
        
        if (self.flag_lat and self.flag_lon): 
            x, y = lonlat2xyz(self.lat, self.lon, self.lat0, self.lon0) # convert latitude and longitude to x and y coordinates
            self.measure[6] = x
            self.measure[7] = y
            self.measure[8] = 0.0 

    
    def callback_gps_lon(self, msg):
        self.lon = msg.data
        if (self.flag_lon == False): #just a trick to recover the initial value of longitude
            self.lon0 = msg.data
            self.flag_lon = True    
    
    def callback_gps_speed_east(self, msg):
        self.measure[9] = msg.data # vx (velocity east)
        self.measure[11] = 0.0 # vz

    def callback_gps_speed_north(self, msg):
        self.measure[10] = msg.data # vy (velocity north)

   
    def ekf_callback(self):
        #print("iteration:  ",self.loop_t)
        if (self.flag_lat and self.flag_lon):  #Trick  to sincronize rosbag with EKF
            self.ekf_function()
        else:
            print("Play the rosbag file...")

    
    
    def ekf_function(self):
        
        # Adjusting angular velocities and acceleration with the corresponding bias
        self.p = (self.measure[0]-self.xhat[10,0])
        self.q = (self.measure[1]-self.xhat[11,0])
        self.r = self.measure[2]-self.xhat[12,0]
        self.fx = (self.measure[3]-self.xhat[13,0])
        self.fy = (self.measure[4]-self.xhat[14,0])
        self.fz = -self.measure[5]-self.xhat[15,0]
        
        # Get the current quaternion values from the state vector
        # Remember again the state vector [x y z vx vy vz q1 q2 q3 q4 bp bq br bx by bz]
        self.quat = np.array([[self.xhat[6,0], self.xhat[7,0], self.xhat[8,0], self.xhat[9,0]]]).T
    
        self.q1 = self.quat[0,0]
        self.q2 = self.quat[1,0]
        self.q3 = self.quat[2,0]
        self.q4 = self.quat[3,0]
                
        # Rotation matrix: body to inertial frame
        self.R_bi = np.array([[pow(self.q1,2)+pow(self.q2,2)-pow(self.q3,2)-pow(self.q4,2), 2*(self.q2*self.q3-self.q1*self.q4), 2*(self.q2*self.q4+self.q1*self.q3)],
                          [2*(self.q2*self.q3+self.q1*self.q4), pow(self.q1,2)-pow(self.q2,2)+pow(self.q3,2)-pow(self.q4,2), 2*(self.q3*self.q4-self.q1*self.q2)],
                          [2*(self.q2*self.q4-self.q1*self.q3), 2*(self.q3*self.q4+self.q1*self.q2), pow(self.q1,2)-pow(self.q2,2)-pow(self.q3,2)+pow(self.q4,2)]])
        
            
        #Prediction step
        #First write out all the dots for all the states, e.g. pxdot, pydot, q1dot etc

        # Position derivatives (equation 101: pdot = v)
        pxdot = self.xhat[3,0]
        pydot = self.xhat[4,0]
        pzdot = self.xhat[5,0]

        # Velocity derivatives (equation 102: vdot = R_bi @ a + gravity)
        accel_body = np.array([[self.fx, self.fy, self.fz]]).T
        accel_inertial = self.R_bi @ accel_body
        vxdot = accel_inertial[0,0]
        vydot = accel_inertial[1,0]
        vzdot = accel_inertial[2,0] - 9.801  # Add gravity in z direction

        # Quaternion derivatives (equation 103: qdot = 0.5 * Omega(w) @ q)
        # Define angular velocities P, Q, R (bias-corrected)
        P = self.p
        Q = self.q
        R = self.r

        # Omega matrix (equation 107)
        Omega = np.array([[0, -P, -Q, -R],
                         [P,  0,  R, -Q],
                         [Q, -R,  0,  P],
                         [R,  Q, -P,  0]])

        qdot = 0.5 * Omega @ self.quat
        q1dot = qdot[0,0]
        q2dot = qdot[1,0]
        q3dot = qdot[2,0]
        q4dot = qdot[3,0]

        # Bias derivatives (equations 104-105: bias derivatives are zero)
        bpdot = 0
        bqdot = 0
        brdot = 0
        bfxdot = 0
        bfydot = 0
        bfzdot = 0
        
        #Now integrate Euler Integration for Process Updates and Covariance Updates
        # Euler works fine
        # Remember again the state vector [x y z vx vy vz q1 q2 q3 q4 bp bq br bx by bz]
        self.xhat[0,0] = self.xhat[0,0] + self.dt*pxdot
        self.xhat[1,0] = self.xhat[1,0] + self.dt*pydot
        self.xhat[2,0] = self.xhat[2,0] + self.dt*pzdot
        self.xhat[3,0] = self.xhat[3,0] + self.dt*vxdot
        self.xhat[4,0] = self.xhat[4,0] + self.dt*vydot
        self.xhat[5,0] = self.xhat[5,0] + self.dt*vzdot
        self.xhat[6,0] = self.xhat[6,0] + self.dt*q1dot
        self.xhat[7,0] = self.xhat[7,0] + self.dt*q2dot
        self.xhat[8,0] = self.xhat[8,0] + self.dt*q3dot
        self.xhat[9,0] = self.xhat[9,0] + self.dt*q4dot

        print("x ekf: ", self.xhat[0,0])
        print("y ekf: ", self.xhat[1,0])
        print("z ekf: ", self.xhat[2,0])
        
        # Extract and normalize the quat
        self.quat = np.array([[self.xhat[6,0], self.xhat[7,0], self.xhat[8,0], self.xhat[9,0]]]).T
        # Normalize quat (equation: q <- q/norm(q))
        self.quat = self.quat / LA.norm(self.quat)
        
        #re-assign quat
        self.xhat[6,0] = self.quat[0,0]
        self.xhat[7,0] = self.quat[1,0]
        self.xhat[8,0] = self.quat[2,0]
        self.xhat[9,0] = self.quat[3,0]
        
                
        # Now write out all the partials to compute the transition matrix Phi
        # Update quaternion values after normalization
        q1 = self.xhat[6,0]
        q2 = self.xhat[7,0]
        q3 = self.xhat[8,0]
        q4 = self.xhat[9,0]

        #delV/delQ (Fvq) - equation 118
        ax = self.fx
        ay = self.fy
        az = self.fz

        Fvq = 2*np.array([[q1*ax + q4*ay - q3*az,  q2*ax + q3*ay + q4*az,  -q3*ax + q2*ay + q1*az,  -q4*ax - q1*ay + q2*az],
                          [q4*ax + q1*ay - q2*az,  q3*ax - q2*ay - q1*az,   q2*ax + q3*ay + q4*az,   q1*ax - q4*ay + q3*az],
                          [-q3*ax + q2*ay + q1*az, q4*ax + q1*ay - q2*az,  -q1*ax + q4*ay - q3*az,   q2*ax + q3*ay + q4*az]])

        #delV/del_abias (Fvba) - equation 114
        Fvb = -self.R_bi

        #delQ/delQ (Fqq) - equation 115: -(1/2)*Omega(omega)
        Fqq = -0.5 * Omega

        #delQ/del_gyrobias (Fqbω) - equation 120
        Fqb = 0.5 * np.array([[q2,  q3,  q4],
                              [-q1, q4, -q3],
                              [-q4, -q1, q2],
                              [q3, -q2, -q1]])
        # Now assemble the Transition matrix A (equation 121)
        # A is 16x16: state vector is [x y z vx vy vz q1 q2 q3 q4 bp bq br bfx bfy bfz]
        # Row structure:
        # rows 0-2 (pos):   Z(3x3) I(3x3) Z(3x4) Z(3x3) Z(3x3)
        # rows 3-5 (vel):   Z(3x3) Z(3x3) Fvq    Z(3x3) Fvb
        # rows 6-9 (quat):  Z(4x3) Z(4x3) Fqq    Fqb    Z(4x3)
        # rows 10-12 (bω):  Z(3x3) Z(3x3) Z(3x4) Z(3x3) Z(3x3)
        # rows 13-15 (ba):  Z(3x3) Z(3x3) Z(3x4) Z(3x3) Z(3x3)

        A = np.zeros((16,16))
        # Position row: pdot = v
        A[0:3, 3:6] = self.I
        # Velocity row: vdot depends on quaternion and accel bias
        A[3:6, 6:10] = Fvq
        A[3:6, 13:16] = Fvb
        # Quaternion row: qdot depends on quaternion and gyro bias
        A[6:10, 6:10] = Fqq
        A[6:10, 10:13] = Fqb

        #Propagate the error covariance matrix, I suggest using the continuous integration since Q, R are not discretized
        #Pdot = A@P+P@A.transpose() + Q
        #P = P +Pdot*dt
        Pdot = A @ self.P + self.P @ A.T + self.Q
        self.P = self.P + Pdot * self.dt
        
        #Correction step
        #Get measurements 3 positions and 3 velocities from GPS
        self.z = np.array([[self.measure[6], self.measure[7], self.measure[8], self.measure[9], self.measure[10], self.measure[11]]]).T #x y z vx vy vz
    
        #Write out the measurement matrix linearization to get H
        # Measurement model accounts for GPS offset from CG
        # pCG = pGPS - R_bi * rGPS (equation 122)
        # vCG = vGPS - R_bi * (omega x rGPS) (equation 123)

        # Get rgps first element (note: rgps = [-0.15, 0, 0] from line 70)
        rgps_x = self.rgps[0]

        #del P/del q (Hxq) - equation 124
        # Since rgps has only x component, we only use first column of R_bi
        Hxq = np.array([[-rgps_x*2*q2,  rgps_x*2*q3,  rgps_x*2*q4, -rgps_x*2*q1],
                        [-rgps_x*2*q3, -rgps_x*2*q2,  rgps_x*2*q1,  rgps_x*2*q4],
                        [-rgps_x*2*q4,  rgps_x*2*q1, -rgps_x*2*q2,  rgps_x*2*q3]])

        # del v/del q (Hvq) - equation 125
        # Hvq accounts for derivative of R_bi*(omega x rgps) w.r.t. q
        Hvq = np.array([[rgps_x*2*(q3*Q + q4*R), rgps_x*2*(q1*Q - q2*R), rgps_x*2*(q2*Q + q1*R), -rgps_x*2*q2],
                        [rgps_x*2*(-q2*Q - q1*R), rgps_x*2*(q4*Q - q3*R), rgps_x*2*(q3*Q + q4*R), -rgps_x*2*q3],
                        [rgps_x*2*(q1*Q - q2*R), rgps_x*2*(-q3*Q - q4*R), rgps_x*2*(q4*Q - q3*R), -rgps_x*2*q4]])

        # Assemble H (equation 126)
        # H is 6x16: measurements are [x y z vx vy vz]
        # H = [ I(3x3)  Z(3x3)  Hxq(3x4)  Z(3x6) ]
        #     [ Z(3x3)  I(3x3)  Hvq(3x4)  Z(3x6) ]
        H = np.zeros((6,16))
        H[0:3, 0:3] = self.I    # position measurements
        H[0:3, 6:10] = Hxq       # position depends on quaternion
        H[3:6, 3:6] = self.I     # velocity measurements
        H[3:6, 6:10] = Hvq       # velocity depends on quaternion

        #Compute Kalman gain
        # L = P @ H^T @ (H @ P @ H^T + R)^-1
        S = H @ self.P @ H.T + self.R  # Innovation covariance
        L = self.P @ H.T @ LA.inv(S)    # Kalman gain

        #Perform xhat correction: xhat = xhat + L@(z-H@xhat)
        innovation = self.z - H @ self.xhat
        self.xhat = self.xhat + L @ innovation

        #propagate error covariance approximation P = (I - L@H)@P
        self.P = (np.eye(16,16) - L @ H) @ self.P

        #Now let us do some book-keeping 
        # Get some Euler angles
        
        phi, theta, psi = quat2euler(self.quat.T)

        self.PHI.append(phi*180/math.pi)
        self.THETA.append(theta*180/math.pi)
        self.PSI.append(psi*180/math.pi)
    
          
        # Saving data for the plots. Uncomment the 4 lines below once you have finished the ekf function

        DP = np.diag(self.P)
        self.P_R.append(DP[0:3])
        self.P_R1.append(DP[3:6])
        self.P_R2.append(DP[6:10])
        self.Pos.append(self.xhat[0:3].T[0])
        self.POS_X.append(self.xhat[0,0])
        self.POS_Y.append(self.xhat[1,0])
        self.Vel.append(self.xhat[3:6].T[0])
        self.Quater.append(self.xhat[6:10].T[0])
        self.Bias.append(self.xhat[10:16].T[0])
        B = self.measure[6:9].T
        self.measure_PosX.append(B[0])
        self.measure_PosY.append(B[1])
        self.measure_PosZ.append(B[2])

        self.P_angular.append(self.p)
        self.Q_angular.append(self.q)
        self.R_angular.append(self.r)

        self.loop_t += 1
        self.time.append(self.loop_t*self.dt)

    def plot_data_callback(self):

        # plt.figure(1)
        # plt.clf()
        # plt.plot(self.time,self.PHI,'b', self.time, self.THETA, 'g', self.time,self.PSI, 'r')
        # plt.legend(['phi','theta','psi'])
        # plt.title('Phi, Theta, Psi [deg]')

        plt.figure(2)
        plt.clf()
        plt.plot(self.measure_PosX, self.measure_PosY, self.POS_X, self.POS_Y)
        plt.title('xy trajectory')
        plt.legend(['GPS','EKF'])

        # plt.figure(3)
        # plt.clf()
        # plt.plot(self.time,self.P_R)
        # plt.title('Covariance of Position')
        # plt.legend(['px','py','pz'])
        # plt.figure(4)
        # plt.clf()
        # plt.plot(self.time,self.P_R1)
        # plt.legend(['pxdot','pydot','pzdot'])
        # plt.title('Covariance of Velocities')
        # plt.figure(5)
        # plt.clf()
        # plt.plot(self.time,self.P_R2)
        # plt.title('Covariance of Quaternions')
        # plt.figure(6)
        # plt.clf()
        # plt.plot(self.time,self.Pos,self.time,self.measure_PosX,'r:', self.time,self.measure_PosY,'r:', self.time,self.measure_PosZ,'r:')
        # plt.legend(['X_ekf', 'Y_ekf', 'Z_ekf','Xgps','Ygps','Z_0'])
        # plt.title('Position')
        # plt.figure(7)
        # plt.clf()
        # plt.plot(self.time,self.Vel)
        # plt.title('vel x y z')
        # plt.figure(8)
        # plt.clf()
        # plt.plot(self.time,self.Quater)
        # plt.title('Quat')
        # plt.figure(9)
        # plt.clf()
        # plt.plot(self.time,self.P_angular,self.time,self.Q_angular,self.time,self.R_angular)
        # plt.title('OMEGA with Bias')
        # plt.legend(['p','q','r'])

        # plt.figure(10)
        # plt.clf()
        # plt.plot(self.time,self.Bias)
        # plt.title('Gyroscope and accelerometer Bias')
        # plt.legend(['bp','bq','br','bfx','bfy','bfz'])
                
        plt.ion()
        plt.show()
        plt.pause(0.0001)
        

def main(args=None):
    rclpy.init(args=args)

    ekf_node = ExtendedKalmanFilter()
    
    rclpy.spin(ekf_node)

   
    ekf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
