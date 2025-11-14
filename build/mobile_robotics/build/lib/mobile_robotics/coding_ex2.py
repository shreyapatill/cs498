# Student name: 

import math
import numpy as np
from numpy import linalg as LA
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Accel
from tf2_ros import TransformBroadcaster

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
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
        
        # Initialize angular velocities (will be updated in ekf_function)
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        # Ros subscribers and publishers
        self.subscription_imu = self.create_subscription(Imu, 'terrasentia/imu', self.callback_imu, 10)
        self.subscription_gps_lat = self.create_subscription(Float32, 'gps_latitude', self.callback_gps_lat, 10)
        self.subscription_gps_lon = self.create_subscription(Float32, 'gps_longitude', self.callback_gps_lon, 10)
        self.subscription_gps_speed_north = self.create_subscription(Float32, 'gps_speed_north', self.callback_gps_speed_north, 10)
        self.subscription_gps_speend_east = self.create_subscription(Float32, 'gps_speed_east', self.callback_gps_speed_east, 10)
        
        # Odometry and path publishers
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.path_publisher = self.create_publisher(Path, 'odom/path', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer_ekf = self.create_timer(self.dt, self.ekf_callback)
        self.timer_plot = self.create_timer(1, self.plot_data_callback)
        self.timer_odom = self.create_timer(0.1, self.publish_odometry)  # Publish at 10Hz

    
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
        self.measure[9] = msg.data # vx (east speed)
        self.measure[11] = 0.0 # vz

    def callback_gps_speed_north(self, msg):
        self.measure[10] = msg.data # vy (north speed)

   
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
       
        pxdot = self.xhat[3,0]
        pydot = self.xhat[4,0]
        pzdot = self.xhat[5,0]
        
        # acceleration in body frame transformed to inertial frame
        a_body = np.array([[self.fx], [self.fy], [self.fz]])
        a_inertial = self.R_bi @ a_body
        
        vxdot = a_inertial[0,0]
        vydot = a_inertial[1,0]
        vzdot = a_inertial[2,0] + 9.801  # add gravity
        
        # quaternion derivative using Omega matrix
        Omega = np.array([[0.0, self.p, self.q, self.r],
                          [-self.p, 0.0, -self.r, self.q],
                          [-self.q, self.r, 0.0, -self.p],
                          [-self.r, -self.q, self.p, 0.0]])
        qdot = -0.5 * Omega @ self.quat
        
        #Now integrate Euler Integration for Process Updates and Covariance Updates
        # Euler works fine
        # Remember again the state vector [x y z vx vy vz q1 q2 q3 q4 bp bq br bx by bz]
        self.xhat[0,0] = self.xhat[0,0] + self.dt*pxdot
        self.xhat[1,0] = self.xhat[1,0] + self.dt*pydot
        self.xhat[2,0] = self.xhat[2,0] + self.dt*pzdot
        self.xhat[3,0] = self.xhat[3,0] + self.dt*vxdot
        self.xhat[4,0] = self.xhat[4,0] + self.dt*vydot
        self.xhat[5,0] = self.xhat[5,0] + self.dt*vzdot
        self.xhat[6,0] = self.xhat[6,0] + self.dt*qdot[0,0]
        self.xhat[7,0] = self.xhat[7,0] + self.dt*qdot[1,0]
        self.xhat[8,0] = self.xhat[8,0] + self.dt*qdot[2,0]
        self.xhat[9,0] = self.xhat[9,0] + self.dt*qdot[3,0]

        print("x ekf: ", self.xhat[0,0])
        print("y ekf: ", self.xhat[1,0])
        print("z ekf: ", self.xhat[2,0])
        
        # Extract and normalize the quat    
        self.quat = np.array([[self.xhat[6,0], self.xhat[7,0], self.xhat[8,0], self.xhat[9,0]]]).T
        # normalize quat
        quat_norm = LA.norm(self.quat)
        if quat_norm > 1e-10:
            self.quat = self.quat / quat_norm
        
        #re-assign quat
        self.xhat[6,0] = self.quat[0,0]
        self.xhat[7,0] = self.quat[1,0]
        self.xhat[8,0] = self.quat[2,0]
        self.xhat[9,0] = self.quat[3,0]
        
        # Update quaternion components after normalization
        self.q1 = self.quat[0,0]
        self.q2 = self.quat[1,0]
        self.q3 = self.quat[2,0]
        self.q4 = self.quat[3,0]
        
        # Update rotation matrix after quaternion normalization
        self.R_bi = np.array([[pow(self.q1,2)+pow(self.q2,2)-pow(self.q3,2)-pow(self.q4,2), 2*(self.q2*self.q3-self.q1*self.q4), 2*(self.q2*self.q4+self.q1*self.q3)],
                          [2*(self.q2*self.q3+self.q1*self.q4), pow(self.q1,2)-pow(self.q2,2)+pow(self.q3,2)-pow(self.q4,2), 2*(self.q3*self.q4-self.q1*self.q2)],
                          [2*(self.q2*self.q4-self.q1*self.q3), 2*(self.q3*self.q4+self.q1*self.q2), pow(self.q1,2)-pow(self.q2,2)-pow(self.q3,2)+pow(self.q4,2)]])
        
                
        # Now write out all the partials to compute the transition matrix Phi
        #delV/delQ - partial derivative of velocity w.r.t. quaternion (equation 118)
        ax, ay, az = self.fx, self.fy, self.fz
        q1, q2, q3, q4 = self.q1, self.q2, self.q3, self.q4
        
        Fvq = np.array([[2*(q1*ax + q4*ay - q3*az), 2*(q2*ax + q3*ay + q4*az), 2*(-q3*ax + q2*ay + q1*az), 2*(-q4*ax - q1*ay + q2*az)],
                        [2*(q4*ax + q1*ay - q2*az), 2*(q3*ax - q2*ay - q1*az), 2*(q2*ax + q3*ay + q4*az), 2*(q1*ax - q4*ay + q3*az)],
                        [2*(-q3*ax + q2*ay + q1*az), 2*(q4*ax + q1*ay - q2*az), 2*(-q1*ax + q4*ay - q3*az), 2*(q2*ax + q3*ay + q4*az)]])
        
        #delV/del_abias - partial derivative of velocity w.r.t. accel bias (equation 114)
        Fvb = -self.R_bi
        
        #delQ/delQ - partial derivative of quaternion w.r.t. quaternion (equation 115)
        Fqq = -0.5 * Omega
     
        #delQ/del_gyrobias - partial derivative of quaternion w.r.t. gyro bias (equation 120)
        Fqb = 0.5 * np.array([[q2, q3, q4],
                              [-q1, q4, -q3],
                              [-q4, -q1, q2],
                              [q3, -q2, -q1]])
        # Now assemble the Transition matrix A (equation 121)
        # State vector: [x, y, z, vx, vy, vz, q1, q2, q3, q4, bp, bq, br, bfx, bfy, bfz]
        A = np.zeros((16, 16))
        A[0:3, 3:6] = self.I  # position derivative w.r.t. velocity
        A[3:6, 6:10] = Fvq  # velocity derivative w.r.t. quaternion
        A[3:6, 13:16] = Fvb  # velocity derivative w.r.t. accel bias
        A[6:10, 6:10] = Fqq  # quaternion derivative w.r.t. quaternion
        A[6:10, 10:13] = Fqb  # quaternion derivative w.r.t. gyro bias
        
        #Propagate the error covariance matrix, I suggest using the continuous integration since Q, R are not discretized 
        #Pdot = A@P+P@A.transpose() + Q
        #P = P +Pdot*dt
        Pdot = A @ self.P + self.P @ A.T + self.Q
        self.P = self.P + Pdot * self.dt
        
        #Correction step
        #Get measurements 3 positions and 3 velocities from GPS
        self.z = np.array([[self.measure[6], self.measure[7], self.measure[8], self.measure[9], self.measure[10], self.measure[11]]]).T #x y z vx vy vz
    
        # Compute expected GPS measurement from state (CG) - equations 122-123
        # GPS position = CG position + R_b→I @ rGPS
        gps_pos_expected = self.xhat[0:3] + self.R_bi @ self.rgps.reshape(3,1)
        
        # GPS velocity = CG velocity + R_b→I @ (ω × rGPS)
        # cross product matrix for angular velocity
        omega_cross = np.array([[0.0, -self.r, self.q],
                                [self.r, 0.0, -self.p],
                                [-self.q, self.p, 0.0]])
        omega_cross_rgps = omega_cross @ self.rgps.reshape(3,1)
        gps_vel_expected = self.xhat[3:6] + self.R_bi @ omega_cross_rgps
        
        z_hat = np.vstack([gps_pos_expected, gps_vel_expected])  # expected measurement
    
        #Write out the measurement matrix linearization to get H
        # GPS offset from CG
        rgps1 = self.rgps[0]  # -0.15
        
        # del P/del q - partial derivative of position w.r.t. quaternion (equation 124)
        Hxq = np.array([[-rgps1*2*q1, -rgps1*2*q2, rgps1*2*q3, rgps1*2*q4],
                        [-rgps1*2*q4, -rgps1*2*q3, -rgps1*2*q2, -rgps1*2*q1],
                        [rgps1*2*q3, -rgps1*2*q4, rgps1*2*q1, -rgps1*2*q2]])
        
        # del v/del q - partial derivative of velocity w.r.t. quaternion (equation 125)
        Q = self.q  # angular velocity around y
        R = self.r  # angular velocity around z
        Hvq = np.array([[rgps1*2*q3*Q + rgps1*2*q4*R, rgps1*2*q4*Q - rgps1*2*q3*R, rgps1*2*q1*Q - rgps1*2*q2*R, rgps1*2*q2*Q + rgps1*2*q1*R],
                        [-rgps1*2*q2*Q - rgps1*2*q1*R, rgps1*2*q2*R - rgps1*2*q1*Q, rgps1*2*q4*Q - rgps1*2*q3*R, rgps1*2*q3*Q + rgps1*2*q4*R],
                        [rgps1*2*q1*Q - rgps1*2*q2*R, -rgps1*2*q2*Q - rgps1*2*q1*R, -rgps1*2*q3*Q - rgps1*2*q4*R, rgps1*2*q4*Q - rgps1*2*q3*R]])
        
        # Assemble H (equation 126)
        H = np.zeros((6, 16))
        H[0:3, 0:3] = self.I  # position w.r.t. position
        H[0:3, 6:10] = Hxq  # position w.r.t. quaternion
        H[3:6, 3:6] = self.I  # velocity w.r.t. velocity
        H[3:6, 6:10] = Hvq  # velocity w.r.t. quaternion

        #Compute Kalman gain
        # L = P @ H^T @ (H @ P @ H^T + R)^(-1)
        S = H @ self.P @ H.T + self.R  # innovation covariance
        L = self.P @ H.T @ LA.inv(S)  # Kalman gain
        
        #Perform xhat correction    xhat = xhat + L@(z-z_hat)
        self.xhat = self.xhat + L @ (self.z - z_hat)
        
        #propagate error covariance approximation P = (np.eye(16,16)-L@H)@P
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

        plt.figure(1)
        plt.clf()
        plt.plot(self.time,self.PHI,'b', self.time, self.THETA, 'g', self.time,self.PSI, 'r')
        plt.legend(['phi','theta','psi'])
        plt.title('Phi, Theta, Psi [deg]')
        plt.xlabel('Time [s]')
        plt.ylabel('Angle [deg]')
        plt.grid(True)

        plt.figure(2)
        plt.clf()
        plt.plot(self.measure_PosX, self.measure_PosY, 'r--', label='GPS', linewidth=1.5)
        plt.plot(self.POS_X, self.POS_Y, 'b-', label='EKF', linewidth=1.5)
        plt.title('xy trajectory')
        plt.legend(['GPS','EKF'])
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.grid(True)
        plt.axis('equal')

        plt.figure(3)
        plt.clf()
        if len(self.P_R) > 0:
            P_R_array = np.array(self.P_R)
            if P_R_array.shape[0] == len(self.time):
                plt.plot(self.time, P_R_array[:,0], 'b', self.time, P_R_array[:,1], 'g', self.time, P_R_array[:,2], 'r')
        plt.title('Covariance of Position')
        plt.legend(['px','py','pz'])
        plt.xlabel('Time [s]')
        plt.ylabel('Covariance [m²]')
        plt.grid(True)
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

        plt.figure(10)
        plt.clf()
        if len(self.Bias) > 0:
            Bias_array = np.array(self.Bias)
            if Bias_array.shape[0] == len(self.time):
                plt.plot(self.time, Bias_array[:,0], 'b', self.time, Bias_array[:,1], 'g', self.time, Bias_array[:,2], 'r')
        plt.title('Gyroscope Bias')
        plt.legend(['bp','bq','br'])
        plt.xlabel('Time [s]')
        plt.ylabel('Bias [rad/s]')
        plt.grid(True)
        
        plt.figure(11)
        plt.clf()
        if len(self.Bias) > 0:
            Bias_array = np.array(self.Bias)
            if Bias_array.shape[0] == len(self.time):
                plt.plot(self.time, Bias_array[:,3], 'b', self.time, Bias_array[:,4], 'g', self.time, Bias_array[:,5], 'r')
        plt.title('Accelerometer Bias')
        plt.legend(['bfx','bfy','bfz'])
        plt.xlabel('Time [s]')
        plt.ylabel('Bias [m/s²]')
        plt.grid(True)
                
        plt.ion()
        plt.show()
        plt.pause(0.0001)
    
    def publish_odometry(self):
        """Publish odometry message, transform, and path"""
        # Always publish transform to keep odom frame alive in tf tree
        # This prevents rviz from showing black screen
        current_time = self.get_clock().now()
        
        # Get current state estimate (or use zeros if GPS not synced)
        if not (self.flag_lat and self.flag_lon):
            # Publish initial transform to establish odom frame
            x = 0.0
            y = 0.0
            z = 0.0
            qw = 1.0
            qx = 0.0
            qy = 0.0
            qz = 0.0
            vx = 0.0
            vy = 0.0
            vz = 0.0
        else:
            # State vector: [x y z vx vy vz q1 q2 q3 q4 bp bq br bfx bfy bfz]
            # where q1, q2, q3, q4 = [qw, qx, qy, qz]
            x = float(self.xhat[0,0])
            y = float(self.xhat[1,0])
            z = float(self.xhat[2,0])
            vx = float(self.xhat[3,0])
            vy = float(self.xhat[4,0])
            vz = float(self.xhat[5,0])
            qw = float(self.xhat[6,0])
            qx = float(self.xhat[7,0])
            qy = float(self.xhat[8,0])
            qz = float(self.xhat[9,0])
        
        # Always publish transform (even before GPS sync)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
        
        # Only publish full odometry and path if GPS is synced
        if not (self.flag_lat and self.flag_lon):
            return
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set pose
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Set velocities
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.x = float(self.p)
        odom.twist.twist.angular.y = float(self.q)
        odom.twist.twist.angular.z = float(self.r)
        
        # Set pose covariance (6x6 matrix: x, y, z, roll, pitch, yaw)
        pose_cov = np.zeros(36)
        pose_cov[0] = self.P[0,0]   # x-x
        pose_cov[7] = self.P[1,1]   # y-y
        pose_cov[14] = self.P[2,2]  # z-z
        # Orientation covariance (using quaternion covariance diagonal)
        pose_cov[21] = self.P[6,6]  # qw-qw
        pose_cov[28] = self.P[7,7]  # qx-qx
        pose_cov[35] = self.P[8,8]  # qy-qy
        odom.pose.covariance = pose_cov.tolist()
        
        # Set twist covariance (6x6 matrix: vx, vy, vz, wx, wy, wz)
        twist_cov = np.zeros(36)
        twist_cov[0] = self.P[3,3]   # vx-vx
        twist_cov[7] = self.P[4,4]   # vy-vy
        twist_cov[14] = self.P[5,5]  # vz-vz
        twist_cov[21] = 0.01  # wx-wx (angular velocity uncertainty)
        twist_cov[28] = 0.01  # wy-wy
        twist_cov[35] = 0.01  # wz-wz
        odom.twist.covariance = twist_cov.tolist()
        
        # Publish odometry
        self.odom_publisher.publish(odom)
        
        # Create and publish path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = current_time.to_msg()
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        
        self.path.header.stamp = current_time.to_msg()
        self.path.poses.append(pose_stamped)
        
        # Keep only last 10000 poses to limit memory
        if len(self.path.poses) > 10000:
            self.path.poses.pop(0)
        
        # Publish path
        self.path_publisher.publish(self.path)
    
    def save_plots(self, output_dir='cp2_plots'):
        """Save all plots to files in the specified directory"""
        import os
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Save Figure 1: Euler angles
        if len(self.time) > 0 and len(self.PHI) > 0:
            plt.figure(1, figsize=(10, 6))
            plt.plot(self.time,self.PHI,'b', self.time, self.THETA, 'g', self.time,self.PSI, 'r')
            plt.legend(['phi','theta','psi'])
            plt.title('Phi, Theta, Psi [deg]')
            plt.xlabel('Time [s]')
            plt.ylabel('Angle [deg]')
            plt.grid(True)
            plt.savefig(os.path.join(output_dir, 'euler_angles.png'), dpi=300, bbox_inches='tight')
            plt.close(1)
        
        # Save Figure 2: xy trajectory
        if len(self.measure_PosX) > 0 and len(self.POS_X) > 0:
            plt.figure(2, figsize=(10, 8))
            plt.plot(self.measure_PosX, self.measure_PosY, 'r--', label='GPS', linewidth=1.5)
            plt.plot(self.POS_X, self.POS_Y, 'b-', label='EKF', linewidth=1.5)
            plt.title('xy trajectory')
            plt.legend(['GPS','EKF'])
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.grid(True)
            plt.axis('equal')
            plt.savefig(os.path.join(output_dir, 'xy_trajectory.png'), dpi=300, bbox_inches='tight')
            plt.close(2)
        
        # Save Figure 3: Covariance of Position
        if len(self.P_R) > 0 and len(self.time) > 0:
            plt.figure(3, figsize=(10, 6))
            P_R_array = np.array(self.P_R)
            if P_R_array.shape[0] == len(self.time):
                plt.plot(self.time, P_R_array[:,0], 'b', label='px', linewidth=1.5)
                plt.plot(self.time, P_R_array[:,1], 'g', label='py', linewidth=1.5)
                plt.plot(self.time, P_R_array[:,2], 'r', label='pz', linewidth=1.5)
            plt.title('Covariance of Position')
            plt.legend(['px','py','pz'])
            plt.xlabel('Time [s]')
            plt.ylabel('Covariance [m²]')
            plt.grid(True)
            plt.savefig(os.path.join(output_dir, 'covariance_position.png'), dpi=300, bbox_inches='tight')
            plt.close(3)
        
        # Save Figure 10: Gyroscope Bias
        if len(self.Bias) > 0 and len(self.time) > 0:
            plt.figure(10, figsize=(10, 6))
            Bias_array = np.array(self.Bias)
            if Bias_array.shape[0] == len(self.time):
                plt.plot(self.time, Bias_array[:,0], 'b', label='bp', linewidth=1.5)
                plt.plot(self.time, Bias_array[:,1], 'g', label='bq', linewidth=1.5)
                plt.plot(self.time, Bias_array[:,2], 'r', label='br', linewidth=1.5)
            plt.title('Gyroscope Bias')
            plt.legend(['bp','bq','br'])
            plt.xlabel('Time [s]')
            plt.ylabel('Bias [rad/s]')
            plt.grid(True)
            plt.savefig(os.path.join(output_dir, 'gyroscope_bias.png'), dpi=300, bbox_inches='tight')
            plt.close(10)
        
        # Save Figure 11: Accelerometer Bias
        if len(self.Bias) > 0 and len(self.time) > 0:
            plt.figure(11, figsize=(10, 6))
            Bias_array = np.array(self.Bias)
            if Bias_array.shape[0] == len(self.time):
                plt.plot(self.time, Bias_array[:,3], 'b', label='bfx', linewidth=1.5)
                plt.plot(self.time, Bias_array[:,4], 'g', label='bfy', linewidth=1.5)
                plt.plot(self.time, Bias_array[:,5], 'r', label='bfz', linewidth=1.5)
            plt.title('Accelerometer Bias')
            plt.legend(['bfx','bfy','bfz'])
            plt.xlabel('Time [s]')
            plt.ylabel('Bias [m/s²]')
            plt.grid(True)
            plt.savefig(os.path.join(output_dir, 'accelerometer_bias.png'), dpi=300, bbox_inches='tight')
            plt.close(11)
        
        print(f"Plots saved to {output_dir}/")
        print("  - euler_angles.png")
        print("  - xy_trajectory.png")
        print("  - covariance_position.png")
        print("  - gyroscope_bias.png")
        print("  - accelerometer_bias.png")

def main(args=None):
    rclpy.init(args=args)

    ekf_node = ExtendedKalmanFilter()
    
    try:
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save plots before shutting down
        ekf_node.save_plots()
        ekf_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
