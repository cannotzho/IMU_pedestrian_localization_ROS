#!/usr/bin/python3

# Install packages to use Python3 with ROS 
# sudo apt-get install python3-yaml
# sudo pip3 install rospkg catkin_pkg

import os
# Constrain OPENBLAS Multithreading (To solve Numpy Performance Issues)
os.environ['OPENBLAS_NUM_THREADS'] = '1'

import sys 
import signal
import rospkg
from pedestrian_localizer import pedestrian_localizer
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import math
from INS.tools.csv_parser import readCSV, readROSBagCSV, writeCSV 
from INS.tools.data_visualizer import show3Dposition, show2Dposition, interactive2Dposition_init, update2Dposition, printProgressBar
from INS.tools.geometry_utils import rotateOutput
from INS.tools.geometry_helpers import euler2quat
import signal
import rosbag

shutdown = False

def signal_handler(sig, frame):
    print('User Interrupt: Ctrl+C!')
    shutdown = True
    sys.exit(0)

def publish_odom(x, p, header):
    """ Publish Odometry Message

        :param x: State
                       pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw
    """
    global x_out
    if x is not None:
        x_out.append([x, header])

        x, y, z, vel_x, vel_y, vel_z, roll, pitch, yaw = x
        qw, qx, qy, qz =  euler2quat(roll, pitch, yaw, axes='sxyz')
        odom = Odometry()
        odom.header = header
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z 
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = vel_y
        odom.twist.twist.linear.z = vel_z

        pose_covariance = np.zeros((6,6))
        pose_covariance[:3, :3] = p[:3, :3]
        pose_covariance[:3, 3:] = p[:3, 6:]
        pose_covariance[3:, :3] = p[6:, :3]
        pose_covariance[3:, 3:] = p[6:, 6:]
        odom.pose.covariance = pose_covariance.reshape(-1).tolist()

        twist_covariance = np.zeros((6,6))
        twist_covariance[:3, :3] = p[3:6, 3:6]
        odom.twist.covariance = twist_covariance.reshape(-1).tolist()

        odom_pub.publish(odom)


if __name__ == '__main__':
    ########## Parameters ###########

    calibrate_yaw = True
    calibration_steps = 1
    yaw_pub_method = 'Stable'  # 'Real', 'Zupt', 'Stable'
    yaw_pub_latch = True 
    directory = '/home/sutd/catkin_ws/src/IMU_pedestrian_localization_ROS/results2/2-3JunData/HngzData/JogRun'

    ########## Initialization ###########
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('imu_odometry_publisher', anonymous=True)
    odom_pub = rospy.Publisher('imu_odometry', Odometry, queue_size=100)

    localizer = pedestrian_localizer(calibrate_yaw=calibrate_yaw, calibration_distance=calibration_steps, yaw_method=yaw_pub_method, yaw_latch=yaw_pub_latch, callback=publish_odom)

    package_path = rospkg.RosPack().get_path('imu_odometry')
    data_dir = os.path.join(package_path, 'results2/2-3JunData/HngzData/JogRun')
    result_dir = os.path.join(package_path, 'results2')

    for entry in os.scandir(directory):        

        if (entry.path.endswith(".csv")) and (entry.is_file()):
            #Check bag file message count
            bagName = entry.path[:-4] + ".bag"
            bagFile = rosbag.Bag(bagName)

            if ((bagFile.get_message_count() < 1000)):
                os.remove(bagName)
                
                os.remove(entry.path)
            else:
                fileName = entry.path[len(directory)+1:-4] # CSV file name without extension


                print ("IMU Odometry Publisher")
                print("Demo: " + fileName)

                

                ########## Reading IMU Data ##########
                fieldNames= ["field.header.seq", "field.header.stamp", "field.header.frame_id",
                                "field.linear_acceleration.x", "field.linear_acceleration.y", "field.linear_acceleration.z",
                                "field.angular_velocity.x", "field.angular_velocity.y", "field.angular_velocity.z"]
                dataTypes = [int, int, 'U20', float, float, float, float, float, float]

                status, userData = readROSBagCSV(os.path.join(data_dir, fileName+'.csv'), fields=fieldNames, dtype=dataTypes)

                # ros_data = userData.view((float, len(userData.dtype.names)))
                # ros_data[:,0] = userData['field.header.stamp']

                dataSteps = len(userData['field.header.stamp'])
                print ("Input data steps: ", dataSteps)

                ########## Generate Path ###########
                odom = Odometry()
                x_out = []
                data = Imu()
            
                for i in range (dataSteps):
                    data.header.seq = int(userData['field.header.seq'][i])
                    data.header.stamp.secs = int(userData['field.header.stamp'][i] * 1e-9)
                    data.header.stamp.nsecs = int(userData['field.header.stamp'][i] % 1e9)
                    data.header.frame_id = userData['field.header.frame_id'][i]
                    data.linear_acceleration.x = userData['field.linear_acceleration.x'][i]
                    data.linear_acceleration.y = userData['field.linear_acceleration.y'][i]
                    data.linear_acceleration.z = userData['field.linear_acceleration.z'][i]
                    data.angular_velocity.x = userData['field.angular_velocity.x'][i]
                    data.angular_velocity.y = userData['field.angular_velocity.y'][i]
                    data.angular_velocity.z = userData['field.angular_velocity.z'][i]

                    # Check shutdown condition
                    if shutdown:
                        sys.exit(0)

                    # Estimate Odometry
                    localizer.update_odometry(data)
                    printProgressBar(i, dataSteps, 'Progress', 'Complete', length=50)
                
                output = np.zeros((len(x_out), 10))
                output[:,0] = userData['field.header.stamp']   # Time Stamps

                # Rotate and generate output
                for i in range(len(x_out)):
                    #output[i,1:10] = rotateOutput(x_out[i][0], roll=math.pi, pitch=0, yaw=0)
                    output[i,1:10] = x_out[i][0]
                

                # writeCSV(output, os.path.join(result_dir, fileName+'.csv'), fields=['time', 
                #                                                     'x_position', 'y_position', 'z_position', 
                #                                                     'vel_x', 'vel_y', 'vel_z',
                #                                                     'roll', 'pitch', 'yaw',])
                show2Dposition(output[:,1:], result_dir+ "/" + fileName)
                show3Dposition(output[:,1:], result_dir+ "/" + fileName)

                rospy.loginfo("Execution Complete")

