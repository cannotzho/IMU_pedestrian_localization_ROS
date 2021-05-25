#!/usr/bin/env python3

import rospy
import subprocess, shlex, psutil
import signal
import time
import sys 
import rospkg

class RosbagRecord:
    def __init__(self):
        
        self.timeNow = time.strftime(("%Y%m%d-%H%M%S"))
        if rospy.has_param('~record_script') and rospy.has_param('~record_folder'):
            self.record_script = rospy.get_param('~record_script')
            self.record_folder = rospy.get_param('~record_folder')
            rospy.on_shutdown(self.stop_recording_handler)


            # Start recording.
            command = "rosbag record -O " + self.record_folder + self.timeNow + ".bag /footIMU/IMU"
            command = shlex.split(command)
            rosbag_convert = subprocess.Popen(command)
            # command = "source " + self.record_script
            # self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.record_folder,
            #                           executable='/bin/bash')

            # Wait for shutdown signal to close rosbag record
            rospy.spin()
        else:
            rospy.signal_shutdown(rospy.get_name() + ' no record script or folder specified.')

    def terminate_ros_node(self, s):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split(b"\n"):
            if (str.startswith(s.encode("ascii"))):
                os.system("rosnode kill " + str)

    def stop_recording_handler(self):
        rospy.loginfo(rospy.get_name() + ' stop recording.')
        self.terminate_ros_node("/record")
        command = 'rostopic echo -b "' + self.timeNow + '.bag" -p /footIMU/IMU > "' + self.timeNow + '.csv"'
        subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.record_folder,
                                      executable='/bin/bash')


if __name__ == '__main__':
    rospy.init_node('rosbag_record')
    rospy.loginfo(rospy.get_name() + ' start')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rosbag_record = RosbagRecord()
    except rospy.ROSInterruptException:
        pass