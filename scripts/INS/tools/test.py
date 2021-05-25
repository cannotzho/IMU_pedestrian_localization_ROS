command = 'rostopic echo -b "' + self.timeNow + '.bag" -p /footIMU/IMU > ' + self.timeNow + ".csv"
print(command)
command = shlex.split(command)
print(command)
