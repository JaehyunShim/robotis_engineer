[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyACM0 | 57600     | r_shoulder_pitch

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       	| BULK READ ITEMS
dynamixel | /dev/ttyACM0 | 1   | XM-430         | 2.0      | r_shoulder_pitch | present_position
dynamixel | /dev/ttyACM0 | 2   | XM-430         | 2.0      | r_shoulder_roll  | present_position
dynamixel | /dev/ttyACM0 | 3   | XM-430         | 2.0      | l_shoulder_pitch | present_position
dynamixel | /dev/ttyACM0 | 4   | XM-430         | 2.0      | l_shoulder_roll  | present_position
dynamixel | /dev/ttyACM0 | 5   | XM-430         | 2.0      | r_hip_roll       | present_position
dynamixel | /dev/ttyACM0 | 6   | XM-430         | 2.0      | r_hip_pitch      | present_position
dynamixel | /dev/ttyACM0 | 7   | XM-430         | 2.0      | l_hip_roll       | present_position
dynamixel | /dev/ttyACM0 | 8   | XM-430         | 2.0      | l_hip_pitch      | present_position
dynamixel | /dev/ttyACM0 | 9   | XM-430         | 2.0      | r_ankle_pitch    | present_position
dynamixel | /dev/ttyACM0 | 10  | XM-430         | 2.0      | r_ankle_roll     | present_position
dynamixel | /dev/ttyACM0 | 11  | XM-430         | 2.0      | l_ankle_pitch    | present_position
dynamixel | /dev/ttyACM0 | 12  | XM-430         | 2.0      | l_ankle_roll     | present_position
