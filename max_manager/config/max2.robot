[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyACM0 | 57600     | r_sho_pitch

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       | BULK READ ITEMS
dynamixel | /dev/ttyACM0 | 1   | XM-430         | 2.0      | r_sho_pitch    | present_position
dynamixel | /dev/ttyACM0 | 2   | XM-430         | 2.0      | l_sho_pitch    | present_position
dynamixel | /dev/ttyACM0 | 3   | XM-430         | 2.0      | r_sho_roll     | present_position
dynamixel | /dev/ttyACM0 | 4   | XM-430         | 2.0      | l_sho_roll     | present_position
dynamixel | /dev/ttyACM0 | 5   | XM-430         | 2.0      | r_el_yaw       | present_position
dynamixel | /dev/ttyACM0 | 6   | XM-430         | 2.0      | l_el_yaw       | present_position
dynamixel | /dev/ttyACM0 | 7   | XM-430         | 2.0      | r_el_pitch     | present_position
dynamixel | /dev/ttyACM0 | 8   | XM-430         | 2.0      | l_el_pitch     | present_position
dynamixel | /dev/ttyACM0 | 9   | XM-430         | 2.0      | head_roll      | present_position
dynamixel | /dev/ttyACM0 | 10  | XM-430         | 2.0      | head_pitch     | present_position
dynamixel | /dev/ttyACM0 | 11  | XM-430         | 2.0      | waist_yaw      | present_position
dynamixel | /dev/ttyACM0 | 12  | XM-430         | 2.0      | waist_pitch    | present_position
