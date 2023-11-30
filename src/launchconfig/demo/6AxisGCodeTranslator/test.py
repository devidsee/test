import re

file = open('/home/wangyabin/catkin_ws/src/launchconfig/6AxisGCodeTranslator/a.txt','r')
gcode = file.readlines()
for line in gcode:
    # print(line)
    coord = re.split(r'[ ]', line)
    if coord:
        print("{} - {}- {}- {}- {}".format(coord[1].split('X')[1], coord[2].split('Y')[1], coord[3].split('Z')[1], coord[4].split('A')[1], coord[5].split('B')[1]))