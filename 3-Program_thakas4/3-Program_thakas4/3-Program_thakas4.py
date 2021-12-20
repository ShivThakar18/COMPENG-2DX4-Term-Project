# 2DX4 Final Project
#	
#  Name: Shiv Thakar 
#  Student No: 400247588
# 	MACID: thakas4
#
#	Assigned Clock-Speed and LED
#			Clock-Speed -> 30 MHz 
#			Onboard LED -> PN1 (LED D1) 
#

import serial
import open3d as o3d
import math
import numpy as np
#Modify the following line with your own serial port details
#   Currently set COM6 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.

loop = 1
s = serial.Serial('COM6', 115200)
x = 0
counter = 0
lines = []
print("Opening: " + s.name)

#Making XYZ file
xyzfile = open("tof_radar.xyz", "w")

while(loop == 1):
    loop = 0

    distance = 0
    angle = math.pi/16

    #Printing out Status
    for i in range(10):
        r = s.readline();
        c = r.decode(); 
        print(c);

    #Print out Distances
    for i in range(32):
        r = s.readline()        # read one byte
        c = r.decode()      # convert byte type to str

        distance = (int)(c)     #type casting to integer

        
        y = distance*(math.sin(angle))
        z = distance*(math.cos(angle))

        xyzfile.write((str)(x))
        xyzfile.write(' '.format(i))
        xyzfile.write((str)(y));
        xyzfile.write(' '.format(i))
        xyzfile.write((str)(z));
        xyzfile.write(' \n'.format(i))
        
        angle = angle + math.pi/16

        print(c)

        
    r = s.readline()
    c = r.decode()

    loop = (int)(c)

    x = x + 200

    
xyzfile.close() 

pcd = o3d.io.read_point_cloud("tof_radar.xyz", format = 'xyz')
print(pcd)
print(np.asarray(pcd.points))

for i in range(10):
    
    pt1 = 0
    pt2 = 1
    pt3 = 2
    pt4 = 3
    pt5 = 4
    pt6 = 5
    pt7 = 6
    pt8 = 7
    pt9 = 8
    pt10 = 9
    pt11 = 10
    pt12 = 11
    pt13 = 12
    pt14 =13
    pt15 =14
    pt16 =15
    pt17 =16
    pt18 =17
    pt19 =18
    pt20 =19
    pt21 =20
    pt22 =21
    pt23 =22
    pt24 =23
    pt25 =24
    pt26 =25
    pt27 =26
    pt28 =27
    pt29 =28
    pt30 =29
    pt31 =30
    pt32 =31
    po = 0

    for i in range(10):
        lines.append([pt1+po,pt2+po])
        lines.append([pt2+po,pt3+po])
        lines.append([pt3+po,pt4+po])
        lines.append([pt4+po,pt5+po])
        lines.append([pt5+po,pt6+po])
        lines.append([pt6+po,pt7+po])
        lines.append([pt7+po,pt8+po])
        lines.append([pt8+po,pt9+po])
        lines.append([pt9+po,pt10+po])
        lines.append([pt10+po,pt11+po])
        lines.append([pt11+po,pt12+po])
        lines.append([pt12+po,pt13+po])
        lines.append([pt13+po,pt14+po])
        lines.append([pt14+po,pt15+po])
        lines.append([pt15+po,pt16+po])
        lines.append([pt16+po,pt17+po])
        lines.append([pt17+po,pt18+po])
        lines.append([pt18+po,pt19+po])
        lines.append([pt19+po,pt20+po])
        lines.append([pt20+po,pt21+po])
        lines.append([pt21+po,pt22+po])
        lines.append([pt22+po,pt23+po])
        lines.append([pt23+po,pt24+po])
        lines.append([pt24+po,pt25+po])
        lines.append([pt25+po,pt26+po])
        lines.append([pt26+po,pt27+po])
        lines.append([pt27+po,pt28+po])
        lines.append([pt28+po,pt29+po])
        lines.append([pt29+po,pt30+po])
        lines.append([pt30+po,pt31+po])
        lines.append([pt31+po,pt32+po])
        lines.append([pt32+po,pt1+po])
        po+= 32
 
    pt1 = 0
    pt2 = 1
    pt3 = 2
    pt4 = 3
    pt5 = 4
    pt6 = 5
    pt7 = 6
    pt8 = 7
    pt9 = 8
    pt10 = 9
    pt11 = 10
    pt12 = 11
    pt13 = 12
    pt14 =13
    pt15 =14
    pt16 =15
    pt17 =16
    pt18 =17
    pt19 =18
    pt20 =19
    pt21 =20
    pt22 =21
    pt23 =22
    pt24 =23
    pt25 =24
    pt26 =25
    pt27 =26
    pt28 =27
    pt29 =28
    pt30 =29
    pt31 =30
    pt32 =31
    po = 0
    do = 32

    for i in range(9):
        lines.append([pt1+po,pt1+do+po])
        lines.append([pt2+po,pt2+do+po])
        lines.append([pt3+po,pt3+do+po])
        lines.append([pt4+po,pt4+do+po])
        lines.append([pt5+po,pt5+do+po])
        lines.append([pt6+po,pt6+do+po])
        lines.append([pt7+po,pt7+do+po])
        lines.append([pt8+po,pt8+do+po])
        lines.append([pt9+po,pt9+do+po])
        lines.append([pt10+po,pt10+do+po])
        lines.append([pt11+po,pt11+do+po])
        lines.append([pt12+po,pt12+do+po])
        lines.append([pt13+po,pt13+do+po])
        lines.append([pt14+po,pt14+do+po])
        lines.append([pt15+po,pt15+do+po])
        lines.append([pt16+po,pt16+do+po])
        lines.append([pt17+po,pt17+do+po])
        lines.append([pt18+po,pt18+do+po])
        lines.append([pt19+po,pt19+do+po])
        lines.append([pt20+po,pt20+do+po])
        lines.append([pt21+po,pt21+do+po])
        lines.append([pt22+po,pt22+do+po])
        lines.append([pt23+po,pt23+do+po])
        lines.append([pt24+po,pt24+do+po])
        lines.append([pt25+po,pt25+do+po])
        lines.append([pt26+po,pt26+do+po])
        lines.append([pt27+po,pt27+do+po])
        lines.append([pt28+po,pt28+do+po])
        lines.append([pt29+po,pt29+do+po])
        lines.append([pt30+po,pt30+do+po])
        lines.append([pt31+po,pt31+do+po])
        lines.append([pt31+po,pt32+do+po])
        po+= 32

line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)) , lines = o3d.utility.Vector2iVector(lines))

o3d.visualization.draw_geometries([line_set])



print("Closing: " + s.name)
s.close()
