#!/usr/bin/env python3

import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int16, Bool, Float32
import random

dt = 0.01
t = 0.0
kpr = 3.2
kpt = 4.5

def posY_callback(data):
    global posY
    posY = data.data

def posX_callback(data):
    global posX
    posX = data.data

def ori_callback(data):
    global ori
    ori = data.data

def lidar_callback(data):
    global stopLidar
    stopLidar = data.data

def ruta():
    numPath = rospy.get_param("/ruta/numMesa")
    #numPath = 1
    ruta_archivo = "/home/puzzlebot/mushuBot/src/mushu/paths/path" + str(numPath) + "Try1.txt"

    datos = np.loadtxt(ruta_archivo, skiprows=1)

    posiciones_x = .01 * datos[:, 0]
    posiciones_y = .01 * datos[:, 1]

    diferenciaX = posiciones_x[0]
    diferenciaY = posiciones_y[0]

    for i in range(len(posiciones_x)):
        posiciones_x[i] -= diferenciaX
        posiciones_y[i] -= diferenciaY
        if posiciones_y[i] < 0:
            posiciones_y[i] *= -1
        # if posiciones_x[i] < 0:
        #     posiciones_x[i] *= -1
    
    return  posiciones_x, posiciones_y

if __name__=="__main__":
    try:
        rospy.init_node('square_mover')
        print('INICIO EL NODO DE CONTROL')
        nodeRate = 100
        rate = rospy.Rate(nodeRate)

        wheel_radius = 0.05  # radius of wheels (m)
        wheelbase = 0.19  # distance between wheels (m) (l)
        wr = 0.0
        wl = 0.0


        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        orientacion = rospy.Subscriber('/thetaOrientation', Float32, ori_callback)
        pos_act_x = rospy.Subscriber('/xPosition', Float32, posX_callback)
        pos_act_y = rospy.Subscriber('/yPosition', Float32, posY_callback)
        stop1 = rospy.Subscriber("/stop", Bool, lidar_callback)
        
        twist = Twist()

        posiciones_x, posiciones_y = ruta()

        #posiciones_x = [.2, .3]
        #posiciones_y = [.2, .7]

        posiciones_x_rev = np.flip(posiciones_x)
        posiciones_y_rev = np.flip(posiciones_y)

        num_positions = len(posiciones_x)

        posX = 0.0
        posY = 0.0
        ori = 0.0
        stopLidar = False
        done = False
        for k in range (2):
            i = 0
            t = 0.0
            reverse = False
            if k == 1:
                reverse = True
            
            error_dis_ant = 0.0
            error_ori_ant = 0.0
            
            while i < num_positions:
                if stopLidar:
                    vmax = 0.00001
                    wmax = 0.00001
                else:
                    vmax = 0.4
                    wmax = 0.4
               
                if not reverse:                                                                                                           
                    x_des = posiciones_x[i]
                    y_des = posiciones_y[i]
                else:       
                    x_des = posiciones_x_rev[i]
                    y_des = posiciones_y_rev[i]     

                error_dis = math.sqrt((x_des-posX)**2+ (y_des-posY)**2)
                #error_ori = ori - math.atan2(y_des-posY, x_des-posX)
                error_ori = math.atan2(y_des-posY, x_des-posX) - ori
                #ori = math.ra
                
                if error_ori > math.pi:
                    error_ori = error_ori - 2*math.pi
                elif error_ori < -math.pi:
                    error_ori = error_ori + 2*math.pi

                print(i)
                print("error distancia: ", error_dis)
                print("error_orientacion: ", error_ori)
                #print("orientacion: ", ori)
                #print("x: ", round(posX,3), "x deseada: ", round(x_des,3))
                #print("y: ", round(posY,3), "y deseada: ", round(y_des,3))
                
                error_dis_der= error_dis - error_dis_ant
                error_ori_der = error_ori - error_ori_ant

                #u_vel = vmax*math.tanh(error_dis*kpt/vmax)

                u_vel = kpr * error_dis + kpt * error_dis_der
                vref = vmax*math.tanh(u_vel/vmax)
                
                u_ori = kpr * error_ori + kpt * error_ori_der
                wref = wmax*math.tanh(u_ori/wmax)
                
                error_dis_ant = error_dis
                error_ori_ant = error_ori
                
                # vref = vmax * (u_vel)
                # wref = wmax * (u_ori)

                twist.linear.x = 0
                if abs(error_ori) < 0.08:
                    twist.linear.x = vref
                twist.angular.z = wref
                pub.publish(twist)

                if abs(error_dis) < 0.08:
                    i += 1
                t = t + dt
                rate.sleep()
            print("Ruta terminada ",k)
            oriVuelta = ori
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)
            rospy.sleep(5)
            
            # while abs(oriVuelta - ori) > math.pi:
            #     print(oriVuelta - ori)
            #     twist.angular.z = wmax * .1
            #     pub.publish(twist)
            # twist.linear.x = 0
            # twist.angular.z = 0
            # pub.publish(twist)
            # rospy.sleep(2)
            
        numPath = rospy.get_param("/ruta/numMesa")
        print("mesa ", numPath )
    except rospy.ROSInterruptException:
        pass
