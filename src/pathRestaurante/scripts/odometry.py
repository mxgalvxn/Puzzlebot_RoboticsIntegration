#!/usr/bin/env python3

import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import math
from std_msgs.msg import Float32

# Inicialización de variables globales
wl = 0.0
wr = 0.0

def wl_callback(data):
    global wl
    wl = data.data

def wr_callback(data):
    global wr
    wr = data.data

def normalize_angle(angle):
    """Normaliza el ángulo al rango [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

if __name__ == "__main__":
    rospy.init_node('odometry')
    nodeRate = 100
    rate = rospy.Rate(nodeRate)

    dt = 1.0 / nodeRate  # Paso de tiempo (s)
    t = 0  # Tiempo total (s)

    wheel_radius = 0.05  # Radio de las ruedas (m)
    wheelbase = 0.19  # Distancia entre ruedas (m) (l)

    # Inicialización de la posición y orientación
    x = 0.0
    y = 0.0
    theta = 0.0  # Orientación (rad)

    # Publicadores
    pubX = rospy.Publisher('/xPosition', Float32, queue_size=10)
    pubY = rospy.Publisher('/yPosition', Float32, queue_size=10)
    pubTheta = rospy.Publisher('/thetaOrientation', Float32, queue_size=10)

    # Subscriptores
    sub_wl = rospy.Subscriber('/wl', Float32, wl_callback)
    sub_wr = rospy.Subscriber('/wr', Float32, wr_callback)

    while not rospy.is_shutdown():
        # Cálculo de velocidades
        v_real = wheel_radius * (wr + wl) / 2
        w_real = wheel_radius * (wr - wl) / wheelbase

        # Actualización de la posición
        vx = v_real * math.cos(theta)
        vy = v_real * math.sin(theta)

        x += vx * dt
        y += vy * dt
        theta += w_real * dt

        # Normalización de theta
        theta = normalize_angle(theta)

        # Publicación de los valores de odometría
        pubX.publish(x)
        pubY.publish(y)
        pubTheta.publish(theta)

        # print("x: ", x)
        # print("y: ", y)
        # print("theta: ", theta)

        rate.sleep()
