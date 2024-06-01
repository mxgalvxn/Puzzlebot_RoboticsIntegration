import numpy as np
import matplotlib.pyplot as plt
import math

dt = 0.01
t = 0.0
kpr = 3.2
kpt = 4.5

# Variables globales
posX = 0.0
posY = 0.0
ori = 0.0
stopLidar = False

def ruta():
    # Simulación de la ruta
    ruta_archivo = "paths/path7Try1.txt"
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

    return posiciones_x, posiciones_y

if __name__ == "__main__":
    wheel_radius = 0.05  # radius of wheels (m)
    wheelbase = 0.19  # distance between wheels (m) (l)
    wr = 0.0
    wl = 0.0

    posiciones_x, posiciones_y = ruta()
    posiciones_x_rev = np.flip(posiciones_x)
    posiciones_y_rev = np.flip(posiciones_y)
    num_positions = len(posiciones_x)

    posX = 0.0
    posY = 0.0
    ori = 0.0
    stopLidar = False

    # Kalman filter variables
    x_hat = np.array([[0.0], [0.0], [0.0]])  # Initial state estimate
    P = np.eye(3)  # Initial estimate covariance
    Q = np.eye(3) * 0.01  # Process noise covariance
    R = np.eye(3) * 0.058  # Measurement noise covariance

    trayectoria = []
    trayectoria_kalman = []

    for k in range(1):
        i = 0
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

            error_dis = math.sqrt((x_des - posX)**2 + (y_des - posY)**2)
            error_ori = math.atan2(y_des - posY, x_des - posX) - ori

            if error_ori > math.pi:
                error_ori = error_ori - 2 * math.pi
            elif error_ori < -math.pi:
                error_ori = error_ori + 2 * math.pi

            error_dis_der = error_dis - error_dis_ant
            error_ori_der = error_ori - error_ori_ant

            u_vel = kpr * error_dis + kpt * error_dis_der
            vref = vmax * math.tanh(u_vel / vmax)

            u_ori = kpr * error_ori + kpt * error_ori_der
            wref = wmax * math.tanh(u_ori / wmax)

            error_dis_ant = error_dis
            error_ori_ant = error_ori

            if abs(error_ori) < 0.05:
                posX += vref * dt * math.cos(ori)
                posY += vref * dt * math.sin(ori) 
            ori += wref * dt 

            # Kalman filter update
            A = np.eye(3)  # State transition matrix
            B = np.array([[dt * math.cos(ori), 0], 
                          [dt * math.sin(ori), 0], 
                          [0, dt]])  # Control input matrix
            u = np.array([[vref], [wref]])  # Control vector

            # Prediction step
            x_hat = A @ x_hat + B @ u  # Add process noise
            P = A @ P @ A.T + Q

            # Measurement update step
            z = np.array([[posX], [posY], [ori]])  # Add measurement noise
            y = z - x_hat  # Innovation
            S = P + R  # Innovation covariance
            K = P @ np.linalg.inv(S)  # Kalman gain
            x_hat = x_hat + K @ y
            P = (np.eye(3) - K) @ P

            trayectoria.append([t, posX, posY, ori])
            trayectoria_kalman.append([t, x_hat[0, 0], x_hat[1, 0], x_hat[2, 0]])

            if abs(error_dis) < 0.05:
                i += 1

            t = t + dt

        print("Ruta terminada ", k)

    # Graficar resultados
    trayectoria = np.array(trayectoria)
    trayectoria_kalman = np.array(trayectoria_kalman)

    plt.figure()
    plt.plot(trayectoria[:, 1], trayectoria[:, 2], label="Trayectoria")
    plt.plot(trayectoria_kalman[:, 1], trayectoria_kalman[:, 2], label="Kalman Filtrada")
    plt.scatter(posiciones_x, posiciones_y, color='red', label="Ruta Deseada")
    plt.xlabel('Posición X')
    plt.ylabel('Posición Y')
    plt.legend()
    plt.title('Trayectoria del Robot')
    plt.show()
