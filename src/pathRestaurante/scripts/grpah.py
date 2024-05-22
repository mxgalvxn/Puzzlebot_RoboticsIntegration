import matplotlib.pyplot as plt
import math
import numpy as np

distancia1 = 0
distancia2 = 0

# Crear una figura fuera del bucle
plt.figure(figsize=(8, 6))

for i in range(1, 8):
    ruta_archivo = "paths/path" + str(i) + "Try1.txt"

    datos = np.loadtxt(ruta_archivo, skiprows=1)

    x1 = .005 * datos[:, 0]
    y1 = .005 * datos[:, 1]

    x2 = np.flip(x1)
    y2 = np.flip(y1)

    for j in range(len(x2)):
        x2[j] -= x1[0]
        y2[j] -= y1[0]
        # if x1[i] < 0:
        #     x1[i] *= -1
        # elif x2[i] < 0:
        #     x2[i] *= -1
        if y1[j] < 0:
            y1[j] *= -1
        elif y2[j] < 0:
            y2[j] *= -1

    for j in range(1, len(x2)):
        distancia1 = math.sqrt((x1[j] - x1[j - 1])**2 + (y1[j] - y1[j - 1])**2)
        distancia2 = math.sqrt((x2[j] - x2[j - 1])**2 + (y2[j] - y2[j - 1])**2)
        print("d1", distancia1, "punto:", x1[j], y1[j])
        print("d2", distancia2, "punto:", x2[j], y2[j])

    plt.plot(x1, y1, marker='o', linestyle='-', label=f'Trayectoria 1 - Archivo {i}')
    #plt.plot(x2, y2, marker='o', linestyle='-', label=f'Trayectoria 2 - Archivo {i}')

#Configurar la visualización de la gráfica
plt.title('Trayectorias')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()
