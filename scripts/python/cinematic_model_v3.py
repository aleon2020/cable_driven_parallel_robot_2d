import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# PARAMETERS
largo_plano = 100
alto_plano = 100
largo_efector = 10
alto_efector = 10
radio_rueda = 5

# REQUEST FOR NUMBER OF COORDINATES
num_posiciones = int(input("\nNÚMERO DE POSICIONES (MÍNIMO 2, MÁXIMO 10): "))
num_posiciones = max(2, min(10, num_posiciones))
posiciones = []
errores = {}

# REQUEST FOR THE COORDINATES OF THE END-EFFECTOR
for i in range(num_posiciones):
    print(f"\nPOSICIÓN NÚMERO {i+1} DEL EFECTOR FINAL")
    x = float(input(f'Coordenada x de la posición número {i+1}: '))
    y = float(input(f'Coordenada y de la posición número {i+1}: '))
    posiciones.append((x, y))

# LIMIT VERIFICATION
def verificar_limites(x, y):
    errores = []
    if x >= (largo_plano - (largo_efector / 2)) or x <= (largo_efector / 2):
        errores.append(f"FUERA DEL LÍMITE en x = {x}")
    if y >= (alto_plano - (alto_efector / 2)) or y <= (alto_efector / 2):
        errores.append(f"FUERA DEL LÍMITE en y = {y}")
    return errores
for i, (x, y) in enumerate(posiciones):
    errores_posicion = verificar_limites(x, y)
    if errores_posicion:
        errores[i+1] = errores_posicion
if errores:
    for pos, errs in errores.items():
        print(f"\nERROR EN LA POSICIÓN NÚMERO {pos}")
        for err in errs:
            print(err)
    exit()

# PLANE
fig, ax = plt.subplots(figsize=(20, 16))
ax.set_xlim([-largo_plano * 0.25, largo_plano * 1.25])
ax.set_ylim([-alto_plano * 0.5, alto_plano * 1.25])
ax.set_xlabel('Eje X (centímetros)')
ax.set_ylabel('Eje Y (centímetros)')
ax.set_title('Robot por cables para el control de un efector final')

# STRUCTURE
ax.plot([0, 0], [alto_plano, -(alto_plano / 2)], 'k', linewidth=5)
ax.plot([largo_plano, largo_plano], [alto_plano, -(alto_plano / 2)], 'k', linewidth=5)
ax.plot([0, largo_plano], [alto_plano, alto_plano], 'k', linewidth=5)
ax.plot([0, largo_plano], [0, 0], 'k', linewidth=5)
ax.plot([-5, 5], [-(alto_plano / 2), -(alto_plano / 2)], 'k', linewidth=5)
ax.plot([largo_plano - 5, largo_plano + 5], [-(alto_plano / 2), -(alto_plano / 2)], 'k', linewidth=5)

# PULLEYS REPRESENTATION
ax.plot(0, alto_plano, 'ko', markersize=radio_rueda, markerfacecolor='black')
ax.plot(0, alto_plano, 'wo', markersize=radio_rueda / 2, markerfacecolor='black')
ax.plot(largo_plano, alto_plano, 'ko', markersize=radio_rueda, markerfacecolor='black')
ax.plot(largo_plano, alto_plano, 'wo', markersize=radio_rueda / 2, markerfacecolor='black')

# UPDATE OF THE POSITION OF THE DISPLACED ELEMENTS
efector, = ax.plot([], [], 'black', linewidth=2)
centro_efector, = ax.plot([], [], 'ko', markersize=2, markerfacecolor='black')
cable1, = ax.plot([], [], 'r', linewidth=2)
cable2, = ax.plot([], [], 'r', linewidth=2)
trayectoria, = ax.plot([], [], 'b--', linewidth=1, alpha=0.5)

# STORAGE OF POINTS OF THE TRAJECTORY TAKEN
trayectoria_x = []
trayectoria_y = []

# END-EFFECTOR
def calcular_esquinas(x, y):
    # Top left corner (x1, y1)
    x1 = x - (largo_efector / 2)
    y1 = y + (alto_efector / 2)
    # Top right corner (x2, y2)
    x2 = x + (largo_efector / 2)
    y2 = y + (alto_efector / 2)
    # Bottom left corner (x3, y3)
    x3 = x - (largo_efector / 2)
    y3 = y - (alto_efector / 2)
    # Bottom right corner (x4, y4)
    x4 = x + (largo_efector / 2)
    y4 = y - (alto_efector / 2)
    return x1, y1, x2, y2, x3, y3, x4, y4

# CABLES
def calcular_cables(x, y):
    x1, y1, x2, y2, _, _, _, _ = calcular_esquinas(x, y)
    # Cable top left corner M1 = (M1x, M1y)
    M1x = 0
    M1y = alto_plano
    # Cable top right corner M2 = (M2x, M2y)
    M2x = largo_plano
    M2y = alto_plano
    # CABLE LENGTHS
    L1 = np.sqrt((x1 - M1x) ** 2 + (y1 - M1y) ** 2)
    L2 = np.sqrt((x2 - M2x) ** 2 + (y2 - M2y) ** 2)
    # ANGLES
    q1 = -np.degrees(np.arctan((x1 - M1x) / (y1 - M1y)))
    q2 = np.degrees(np.arctan((x2 - M2x) / (y2 - M2y)))
    return L1, L2, q1, q2

# ANIMATION DATA INITIALIZATION
def init():
    efector.set_data([], [])
    centro_efector.set_data([], [])
    cable1.set_data([], [])
    cable2.set_data([], [])
    trayectoria.set_data([], [])
    return efector, centro_efector, cable1, cable2, trayectoria

# ANIMACIÓN
def animate(i):

    # FRAMES
    segmentos = num_posiciones - 1
    frames_por_segmento = 100 // segmentos
    segmento_actual = min(i // frames_por_segmento, segmentos - 1)
    progreso = (i % frames_por_segmento) / frames_por_segmento
    
    # INITIAL AND FINAL POSITION OF THE CURRENT SEGMENT
    x_inicio, y_inicio = posiciones[segmento_actual]
    x_fin, y_fin = posiciones[segmento_actual + 1]
    
    # CALCULATION OF CURRENT POSITION
    x = x_inicio + (x_fin - x_inicio) * progreso
    y = y_inicio + (y_fin - y_inicio) * progreso
    
    # END-EFFECTOR CORNERS
    x1, y1, x2, y2, x3, y3, x4, y4 = calcular_esquinas(x, y)
    
    # END-EFFECTOR REPRESENTATION
    xe = [x3, x4, x2, x1, x3]
    ye = [y3, y4, y2, y1, y3]
    efector.set_data(xe, ye)
    centro_efector.set_data(x, y)

    # CABLES
    # Cable top left corner M1 = (M1x, M1y)
    M1x = 0
    M1y = alto_plano
    # Cable top right corner M2 = (M2x, M2y)
    M2x = largo_plano
    M2y = alto_plano

    # UPDATE CABLE POSITION
    cable1.set_data([M1x, x1], [M1y, y1])
    cable2.set_data([M2x, x2], [M2y, y2])
    
    # TRAJECTORY UPDATE
    trayectoria_x.append(x)
    trayectoria_y.append(y)
    trayectoria.set_data(trayectoria_x, trayectoria_y)
    return efector, centro_efector, cable1, cable2, trayectoria

# SHOW DATA FOR EACH POSITION
for i, (x, y) in enumerate(posiciones):
    print(f"\nDATOS POSICIÓN NÚMERO {i+1}")
    L1, L2, q1, q2 = calcular_cables(x, y)
    print("Longitud del cable L1 =", L1, "cm")
    print("Longitud del cable L2 =", L2, "cm")
    print("Ángulo del cable L1 (q1) =", q1, "°")
    print("Ángulo del cable L2 (q2) =", q2, "°")

# SHOW MOVEMENT DATA BETWEEN POSITIONS
for i in range(num_posiciones - 1):
    print(f"\nLONGITUD DE CABLE ELONGADA / RECOGIDA Y ÁNGULO DE GIRO DE CADA POLEA ENTRE LAS POSICIONES {i+1} Y {i+2}")
    L1_inicial, L2_inicial, q1_inicial, q2_inicial = calcular_cables(posiciones[i][0], posiciones[i][1])
    L1_final, L2_final, q1_final, q2_final = calcular_cables(posiciones[i+1][0], posiciones[i+1][1])
    L1_movido = L1_final - L1_inicial
    L2_movido = L2_final - L2_inicial
    P1_movido_radianes = L1_movido / radio_rueda
    P2_movido_radianes = L2_movido / radio_rueda
    print("Longitud de cable elongada / recogida por el cable L1 =", L1_movido, "cm")
    print("Longitud de cable elongada / recogida por el cable L2 =", L2_movido, "cm")
    print("Ángulo girado por la polea P1 =", P1_movido_radianes, "radianes")
    print("Ángulo girado por la polea P1 =", np.degrees(P1_movido_radianes), "°")
    print("Ángulo girado por la polea P2 =", P2_movido_radianes, "radianes")
    print("Ángulo girado por la polea P2 =", np.degrees(P2_movido_radianes), "°")

# ANIMATION CREATION
ani = FuncAnimation(fig, 
                    animate, 
                    frames=100, 
                    init_func=init,
                    blit=True, 
                    interval=50, 
                    repeat=False)

plt.show()
