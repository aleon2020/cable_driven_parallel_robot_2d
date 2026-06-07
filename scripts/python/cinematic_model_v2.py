import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.path import Path
import matplotlib.patches as patches

# PARAMETERS
largo_plano = 100
alto_plano = 100
largo_efector = 10
alto_efector = 10
radio_rueda = 5

# REQUEST FOR THE COORDINATES OF THE END-EFFECTOR
print("\nCOORDENADAS INICIALES DEL EFECTOR FINAL")
x_inicial = float(input('Coordenada x inicial del efector final: '))
y_inicial = float(input('Coordenada y inicial del efector final: '))
print("\nCOORDENADAS FINALES DEL EFECTOR FINAL")
x_final = float(input('Coordenada x final del efector final: '))
y_final = float(input('Coordenada y final del efector final: '))

# LIMIT VERIFICATION
def verificar_limites(x, y):
    error_msg = ""
    if x >= (largo_plano - (largo_efector / 2)) or x <= (largo_efector / 2):
        error_msg += f"\nFUERA DEL LÍMITE en x = {x}"
    if y >= (alto_plano - (alto_efector / 2)) or y <= (alto_efector / 2):
        error_msg += f"\nFUERA DEL LÍMITE en y = {y}"
    return error_msg
error_inicial = verificar_limites(x_inicial, y_inicial)
if error_inicial:
    print("\nERROR EN LA POSICIÓN INICIAL" + error_inicial)
error_final = verificar_limites(x_final, y_final)
if error_final:
    print("\nERROR EN LA POSICIÓN FINAL" + error_final)
if error_inicial or error_final:
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

# ANIMATION
def animate(i):
    
    # FRAMES
    t = i / 100
    x = x_inicial + (x_final - x_inicial) * t
    y = y_inicial + (y_final - y_inicial) * t
    
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

# ANIMATION CREATION
ani = FuncAnimation(fig, 
                    animate, 
                    frames=100, 
                    init_func=init,
                    blit=True, 
                    interval=50, 
                    repeat=False)

# LENGTH AND ANGLE OF EACH CABLE IN THE INITIAL POSITION
print("\nDATOS POSICIÓN INICIAL")
L1_inicial, L2_inicial, q1_inicial, q2_inicial = calcular_cables(x_inicial, y_inicial)
print("Longitud del cable L1 =", L1_inicial, "cm")
print("Longitud del cable L2 =", L2_inicial, "cm")
print("Ángulo del cable L1 (q1) =", q1_inicial, "°")
print("Ángulo del cable L2 (q2) =", q2_inicial, "°")

# LENGTH AND ANGLE OF EACH CABLE IN THE FINAL POSITION
print("\nDATOS POSICIÓN FINAL")
L1_final, L2_final, q1_final, q2_final = calcular_cables(x_final, y_final)
print("Longitud del cable L1 =", L1_final, "cm")
print("Longitud del cable L2 =", L2_final, "cm")
print("Ángulo del cable L1 (q1) =", q1_final, "°")
print("Ángulo del cable L2 (q2) =", q2_final, "°")

# CABLE LENGTH EXTENDED / RETRACTED AND TURNING ANGLE OF EACH PULLEY
print("\nLONGITUD DE CABLE ELONGADA / RECOGIDA Y ÁNGULO DE GIRO DE CADA POLEA")
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

plt.show()
