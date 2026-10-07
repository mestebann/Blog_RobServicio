import HAL
import WebGUI
import numpy as np
import time
import Frequency

# PASO 1: registro de los puntos y relacion entre Gazebo (metros) y el mapa (pixeles)

# Puntos de calibracion: [x_gazebo, y_gazebo, columna, fila]
# (x, y los leemos de la pose del robot y la columna y fila las estimamos en el mapa)
PUNTOS_CALIBRACION = [[-1.000,  1.500, 664, 538],
                      [ 2.422,  2.647, 317, 675],
                      [ 3.449,  5.174, 210, 928],
                      [-0.956,  5.374, 656, 942],
                      [ 3.871,  2.728, 176, 678],
                      [ 3.220, -0.334, 237, 355],
                      [-3.643,  0.286, 921, 429],
                      [-2.250,  2.929, 790, 708],
                      [ 0.190, -0.249, 537, 384],
                      [ 4.581, -2.298, 100, 161]]

# Queremos columna = a*x + b*y + c  y  fila = d*x + e*y + f
# Montamos el sistema: una fila [x, y, 1] por cada punto
matriz_sistema = []
columnas_medidas = []
filas_medidas = []
for punto in PUNTOS_CALIBRACION:
    matriz_sistema.append([punto[0], punto[1], 1])
    columnas_medidas.append(punto[2])
    filas_medidas.append(punto[3])
matriz_sistema = np.array(matriz_sistema)

# Minimos cuadrados con las ecuaciones normales
traspuesta = matriz_sistema.T
inversa = np.linalg.inv(traspuesta @ matriz_sistema)
coef_col = inversa @ traspuesta @ columnas_medidas
coef_fila = inversa @ traspuesta @ filas_medidas

# Correccion de la traslacion que medimos con el laser (en pixeles). 
coef_col[2] = coef_col[2] + 15
coef_fila[2] = coef_fila[2] + 20


# Pasa un punto de Gazebo (en metros) a pixeles del mapa
def gazebo_a_pixel(x_gazebo, y_gazebo):
    columna = coef_col[0] * x_gazebo + coef_col[1] * y_gazebo + coef_col[2]
    fila = coef_fila[0] * x_gazebo + coef_fila[1] * y_gazebo + coef_fila[2]
    return columna, fila


# PASO 2: rejilla de ocupacion 
LADO_CELDA = 0.30               # lado de la celda (m), un poco menor que el robot
MINIMO_PIXELES_NEGROS = 20      # si la celda tiene al menos estos pixeles negros, esta ocupada
MARGEN = 8                      # pixeles que miramos alrededor de cada celda (1 px = 1 cm)
X_ESQUINA_REJILLA = -4.43       # esquina de la celda (0, 0) en Gazebo
Y_ESQUINA_REJILLA = -3.55
CELDAS_EN_X = 33                # numero de celdas en x y en y: cubren toda la casa
CELDAS_EN_Y = 32                # sin salirse de la imagen
# Casillas que ponemos ocupadas a mano
CASILLAS_BLOQUEADAS = [(15, 28)]

mapa = WebGUI.getMap('/workspace/code/mapgrannyannie.png')     
pixeles_negros = mapa[:, :, 0] < 128        # True en los pixeles negros (obstaculos)
celda_ocupada = np.zeros((CELDAS_EN_X, CELDAS_EN_Y), dtype=bool)


# Devuelve el rectangulo de pixeles de la celda (i, j): pasamos
# sus 4 esquinas a pixeles y nos quedamos con el minimo y el maximo
def pixeles_celda(i, j):
    x_inicio = X_ESQUINA_REJILLA + i * LADO_CELDA
    x_final = x_inicio + LADO_CELDA
    y_inicio = Y_ESQUINA_REJILLA + j * LADO_CELDA
    y_final = y_inicio + LADO_CELDA
    columna_1, fila_1 = gazebo_a_pixel(x_inicio, y_inicio)
    columna_2, fila_2 = gazebo_a_pixel(x_final, y_inicio)
    columna_3, fila_3 = gazebo_a_pixel(x_inicio, y_final)
    columna_4, fila_4 = gazebo_a_pixel(x_final, y_final)
    col_min = int(round(min(columna_1, columna_2, columna_3, columna_4)))
    col_max = int(round(max(columna_1, columna_2, columna_3, columna_4)))
    fila_min = int(round(min(fila_1, fila_2, fila_3, fila_4)))
    fila_max = int(round(max(fila_1, fila_2, fila_3, fila_4)))
    return col_min, col_max, fila_min, fila_max


for i in range(CELDAS_EN_X):
    for j in range(CELDAS_EN_Y):
        col_min, col_max, fila_min, fila_max = pixeles_celda(i, j)
        # Contamos los pixeles negros de la celda y de una franja de MARGEN
        # pixeles a su alrededor, para que el robot no pase pegado a las paredes
        n_negros = np.sum(pixeles_negros[fila_min - MARGEN:fila_max + MARGEN,
                                         col_min - MARGEN:col_max + MARGEN])
        if n_negros >= MINIMO_PIXELES_NEGROS or (i, j) in CASILLAS_BLOQUEADAS:
            celda_ocupada[i][j] = True
            mapa[fila_min:fila_max, col_min:col_max] = 0     # celda ocupada: la pintamos negra
        else:
            mapa[fila_min, col_min:col_max] = 0              # celda libre: borde de arriba
            mapa[fila_min:fila_max, col_min] = 0             # y borde de la izquierda


# PASO 3: planificacion de la ruta con BSA y navegación

# Direcciones de movimiento entre celdas (cambio en i, cambio en j)
OESTE = (1, 0)
NORTE = (0, -1)
ESTE = (-1, 0)
SUR = (0, 1)
# Orden en el que el robot va cambiando de direccion cuando se topa con algo
DIRECCIONES = [OESTE, NORTE, ESTE, SUR]

# Colores
VERDE = (0, 200, 0)       # celda visitada
AZUL = (255, 0, 0)        # punto de retorno al que hemos vuelto
ROJO = (0, 0, 255)        # punto critico


# Pinta una celda y le vuelve a pintar el borde (las celdas vecinas pueden
# solaparse un pixel y al pintar una se borraria el borde de la otra)
def pintar(celda, color):
    col_min, col_max, fila_min, fila_max = pixeles_celda(celda[0], celda[1])
    mapa[fila_min:fila_max, col_min:col_max] = color
    mapa[fila_min, col_min:col_max] = 0          # borde de arriba
    mapa[fila_max - 1, col_min:col_max] = 0      # borde de abajo
    mapa[fila_min:fila_max, col_min] = 0         # borde izquierdo
    mapa[fila_min:fila_max, col_max - 1] = 0     # borde derecho


# Devuelve las celdas vecinas que no estan bloqueadas, en el orden de 'direcciones'
def vecinas_libres(celda, bloqueada, direcciones):
    libres = []
    for d in direcciones:
        vecina = (celda[0] + d[0], celda[1] + d[1])
        if not bloqueada[vecina[0]][vecina[1]]:
            libres.append(vecina)
    return libres


# Busqueda en anchura (BFS): camino mas corto por celdas libres desde
# 'inicio' hasta el destino mas cercano de la lista 'destinos'
def camino_bfs(inicio, destinos):
    cola = [inicio]
    padre = {inicio: None}             # para cada celda, desde que celda llegamos
    while len(cola) > 0:
        celda = cola.pop(0)            # sacamos la primera de la cola
        if celda in destinos:
            break                      # es el destino mas cercano: paramos
        for vecina in vecinas_libres(celda, celda_ocupada, DIRECCIONES):
            if vecina not in padre:    # si no la habiamos visto, la apuntamos
                padre[vecina] = celda
                cola.append(vecina)
    # Reconstruimos el camino hacia atras, desde el destino hasta el inicio
    camino = []
    while celda is not None:
        camino.insert(0, celda)        # la metemos al principio para que quede en orden
        celda = padre[celda]
    return camino


# Comprueba si se puede ir en linea recta del centro de celda_origen al de
# celda_destino sin pisar celdas ocupadas
def se_ve(celda_origen, celda_destino):
    diferencia_x = celda_destino[0] - celda_origen[0]
    diferencia_y = celda_destino[1] - celda_origen[1]
    distancia = np.sqrt(diferencia_x ** 2 + diferencia_y ** 2)   # en celdas
    pasos = int(10 * distancia) + 1    # unos 10 puntos por celda
    for k in range(pasos + 1):
        # Punto de la recta entre los dos centros (en unidades de celda)
        x_punto = celda_origen[0] + 0.5 + diferencia_x * k / pasos
        y_punto = celda_origen[1] + 0.5 + diferencia_y * k / pasos
        # Miramos las 4 esquinas de un cuadrado de casi una celda alrededor
        # del punto, porque el robot es ancho
        for desplazamiento_x in (-0.45, 0.45):
            for desplazamiento_y in (-0.45, 0.45):
                if celda_ocupada[int(x_punto + desplazamiento_x)][int(y_punto + desplazamiento_y)]:
                    return False
    return True


# Quita los puntos que sobran del camino: desde cada punto vamos al mas
# lejano que se ve en linea recta. Solo quedan los puntos donde hay que girar
def camino_directo(camino):
    puntos = [camino[0]]
    k = 0
    while k < len(camino) - 1:
        siguiente = k + 1
        for m in range(k + 1, len(camino)):
            if se_ve(camino[k], camino[m]):
                siguiente = m
        puntos.append(camino[siguiente])
        k = siguiente
    return puntos


# Algoritmo BSA

# Celda de la posicion inicial del robot (-1.0, 1.5)
celda_actual = (int((-1.0 - X_ESQUINA_REJILLA) / LADO_CELDA),
                int((1.5 - Y_ESQUINA_REJILLA) / LADO_CELDA))
direccion = OESTE                  # el robot empieza mirando al oeste (yaw = 0)
celda_bloqueada = celda_ocupada.copy()   # ocupadas + visitadas (las visitadas son obstaculos virtuales)
celda_bloqueada[celda_actual[0]][celda_actual[1]] = True
pintar(celda_actual, VERDE)
puntos_retorno = []                # posibles puntos de retorno
plan = [celda_actual]              # celdas que recorrera el robot, en orden

while True:
    WebGUI.showNumpy(mapa)
    time.sleep(0.1)                # para ver despacio como se rellena la rejilla

    # Recorremos la lista DIRECCIONES empezando por la direccion actual
    k = DIRECCIONES.index(direccion)
    orden_prioridad = DIRECCIONES[k:] + DIRECCIONES[:k]
    libres = vecinas_libres(celda_actual, celda_bloqueada, orden_prioridad)

    if len(libres) > 0:
        # Las vecinas libres que no elegimos son posibles puntos de retorno
        for celda in libres[1:]:
            if celda not in puntos_retorno:
                puntos_retorno.append(celda)
        # Avanzamos a la vecina con mas prioridad
        celda_siguiente = libres[0]
        direccion = (celda_siguiente[0] - celda_actual[0], celda_siguiente[1] - celda_actual[1])
        celda_actual = celda_siguiente
        plan.append(celda_actual)
        pintar(celda_actual, VERDE)
    else:
        # Punto critico: no queda ninguna vecina libre
        pintar(celda_actual, ROJO)
        # Nos quedamos solo con los puntos de retorno que siguen sin visitar
        retornos_pendientes = []
        for celda in puntos_retorno:
            if not celda_bloqueada[celda[0]][celda[1]]:
                retornos_pendientes.append(celda)
        puntos_retorno = retornos_pendientes
        if len(puntos_retorno) == 0:
            break                  # ya no queda nada por limpiar
        # Vamos al punto de retorno mas cercano, en linea recta si se puede
        tramo_vuelta = camino_directo(camino_bfs(celda_actual, puntos_retorno))
        plan = plan + tramo_vuelta[1:]
        celda_actual = tramo_vuelta[-1]
        pintar(celda_actual, AZUL)

    celda_bloqueada[celda_actual[0]][celda_actual[1]] = True      # la nueva celda ya esta visitada


# NAVEGACIÓN

VELOCIDAD_MAXIMA = 0.4    # velocidad lineal maxima (m/s)
KP = 1.0                  # ganancia del control de orientacion
ANGULO_MAX = 0.1          # si el error de orientacion es mayor (rad), gira sin avanzar
GIRO_MINIMO = 0.3         # velocidad minima de giro en el sitio (rad/s): con menos el robot no llega a girar
TOLERANCIA = 0.1         # a esta distancia (m) damos el punto por alcanzado
DIST_PARED = 0.25         # si el laser ve una pared delante a menos de esto (m), no seguimos avanzando


# Centro de una celda en metros de Gazebo
def centro(celda):
    x_centro = X_ESQUINA_REJILLA + (celda[0] + 0.5) * LADO_CELDA
    y_centro = Y_ESQUINA_REJILLA + (celda[1] + 0.5) * LADO_CELDA
    return x_centro, y_centro


# Nos quedamos solo con los puntos del plan donde el robot tiene que girar.
ruta = []
for k in range(1, len(plan) - 1):
    paso_anterior = (plan[k][0] - plan[k - 1][0], plan[k][1] - plan[k - 1][1])
    paso_siguiente = (plan[k + 1][0] - plan[k][0], plan[k + 1][1] - plan[k][1])
    if paso_anterior != paso_siguiente:
        ruta.append(plan[k])
ruta.append(plan[-1])

n = 0                     # punto de la ruta al que vamos
while n < len(ruta):
    pose = HAL.getPose3d()

    # Distancia y angulo hasta el centro de la celda objetivo
    x_objetivo, y_objetivo = centro(ruta[n])
    diferencia_x = x_objetivo - pose.x
    diferencia_y = y_objetivo - pose.y
    distancia = np.sqrt(diferencia_x ** 2 + diferencia_y ** 2)

    # Error de orientacion: hacia donde deberia mirar menos hacia donde mira
    error_orientacion = np.arctan2(diferencia_y, diferencia_x) - pose.yaw
    # Lo dejamos entre -pi y pi para que gire siempre por el lado corto
    if error_orientacion > np.pi:
        error_orientacion = error_orientacion - 2 * np.pi
    if error_orientacion < -np.pi:
        error_orientacion = error_orientacion + 2 * np.pi

    # Distancia a lo que tiene el robot justo delante segun el laser
    distancia_delante = HAL.getLaserData().values[90]

    if distancia < TOLERANCIA:
        n = n + 1         # hemos llegado cerca del centro: siguiente punto
    elif abs(error_orientacion) <= ANGULO_MAX and distancia_delante < DIST_PARED:
        # Vamos hacia el punto pero hay una pared delante: no podemos
        # acercarnos mas, asi que paramos, lo damos por alcanzado y seguimos
        HAL.setV(0)
        n = n + 1
    elif abs(error_orientacion) > ANGULO_MAX:
        # Esta desviado: gira en el sitio, como minimo a GIRO_MINIMO
        HAL.setV(0)
        if error_orientacion > 0:
            HAL.setW(max(KP * error_orientacion, GIRO_MINIMO))
        else:
            HAL.setW(min(KP * error_orientacion, -GIRO_MINIMO))
    else:
        # Bien orientado: avanza corrigiendo el rumbo y frena al acercarse al punto
        HAL.setV(min(VELOCIDAD_MAXIMA, distancia))
        HAL.setW(KP * error_orientacion)

    Frequency.tick(20)        

# Plan terminado: paramos el robot
HAL.setV(0)
HAL.setW(0)
print("Recorrido terminado")

while True:
    WebGUI.showNumpy(mapa)
    Frequency.tick(1)
