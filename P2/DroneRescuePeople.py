import HAL
import WebGUI
import Frequency
import numpy as np
import cv2

BORRAR_LINEA = "\r\033[K"  # vuelve al principio de la línea y la borra

# PASO 1: coordenadas, despegue y vuelo a la zona 

# GPS del enunciado pasado a UTM
BARCA_UTM = (430492.16, 4459162.06)
SUPERVIVIENTES_UTM = (430532.50, 4459131.78)  

def utm_to_local(este, norte):
    # Marco del mundo en Gazebo: x = Este, y = Norte, con origen en la barca
    return este - BARCA_UTM[0], norte - BARCA_UTM[1]


SUPERV_X, SUPERV_Y = utm_to_local(*SUPERVIVIENTES_UTM) 

ALTURA = 8.0        # altura de vuelo y de búsqueda (m)
TOL = 0.5             # distancia a la que damos un punto por alcanzado (m)
ORIENTACION = HAL.get_yaw()   # mantenemos la orientación inicial todo el vuelo


def go_to(x, y, z, buscar=False):

    objetivo = np.array([x, y, z])
    while np.linalg.norm(HAL.get_position() - objetivo) > TOL:
        HAL.set_cmd_pos(x, y, z, ORIENTACION)
        update_battery()
        if buscar:
            pos = HAL.get_position()  # posición del dron en el momento de la foto
            img = HAL.get_ventral_image().copy()
            for (u, v) in detect_and_display(img):
                save_person(*pixel_to_world(u, v, pos, img))
            if len(survivors()) == N_PERSONAS or bateria <= BATERIA_BAJA:
                return False
        else:
            WebGUI.showImage(HAL.get_ventral_image())
        Frequency.tick()
    return True


# PASO 2: búsqueda en espiral

CAMPO_VISION = 1.02974          # cámara ventral
HUELLA = 2 * ALTURA * np.tan(CAMPO_VISION / 2) * 240 / 320   # lado corto de lo que ve la cámara (m)
SEPARACION = 0.8 * HUELLA      # separación entre vueltas (20 % de solape)
RADIO_BUSQUEDA = 10.0          # media anchura de la zona a barrer (m)
DESPLAZAMIENTO_BUSQUEDA = (-7.0, -5.0)  # centro de la espiral respecto al punto GPS


def spiral(cx, cy):
    puntos = [(cx, cy)]
    direcciones = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    vueltas = int(np.ceil(RADIO_BUSQUEDA / SEPARACION))
    for tramo in range(4 * vueltas):  # vueltas completas 
        dx, dy = direcciones[tramo % 4]
        longitud = (tramo // 2 + 1) * SEPARACION
        x, y = puntos[-1]
        puntos.append((x + dx * longitud, y + dy * longitud))
    return puntos


# PASO 3: detección de caras

clasificador_caras = cv2.CascadeClassifier()

if not clasificador_caras.load("/resources/exercises/rescue_people/haarcascade_frontalface_default.xml"):
    print("--(!)Error loading face cascade")
    exit(0)

ANCHO_TRABAJO = 640                # ampliamos la imagen a este ancho
ANGULOS = range(0, 360, 30)  # Probamos la imagen girada


def detect_and_display(imagen):
    # Devuelve los centros (u, v) de las caras en píxeles de la imagen original
    imagen_gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
    
    escala = ANCHO_TRABAJO / imagen.shape[1]
    imagen_gris = cv2.resize(imagen_gris, None, fx=escala, fy=escala)
    # Rellenamos hasta un cuadrado de lado la diagonal para no perder las esquinas al girar
    h, w = imagen_gris.shape
    d = int(np.hypot(h, w))
    arriba, izquierda = (d - h) // 2, (d - w) // 2
    cuadrado = cv2.copyMakeBorder(imagen_gris, arriba, d - h - arriba, izquierda, d - w - izquierda,
                                cv2.BORDER_CONSTANT, value=int(imagen_gris.mean()))
    centros = []
    for angulo in ANGULOS:
        M = cv2.getRotationMatrix2D((d / 2, d / 2), angulo, 1.0)
        girada = cv2.warpAffine(cuadrado, M, (d, d))
        
        caras = clasificador_caras.detectMultiScale(girada, 1.1, 5)
        for (x, y, ancho_cara, alto_cara) in caras:
            # Centro de la cara en la imagen original: deshacemos el giro, el relleno y la ampliación
            u, v = cv2.invertAffineTransform(M) @ (x + ancho_cara / 2, y + alto_cara / 2, 1)
            centro = (int((u - izquierda) / escala), int((v - arriba) / escala))
            ejes = (int(ancho_cara / 2 / escala), int(alto_cara / 2 / escala))
            imagen = cv2.ellipse(imagen, centro, ejes, 0, 0, 360, (255, 0, 255), 2)
            centros.append(centro)
    WebGUI.showImage(imagen)
    return centros


# PASO 4: guardar cada persona una sola vez

N_PERSONAS = 6     # personas a encontrar antes de volver a la barca
DIST_MISMA_PERSONA = 2.0  # detecciones a menos de esta distancia son la misma persona
VECES_CONFIRMAR = 3      # veces que hay que ver una cara para darla por buena 
personas = []      # [x, y, veces vista] de cada cara en el mapa (x = Este, y = Norte)


def pixel_to_world(u, v, pos, img):
    # La cámara ventral mira hacia abajo: arriba en la imagen = morro del dron, izquierda = su izquierda
    h, w = img.shape[:2]
    m_por_px = 2 * pos[2] * np.tan(CAMPO_VISION / 2) / w  # metros que ocupa un píxel sobre el agua
    adelante = (h / 2 - v) * m_por_px
    izquierda = (w / 2 - u) * m_por_px
    return (pos[0] + adelante * np.cos(ORIENTACION) - izquierda * np.sin(ORIENTACION),
            pos[1] + adelante * np.sin(ORIENTACION) + izquierda * np.cos(ORIENTACION))


def save_person(x, y):
    # Si ya hay una cara guardada cerca es la misma persona: solo afinamos su posición. Si no, es una persona nueva
    for p in personas:
        if np.hypot(p[0] - x, p[1] - y) < DIST_MISMA_PERSONA:
            p[0] = (p[0] * p[2] + x) / (p[2] + 1)
            p[1] = (p[1] * p[2] + y) / (p[2] + 1)
            p[2] += 1
            if p[2] == VECES_CONFIRMAR:
                print(f"{BORRAR_LINEA}Persona {len(survivors())} encontrada")
            return
    personas.append([x, y, 1])


def survivors():
    # Personas confirmadas: vistas al menos VECES_CONFIRMAR veces
    return [p for p in personas if p[2] >= VECES_CONFIRMAR]


# PASO 5: batería simulada

CONSUMO_BATERIA = 0.5  # % de batería por metro recorrido 
BATERIA_BAJA = 40     # % al que vuelve a recargar
bateria = 100.0
ultima_pos = np.array(HAL.get_position()) 


def show_battery():
    # Escribe la batería en la última línea de la terminal, reescribiéndola en el sitio (sin saltar de línea)
    print(f"{BORRAR_LINEA}Batería: {bateria:.0f} %", end="", flush=True)


def update_battery():
    # La batería baja con los metros recorridos
    global bateria, ultima_pos
    pos = np.array(HAL.get_position())
    bateria -= CONSUMO_BATERIA * np.linalg.norm(pos - ultima_pos)
    ultima_pos = pos
    show_battery()


def recharge():
    # Vuelve a la barca, aterriza para "recargar" y regresa al punto donde se quedó buscando
    global bateria
    pos_reanudar = HAL.get_position()
    print(f"{BORRAR_LINEA}Batería baja ({bateria:.0f} %): vuelta a la barca para recargar")
    go_to(0, 0, ALTURA)
    HAL.land()
    update_battery()
    while bateria < 100:  # recarga en tierra: sube un 20 % por segundo
        bateria = min(100.0, bateria + 0.4)
        show_battery()
        WebGUI.showImage(HAL.get_ventral_image())
        Frequency.tick()
    print(f"{BORRAR_LINEA}Batería al 100 %, volviendo a ({pos_reanudar[0]:.1f}, {pos_reanudar[1]:.1f}) para seguir buscando")
    HAL.takeoff(ALTURA)
    go_to(pos_reanudar[0], pos_reanudar[1], ALTURA)


# SECUENCIA

HAL.takeoff(ALTURA)
go_to(0, 0, ALTURA)

cx, cy = SUPERV_X + DESPLAZAMIENTO_BUSQUEDA[0], SUPERV_Y + DESPLAZAMIENTO_BUSQUEDA[1]
puntos_paso = spiral(cx, cy)
go_to(cx, cy, ALTURA)  # hasta el centro de la espiral sin buscar
i = 0
while i < len(puntos_paso) and len(survivors()) < N_PERSONAS:
    x, y = puntos_paso[i]
    if go_to(x, y, ALTURA, buscar=True):
        i += 1
    elif bateria <= BATERIA_BAJA:
        recharge()  # al volver, se repite el mismo punto: sigue el tramo donde lo dejó

print(f"{BORRAR_LINEA}Búsqueda terminada")

go_to(0, 0, ALTURA)  # vuelta a la barca
HAL.land()
print(f"{BORRAR_LINEA}Aterrizado en la barca")
update_battery()  # la batería final queda en la última línea
print()
