import math
import HAL
import Frequency

# Busqueda del hueco
V = 0.8                        
DIST_LATERAL = 1.6             
LARGO_HUECO = 5.5              
ANCHO_HUECO = 2.0              
K_LATERAL = 0.5                # rad de giro por cada metro de error lateral
K_YAW = 1.5                    # ganancia del control de orientacion
GIRO_MAX = math.radians(20)    

# Maniobra
V_MANIOBRA = 0.3               
V_CENTRAR = 0.3                
ESPACIO_MANIOBRA = 11.5        
ANGULO_ENTRADA = math.radians(30)  
VOLANTE = 1.0                  # volante a tope
SEGURIDAD = 0.6                # distancia mínima de los parachoques
FONDO_MAX = 2.2                
FONDO = 1.05                   # donde queremos el centro del coche por detras de la fila
ANGULO_AJUSTE = math.radians(6)    
RECTO = math.radians(1)        

# LIDAR
ALTURA_LIDAR = 1.62           
LIDAR_ATRAS = 1.25             
ALTURA_MIN = 0.3              
ALTURA_MAX = 1.5           

# Medidas del coche
DELANTERA = 2.19               
TRASERA = 2.43                
MEDIO_ANCHO = 0.92             


def no_imprimir(*args, **kwargs):
    """Sustituye al print del HAL, que imprime los datos del LIDAR en cada
    lectura y llena la terminal"""
    pass


HAL.print = no_imprimir


def normalizar(angulo):
    """Lleva un angulo al intervalo [-pi, pi]"""
    return math.atan2(math.sin(angulo), math.cos(angulo))


def posicion_lateral():
    """Posicion del coche perpendicular a la calle (positivo = izquierda)"""
    pose = HAL.getPose3d()
    return -pose.x * math.sin(yaw_calle) + pose.y * math.cos(yaw_calle)


def posicion_calle():
    """Posicion del coche a lo largo de la calle (positivo = hacia delante)"""
    pose = HAL.getPose3d()
    return pose.x * math.cos(yaw_calle) + pose.y * math.sin(yaw_calle)


def angulo_calle():
    """Giro del coche respecto a la calle (positivo = morro hacia la izquierda"""
    return normalizar(HAL.getPose3d().yaw - yaw_calle)


def puntos_coche():
    """Puntos del LIDAR a la altura de los coches, como (adelante, derecha),
    en metros, con los ejes del coche y el origen en el centro del coche."""
    lidar = HAL.getLidarData()
    puntos = []
    for x, y, z in lidar.points[::2]:      # se queda con uno de cada dos puntos
        # El LIDAR va girado 90 grados: su x apunta a la derecha del coche, su y hacia delante, y z es la altura respecto al propio LIDAR
        altura = z + ALTURA_LIDAR
        if ALTURA_MIN < altura < ALTURA_MAX:
            adelante = y - LIDAR_ATRAS
            derecha = x
            # El LIDAR tambien ve partes de nuestro propio coche, esas se quitan
            en_el_coche = False
            if -TRASERA - 0.1 < adelante < DELANTERA + 0.1:
                if abs(derecha) < MEDIO_ANCHO + 0.15:
                    en_el_coche = True
            if not en_el_coche:
                puntos.append((adelante, derecha))
    return puntos


def puntos_calle():
    """Los mismos puntos girados a los ejes de la calle: (delante, derecha)"""
    giro = angulo_calle()
    cos_giro = math.cos(giro)
    sin_giro = math.sin(giro)
    puntos = []
    for adelante, derecha_coche in puntos_coche():
        # Giramos el punto de los ejes del coche a los de la calle para no torcer el rectángulo
        delante = adelante * cos_giro + derecha_coche * sin_giro
        derecha = derecha_coche * cos_giro - adelante * sin_giro
        puntos.append((delante, derecha))
    return puntos


def en_la_fila(derecha):
    """True si un punto esta en la franja de los coches aparcados"""
    if DIST_LATERAL - 0.5 < derecha < DIST_LATERAL + ANCHO_HUECO:
        return True
    else:
        return False


def hueco_libre():
    """True si no hay ningun punto dentro del rectangulo de la derecha.
    Solo cuenta lo que cae dentro, asi que da igual que haya coche delante,
    detras, los dos, o una pared al fondo"""
    for delante, derecha in puntos_calle():
        if abs(delante) < LARGO_HUECO / 2 and en_la_fila(derecha):
            return False
    return True


def inicio_del_hueco():
    """Posicion a lo largo de la calle del morro del coche de detras.
    Si no se ve ninguno, el principio del rectangulo libre"""
    mas_cerca = -LARGO_HUECO / 2    # valor por defecto por si no se ve ningún coche detrás
    vistos = []
    for delante, derecha in puntos_calle():
        if delante < 0 and en_la_fila(derecha):
            vistos.append(delante)
    if vistos:
        mas_cerca = max(vistos)     # el punto del coche  de detrás más cercano
    return posicion_calle() + mas_cerca


def distancias_pasillo():
    """Distancia de cada parachoques (delantero y trasero) a lo mas cercano
    que hay en el pasillo que ocupa el coche. Lee el LIDAR una sola vez"""
    delante = 15.0
    detras = 15.0
    for adelante, derecha in puntos_coche():
        if abs(derecha) < MEDIO_ANCHO + 0.1:
            if adelante > DELANTERA:
                delante = min(delante, adelante - DELANTERA)
            elif adelante < -TRASERA:
                detras = min(detras, -adelante - TRASERA)
    return delante, detras


def esquina_trasera():
    """Posicion lateral de la esquina trasera derecha"""
    a = angulo_calle()
    return posicion_lateral() - TRASERA * math.sin(a) - MEDIO_ANCHO * math.cos(a)


def esquina_delantera():
    """Posicion lateral de la esquina delantera derecha"""
    a = angulo_calle()
    return posicion_lateral() + DELANTERA * math.sin(a) - MEDIO_ANCHO * math.cos(a)


def detras_bloqueado():
    """True si hay algo cerca detras o la esquina trasera llega al fondo del hueco"""
    delante, detras = distancias_pasillo()
    if detras < SEGURIDAD or esquina_trasera() < lateral_fondo:
        return True
    else:
        return False


def seguir_carril():
    """Avanza paralelo a la calle y se va colocando a la distancia lateral
    deseada. Devuelve el error lateral en metros"""
    error = posicion_lateral() - lateral_objetivo    # > 0: hay que ir a la derecha
    angulo = max(-GIRO_MAX, min(GIRO_MAX, K_LATERAL * error))   # se limita el angulo entre -20º y 20º
    yaw_deseado = yaw_calle - angulo
    HAL.setV(V)
    HAL.setW(K_YAW * normalizar(yaw_deseado - HAL.getPose3d().yaw))
    return error

# La calle va a lo largo de un eje del mundo: el mas cercano a nuestra orientacion
yaw_calle = round(HAL.getPose3d().yaw / (math.pi / 2)) * (math.pi / 2)

# Distancias hacia la derecha de los puntos que estan a menos de 15 m por delante o por detras del coche
cercanos = []
for delante, derecha in puntos_calle():
    if abs(delante) < 15 and derecha > 0:
        cercanos.append(derecha)

# La mas pequeña es la del lateral de los coches aparcados
distancia = min(cercanos)

lateral_objetivo = posicion_lateral() - (distancia - DIST_LATERAL)
print("Coches de la derecha a %.2f m, me acerco a %.2f m" % (distancia, DIST_LATERAL))

estado = "ALINEAR"
while True:
    if estado == "ALINEAR":
        error = seguir_carril()
        if abs(error) < 0.1:
            print("Alineado. Buscando hueco...")
            estado = "BUSCAR_HUECO"

    elif estado == "BUSCAR_HUECO":
        seguir_carril()
        if hueco_libre():
            inicio_hueco = inicio_del_hueco()
            print("Hueco encontrado")
            estado = "COLOCARSE"

    elif estado == "COLOCARSE":
        # Avanzar hasta dejar detras el espacio que necesita la maniobra
        seguir_carril()
        if posicion_calle() > inicio_hueco + ESPACIO_MANIOBRA:
            lateral_fondo = posicion_lateral() - (DIST_LATERAL + FONDO_MAX)
            lateral_aparcado = posicion_lateral() - (DIST_LATERAL + FONDO)
            ajustes = 0
            print("Empiezo la maniobra")
            estado = "ATRAS_DERECHA"

    elif estado == "ATRAS_DERECHA":
        # Marcha atras con el volante a tope a la derecha: la trasera entra en el hueco
        HAL.setV(-V_MANIOBRA)
        HAL.setW(VOLANTE)
        if angulo_calle() > ANGULO_ENTRADA or detras_bloqueado():
            estado = "ATRAS_IZQUIERDA"

    elif estado == "ATRAS_IZQUIERDA":
        # Marcha atras con el volante a tope a la izquierda: el coche se endereza
        HAL.setV(-V_MANIOBRA)
        HAL.setW(-VOLANTE)
        if angulo_calle() < RECTO:      # con menos de 1º ya esta recto
            estado = "COMPROBAR"
        elif detras_bloqueado():
            estado = "ADELANTE_DERECHA"

    elif estado == "ADELANTE_DERECHA":
        # Hacia delante con el volante a la derecha: sigue enderezandose
        HAL.setV(V_MANIOBRA)
        HAL.setW(-VOLANTE)
        delante, detras = distancias_pasillo()
        if angulo_calle() < RECTO:
            estado = "COMPROBAR"
        elif delante < SEGURIDAD:
            estado = "ATRAS_IZQUIERDA"

    elif estado == "COMPROBAR":
        # Ya esta recto: si se ha quedado fuera, hace una ese hacia delante para meterse
        if posicion_lateral() > lateral_aparcado + 0.1 and ajustes < 3:
            ajustes = ajustes + 1
            print("Me meto un poco mas")
            estado = "AJUSTE_ATRAS"
        else:
            estado = "CENTRAR"

    elif estado == "AJUSTE_ATRAS":
        # Marcha atras recto hasta el fondo del hueco, para tener sitio delante
        HAL.setV(-V_MANIOBRA)
        HAL.setW(K_YAW * normalizar(yaw_calle - HAL.getPose3d().yaw))
        if detras_bloqueado():
            estado = "AJUSTE_DERECHA"

    elif estado == "AJUSTE_DERECHA":
        # Primera mitad de la ese: adelante con el volante a la derecha
        HAL.setV(V_CENTRAR)
        HAL.setW(-VOLANTE)
        delante, detras = distancias_pasillo()
        if (angulo_calle() < -ANGULO_AJUSTE or delante < SEGURIDAD
                or esquina_delantera() < lateral_fondo):
            estado = "AJUSTE_IZQUIERDA"

    elif estado == "AJUSTE_IZQUIERDA":
        # Segunda mitad de la ese: adelante con el volante a la izquierda hasta quedar recto
        HAL.setV(V_CENTRAR)
        HAL.setW(VOLANTE)
        delante, detras = distancias_pasillo()
        if angulo_calle() > 0 or delante < SEGURIDAD:
            estado = "COMPROBAR"

    elif estado == "CENTRAR":
        # Moverse recto hasta tener el mismo espacio delante que detras
        # (como mucho se cuentan 3 m, por si no hay coche en algun lado)
        delante, detras = distancias_pasillo()
        hueco_delante = min(delante, 3.0)
        hueco_detras = min(detras, 3.0)
        recto = K_YAW * normalizar(yaw_calle - HAL.getPose3d().yaw)
        if abs(hueco_delante - hueco_detras) < 0.2:
            print("Aparcado")
            estado = "APARCADO"
        elif hueco_delante > hueco_detras:
            HAL.setV(V_CENTRAR)
            HAL.setW(recto)
        else:
            HAL.setV(-V_CENTRAR)
            HAL.setW(recto)

    elif estado == "APARCADO":
        HAL.setV(0)
        HAL.setW(0)

    Frequency.tick()
