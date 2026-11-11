import math
import numpy as np
import cv2
import HAL
import WebGUI
import Frequency
from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou

CASE = 3

# PARAMETROS
# Medidas en metros (las de los robots, del modelo URDF/Xacro)
ROBOT_HALF = 0.275          # holonomico: 0.55 x 0.55
CAR_HALF_LENGTH = 0.58      # Ackermann: 1.15 x 0.53
CAR_HALF_WIDTH = 0.27
REAR_AXLE = 0.244           # del centro del Ackermann a su eje trasero
SHELF_HALF_LENGTH = 1.03    # estanteria: 2.06 x 0.88
SHELF_HALF_WIDTH = 0.44

ROBOT_RADIUS = 0.39         # caso 1: circulo que envuelve al robot
SHELF_RADIUS = 1.1          # caso 1: circulo que envuelve la estanteria
MARGIN = 0.1                # caso 1: margen al engordar
CAR_MARGIN = 0.15           # caso 3: margen del rectangulo (se desvia algo)
TURN_RADIUS = 1.3           # caso 3: radio de giro (el minimo es 0.82)

PLAN_TIME = 3.0             # s que piensa OMPL
PLAN_TRIES = 5              # intentos de planificar antes de rendirse
APPROACH_DIST = 2.04        # m del punto de espera al destino
APPROACH_CAR = 3.0          # lo mismo con el Ackermann

V_MAX = 0.5                 # m/s del holonomico sin carga
W_MAX = 1.0                 # rad/s del holonomico sin carga
V_LOADED = 0.3              # con la estanteria encima
W_LOADED = 0.5
V_SLOW = 0.2                # al entrar en el destino y al salir
V_MIN = 0.05
V_CAR = 0.2                 # m/s del Ackermann
V_START = 0.01              # Ackermann al empezar cada tramo
RAMP_TIME = 3.0             # s que tarda en llegar a V_CAR
K_W = 1.5                   # ganancia del giro
ANGLE_TOL = 0.05            # rad
DIST_TOL = 0.03             # m
LOOKAHEAD = 0.4             # m, punto al que apunta el holonomico
LOOKAHEAD_CAR = 0.6         # m, punto al que apunta el Ackermann
WAIT_TIME = 2.0             # s para que suba o baje la plataforma

# Medio largo y medio ancho del robot de este caso
if CASE == 3:
    HALF_LENGTH, HALF_WIDTH = CAR_HALF_LENGTH, CAR_HALF_WIDTH
else:
    HALF_LENGTH, HALF_WIDTH = ROBOT_HALF, ROBOT_HALF

# Estanterias (x, y, yaw). Se actualiza al moverlas.
shelves = [[3.84, 0.579, 0.0], [3.84, -1.243, 0.0], [3.84, -3.039, 0.0],
           [3.84, -4.827, 0.0], [3.84, -6.751, 0.0], [3.84, -8.665, 0.0]]

# Traslados: (estanteria, (x, y, yaw) del robot al dejarla). Las 4, 5 y 6
# van a los huecos de las 1, 2 y 3. OMPL no acepta yaw = pi: se usa -pi.
MISSIONS = [
    (1, (-1.5, -0.2, -math.pi)),
    (2, (-1.5, -2.0, -math.pi)),
    (3, (-1.5, -3.8, -math.pi)),
    (4, (3.84, 0.579, 0.0)),
    (5, (3.84, -1.243, 0.0)),
    (6, (3.84, -3.039, 0.0)),
]

# MAPA 
# 415 x 279 pixeles de 0.05 m. El eje x va hacia arriba y el y a la izquierda.
RES = 0.05
X_TOP = 7.02
Y_LEFT = 10.35
rgb = WebGUI.getMap("/resources/exercises/amazon_warehouse/images/map.png")
gray = np.mean(rgb, axis=2)
occupied = (gray < 0.9).astype(np.uint8)  # 1 = obstaculo


def world_to_pixel(x, y):
    return int((Y_LEFT - y) / RES), int((X_TOP - x) / RES)


def to_world(x, y, yaw, dx, dy):
    """Pasa el punto (dx, dy) de un robot en (x, y, yaw) al mundo."""
    return (x + dx * math.cos(yaw) - dy * math.sin(yaw),
            y + dx * math.sin(yaw) + dy * math.cos(yaw))


def ahead(pose, d):
    """La pose (x, y, yaw) movida d metros hacia donde mira."""
    x, y = to_world(pose[0], pose[1], pose[2], d, 0)
    return x, y, pose[2]


def inflate(radius):
    """Mapa con los obstaculos engordados radius metros."""
    size = 2 * round(radius / RES) + 1
    circle = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    return cv2.dilate(occupied, circle)


def remove_shelf(x, y):
    """Borra la estanteria: sus 4 patas caen a menos de 1.2 m (24 px)."""
    cv2.circle(occupied, world_to_pixel(x, y), 24, 0, -1)


def add_shelf(x, y, yaw):
    """Dibuja las 4 patas de la estanteria."""
    for dx, dy in [(0.99, 0.4), (0.99, -0.4), (-0.99, 0.4), (-0.99, -0.4)]:
        leg_x, leg_y = to_world(x, y, yaw, dx, dy)
        cv2.circle(occupied, world_to_pixel(leg_x, leg_y), 1, 1, -1)


def draw_rectangle(img, x, y, yaw):
    """Dibuja el contorno del robot en naranja."""
    corners = []
    for dx, dy in [(HALF_LENGTH, HALF_WIDTH), (HALF_LENGTH, -HALF_WIDTH),
                   (-HALF_LENGTH, -HALF_WIDTH), (-HALF_LENGTH, HALF_WIDTH)]:
        corner_x, corner_y = to_world(x, y, yaw, dx, dy)
        corners.append(world_to_pixel(corner_x, corner_y))
    cv2.polylines(img, [np.array(corners, dtype=np.int32)], True, 129, 1)


def show_map(pose, path):
    """Mapa con las estanterias actualizadas, la zona engordada (caso 1),
    los waypoints (casos 1 y 2) y el robot (casos 2 y 3)."""
    img = np.where(occupied == 1, gray * 127, 127).astype(np.uint8)
    img[(occupied == 1) & (gray > 0.9)] = 0  # estanterias dejadas
    if CASE == 1:
        img[grid > occupied] = 0  # zona engordada
    else:
        draw_rectangle(img, pose.x, pose.y, pose.yaw)
    if CASE < 3:
        for x, y, yaw in path:
            cv2.circle(img, world_to_pixel(x, y), 2, 129, -1)
    WebGUI.showNumpy(img)


def show_path(path):
    """Pinta la ruta en verde. El visor multiplica las columnas por
    0.72 * 1.38 y las filas por 0.545 * 1.9, asi que se deshace."""
    points = []
    for x, y, yaw in path:
        col, row = world_to_pixel(x, y)
        points.append([col / (0.72 * 1.38), row / (0.545 * 1.9)])
    WebGUI.showPath(points)


# PLANIFICADOR (OMPL)
ou.setLogLevel(ou.LOG_WARN)  # que OMPL no llene la consola


def robot_free(x, y, yaw):
    """True si el robot, o la estanteria si la lleva, cabe con el centro
    en (x, y) mirando hacia yaw. Mira un punto cada 2.5 cm."""
    half_length, half_width = HALF_LENGTH, HALF_WIDTH
    if carrying:
        half_length, half_width = SHELF_HALF_LENGTH, SHELF_HALF_WIDTH
    if CASE == 3:
        half_length = half_length + CAR_MARGIN
        half_width = half_width + CAR_MARGIN
    dx, dy = np.meshgrid(np.arange(-half_length, half_length + 0.001, 0.025),
                         np.arange(-half_width, half_width + 0.001, 0.025))
    px, py = to_world(x, y, yaw, dx, dy)
    cols = ((Y_LEFT - py) / RES).astype(int)
    rows = ((X_TOP - px) / RES).astype(int)
    if cols.min() < 0 or rows.min() < 0:
        return False
    if cols.max() >= 415 or rows.max() >= 279:
        return False
    return not occupied[rows, cols].any()


def car_free(x, y, yaw):
    """Caso 3: como robot_free, pero (x, y) es el eje trasero."""
    center_x, center_y, yaw = ahead((x, y, yaw), REAR_AXLE)
    return robot_free(center_x, center_y, yaw)


def is_valid(state):
    """OMPL pregunta aqui si un estado (x, y, yaw) es valido."""
    x, y, yaw = state.getX(), state.getY(), state.getYaw()
    if CASE == 2:
        return robot_free(x, y, yaw)
    if CASE == 3:
        return car_free(x, y, yaw)
    # Caso 1: el robot es un punto en el mapa engordado
    col, row = world_to_pixel(x, y)
    if col < 0 or row < 0 or col >= 415 or row >= 279:
        return False
    return bool(grid[row, col] == 0)  # OMPL no acepta el booleano de numpy


def turn_free(x, y, yaw1, yaw2):
    """Caso 2: True si puede girar en el sitio de yaw1 a yaw2."""
    angle = wrap(yaw2 - yaw1)
    steps = int(abs(angle) / 0.03) + 1
    for i in range(steps + 1):
        if not robot_free(x, y, yaw1 + angle * i / steps):
            return False
    return True


def move_free(x1, y1, x2, y2, yaw):
    """Caso 2: True si puede ir recto de (x1, y1) a (x2, y2)."""
    steps = int(math.hypot(x2 - x1, y2 - y1) / 0.025) + 1
    for i in range(steps + 1):
        x = x1 + (x2 - x1) * i / steps
        y = y1 + (y2 - y1) * i / steps
        if not robot_free(x, y, yaw):
            return False
    return True


def path_free(path):
    """Caso 2: True si puede hacer la ruta como la hace de verdad: en cada
    punto gira en el sitio hacia el siguiente y avanza recto."""
    yaw = path[0][2]
    for i in range(len(path) - 1):
        x1, y1, yaw1 = path[i]
        x2, y2, yaw2 = path[i + 1]
        heading = math.atan2(y2 - y1, x2 - x1)
        if not turn_free(x1, y1, yaw, heading):
            return False
        if not move_free(x1, y1, x2, y2, heading):
            return False
        yaw = heading
    x, y, last_yaw = path[-1]
    return turn_free(x, y, yaw, last_yaw)


def plan(start, goal):
    """Ruta de start a goal, los dos (x, y, yaw), como en la guia.
    Devuelve [(x, y, yaw), ...] o None."""
    # Caso 3: Reeds-Shepp, curvas de radio TURN_RADIUS y rectas,
    # hacia delante y marcha atras
    if CASE == 3:
        space = ob.ReedsSheppStateSpace(TURN_RADIUS)
    else:
        space = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, -6.93)
    bounds.setLow(1, -10.4)
    bounds.setHigh(0, 7.02)
    bounds.setHigh(1, 10.35)
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(ob.StateValidityCheckerFn(is_valid))

    start_state = ob.State(space)
    start_state().setX(start[0])
    start_state().setY(start[1])
    start_state().setYaw(start[2])
    goal_state = ob.State(space)
    goal_state().setX(goal[0])
    goal_state().setY(goal[1])
    goal_state().setYaw(goal[2])

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start_state, goal_state)
    planner = og.RRTstar(si)
    planner.setProblemDefinition(pdef)
    planner.setup()

    # solved tambien es True si solo se acerca al destino
    solved = planner.solve(PLAN_TIME)
    if not solved or not pdef.hasExactSolution():
        return None

    # Caso 3: entre dos estados hay una curva; se rellena con un punto
    # cada 5 cm para poder seguirla y comprobarla
    solution = pdef.getSolutionPath()
    if CASE == 3:
        solution.interpolate(int(solution.length() / 0.05) + 2)
    path = []
    for line in solution.printAsMatrix().splitlines():
        if line.strip() != "":
            x, y, yaw = line.split()
            path.append((float(x), float(y), float(yaw)))

    # OMPL no sabe que el holonomico gira en el sitio y avanza recto
    # (caso 2), y solo mira la curva cada 27 cm (caso 3)
    if CASE == 2 and not path_free(path):
        return None
    if CASE == 3:
        for x, y, yaw in path:
            if not car_free(x, y, yaw):
                return None
    return path


def split_directions(path):
    """Caso 3: parte la curva en tramos todo hacia delante (+1) o todo
    marcha atras (-1). Devuelve [(sentido, [puntos]), ...]."""
    pieces = []
    for i in range(len(path) - 1):
        x1, y1, yaw1 = path[i]
        x2, y2, yaw2 = path[i + 1]
        if (x2 - x1) * math.cos(yaw1) + (y2 - y1) * math.sin(yaw1) >= 0:
            sense = 1
        else:
            sense = -1
        if len(pieces) == 0 or pieces[-1][0] != sense:
            pieces.append((sense, [path[i]]))
        pieces[-1][1].append(path[i + 1])
    return pieces


# CONTROL
def wrap(angle):
    """Deja el angulo entre -pi y pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def sim_time():
    """Tiempo de simulacion en segundos."""
    t = HAL.getSimTime()
    return t.seconds + t.nanoseconds / 1000000000


def turn(pose, target, w_max):
    """Casos 1 y 2: gira en el sitio hasta mirar hacia target."""
    error = wrap(target - pose.yaw)
    if abs(error) < ANGLE_TOL:
        return 0, 0, True
    return 0, max(-w_max, min(w_max, K_W * error)), False


def drive(pose, a, b, v_max, w_max):
    """Casos 1 y 2: avanza por el tramo a-b apuntando a un punto del
    tramo LOOKAHEAD m por delante, para volver a la linea si se desvia."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    ux = (b[0] - a[0]) / length
    uy = (b[1] - a[1]) / length
    advanced = (pose.x - a[0]) * ux + (pose.y - a[1]) * uy
    remaining = length - advanced
    if remaining < DIST_TOL:
        return 0, 0, True
    aim = min(advanced + LOOKAHEAD, length)
    aim_x = a[0] + ux * aim
    aim_y = a[1] + uy * aim
    error = wrap(math.atan2(aim_y - pose.y, aim_x - pose.x) - pose.yaw)
    w = max(-w_max, min(w_max, K_W * error))
    v = min(v_max, max(V_MIN, remaining))
    if abs(error) > 0.5:  # muy desorientado: primero girar
        v = 0
    return v, w, False


def follow(pose, points, sense, index, v_max):
    """Caso 3: persecucion pura por un tramo, hacia delante (sense = 1) o
    marcha atras (-1). Devuelve v, w, el punto mas cercano y si acabo."""
    rx, ry, ryaw = ahead((pose.x, pose.y, pose.yaw), -REAR_AXLE)
    here = (rx, ry)
    # Punto mas cercano, buscando solo un poco por delante del anterior
    for i in range(index, min(index + 20, len(points))):
        if math.dist(points[i][:2], here) < math.dist(points[index][:2], here):
            index = i
    ex, ey, eyaw = points[-1]
    left = sense * ((ex - rx) * math.cos(eyaw) + (ey - ry) * math.sin(eyaw))
    if index >= len(points) - 2 and left < DIST_TOL:
        return 0, 0, index, True
    # Apunta al primer punto a LOOKAHEAD_CAR m (o mas alla del final)
    tx, ty, tyaw = ahead(points[-1], sense * LOOKAHEAD_CAR)
    for x, y, yaw in points[index:]:
        if math.dist((x, y), here) >= LOOKAHEAD_CAR:
            tx, ty = x, y
            break
    # Arco del eje trasero a ese punto: curvatura = 2 * lateral / d^2
    dx = tx - rx
    dy = ty - ry
    side = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
    curvature = 2 * side / (dx * dx + dy * dy)
    # Frena al acercarse al final del tramo (un punto cada ~5 cm)
    v = sense * min(v_max, max(V_MIN, (len(points) - 1 - index) * 0.05))
    return v, v * curvature, index, False


# MAQUINA DE ESTADOS
#   PLAN     -> ruta hasta la estanteria o hasta su destino
#   TURN     -> casos 1 y 2: girar hacia el siguiente punto
#   DRIVE    -> casos 1 y 2: avanzar hasta el siguiente punto
#   FOLLOW   -> caso 3: seguir un tramo de la curva
#   STOP     -> esperar a que se pare: siguiente tramo o levantar/dejar
#   PLATFORM -> esperar a la plataforma y actualizar el mapa
#   EXIT     -> salir marcha atras de debajo de la estanteria
#   DONE     -> fin
state = "PLAN"
mission = 0
carrying = False
path = []
grid = occupied
last_xy = (0, 0)

while True:
    pose = HAL.getPose3d()
    v = 0
    w = 0
    if carrying:
        v_max, w_max = V_LOADED, W_LOADED
    else:
        v_max, w_max = V_MAX, W_MAX

    if state == "PLAN":
        number, destination = MISSIONS[mission]
        if carrying:
            goal = destination
            radius = SHELF_RADIUS
        else:
            goal = shelves[number - 1]
            radius = ROBOT_RADIUS
            print("Traslado", mission + 1, ": a por la estanteria", number)
        if CASE == 1:
            grid = inflate(radius + MARGIN)
        # Punto de espera detras del destino: de ahi entra en linea recta
        distance = APPROACH_DIST
        if CASE == 3:
            distance = APPROACH_CAR
        start = (pose.x, pose.y, pose.yaw)
        approach = ahead(goal, -distance)
        if CASE == 3:  # el Ackermann se planifica con el eje trasero
            start = ahead(start, -REAR_AXLE)
            approach = ahead(approach, -REAR_AXLE)
        for i in range(PLAN_TRIES):
            path = plan(start, approach)
            if path is not None:
                break
        if path is None:
            print("No se ha encontrado ruta")
            path = []
            state = "DONE"
        elif CASE == 3:
            # Tramo recto final, tambien con un punto cada 5 cm
            for i in range(1, round(distance / 0.05) + 1):
                path.append(ahead(approach, i * 0.05))
            pieces = split_directions(path)
            piece = 0
            index = 0
            piece_start = sim_time()
            state = "FOLLOW"
        else:
            path.append(goal)
            seg = 0
            state = "TURN"
        show_path(path)

    elif state == "TURN":
        heading = math.atan2(path[seg + 1][1] - path[seg][1],
                             path[seg + 1][0] - path[seg][0])
        v, w, done = turn(pose, heading, w_max)
        if done:
            state = "DRIVE"

    elif state == "DRIVE":
        if seg == len(path) - 2:
            v_max = V_SLOW
        v, w, done = drive(pose, path[seg], path[seg + 1], v_max, w_max)
        if done:
            seg = seg + 1
            state = "TURN"
            if seg == len(path) - 1:
                state = "STOP"

    elif state == "FOLLOW":
        # Arranca casi parado y acelera poco a poco: asi las ruedas, que
        # tardan unos 2 s en girar, giran antes de que avance mucho
        sense, points = pieces[piece]
        ramp = min(1, (sim_time() - piece_start) / RAMP_TIME)
        v_top = V_START + (V_CAR - V_START) * ramp * ramp
        v, w, index, done = follow(pose, points, sense, index, v_top)
        if done:
            piece = piece + 1
            index = 0
            state = "STOP"

    elif state == "STOP":
        if math.dist((pose.x, pose.y), last_xy) < 0.001:  # ya esta parado
            if CASE == 3 and piece < len(pieces):
                piece_start = sim_time()
                state = "FOLLOW"
            else:
                if carrying:
                    HAL.putdown()
                else:
                    HAL.lift()
                wait = sim_time()
                state = "PLATFORM"

    elif state == "PLATFORM":
        if sim_time() - wait >= WAIT_TIME:
            if carrying:  # la deja donde esta el robot
                shelves[number - 1] = [pose.x, pose.y, pose.yaw]
                add_shelf(pose.x, pose.y, pose.yaw)
                exit_start = (pose.x, pose.y)
                exit_yaw = pose.yaw
                print("Estanteria", number, "dejada en",
                      (round(pose.x, 2), round(pose.y, 2)))
                state = "EXIT"
            else:  # la levanta: su hueco queda libre
                remove_shelf(goal[0], goal[1])
                state = "PLAN"
            carrying = not carrying

    elif state == "EXIT":
        v = -V_SLOW
        w = K_W * wrap(exit_yaw - pose.yaw)
        if math.dist((pose.x, pose.y), exit_start) > APPROACH_DIST:
            v = 0
            w = 0
            mission = mission + 1
            state = "PLAN"
            if mission == len(MISSIONS):
                print("Fin: estanterias trasladadas")
                state = "DONE"

    last_xy = (pose.x, pose.y)
    HAL.setV(v)
    HAL.setW(w)
    show_map(pose, path)
    Frequency.tick(20)
