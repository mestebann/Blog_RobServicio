# Blog_RobServicio

El objetivo es que un coche autónomo aparque en línea en un hueco libre de la fila de coches que tiene a su derecha. El coche arranca en la calle, paralelo a la fila. El mundo final es Autopark Lidar Line,en el que solo se pueden usar el LiDAR 3D del techo y la odometría. Primero entrené la maniobra en Autopark_line, que sí tiene los láseres delantero, trasero y de la derecha, y después la pasé al LiDAR. El código es una máquina de estados y la práctica la he dividido en cinco pasos: leer el entorno con el LiDAR, alinearse con la calle, encontrar el hueco, hacer la maniobra y ajustar la posición final.

`ALINEAR → BUSCAR_HUECO → COLOCARSE → ATRAS_DERECHA → ATRAS_IZQUIERDA ⇄ ADELANTE_DERECHA
→ COMPROBAR → [AJUSTE_ATRAS → AJUSTE_DERECHA → AJUSTE_IZQUIERDA] (hasta 3 veces) → CENTRAR → APARCADO`

## PASO 1: LECTURA DEL ENTORNO CON EL LIDAR

El LiDAR devuelve unos 64 800 puntos (x, y, z) por lectura, medidos desde el propio sensor. Está en el techo, a 1,62 m del suelo y 1,25 m por detrás del centro del coche, y va girado 90º: su x apunta a la derecha del coche y su y hacia delante. La función `puntos_coche` recorre uno de cada dos puntos y se queda con los que están entre 0,3 y 1,5 m del suelo: por debajo están el asfalto y la acera, y por encima no hay coches. Cada punto se pasa a los ejes del coche, (adelante, derecha), con el origen en su centro.

A partir de esos puntos trabajan otras dos funciones. `puntos_calle` los gira el ángulo que forma el coche con la calle, para que el rectángulo del hueco no se tuerza aunque el coche no vaya recto. `distancias_pasillo` sustituye a los láseres delantero y trasero: mide la distancia de cada parachoques a lo más cercano que hay en el pasillo que ocupa el coche, con una sola lectura del LiDAR.

### Problemas y soluciones

- La terminal se llenaba con los datos del LiDAR: `getLidarData()` hace un print de los datos en cada lectura. Sustituimos el print del módulo HAL por una función que no hace nada; los print de nuestro código siguen funcionando.

```python
def no_imprimir(*args, **kwargs):
    """Sustituye al print del HAL, que imprime los datos del LIDAR en cada
    lectura y llena la terminal"""
    pass

HAL.print = no_imprimir
```

- El LiDAR veía nuestro propio coche: Desde el techo, el LiDAR ve partes del propio coche, como el capó y los retrovisores. Cuando el coche salía torcido, esos puntos quedaban a la derecha al girarlos a los ejes de la calle, y el coche medía "Coches de la derecha a 1.20 m": se tomaba a sí mismo por la fila de coches aparcados. Ahora quitamos, antes de girarlos, los puntos que caen dentro del rectángulo de nuestro coche, con 10 cm de margen delante y detrás y 15 cm a los lados por los retrovisores.

```python
# El LIDAR tambien ve partes de nuestro propio coche, esas se quitan
en_el_coche = False
if -TRASERA - 0.1 < adelante < DELANTERA + 0.1:
    if abs(derecha) < MEDIO_ANCHO + 0.15:
        en_el_coche = True
if not en_el_coche:
    puntos.append((adelante, derecha))
```

## PASO 2: ALINEAMIENTO CON LA CALLE

La calle va a lo largo de un eje del mundo, así que su dirección (`yaw_calle`) es el múltiplo de 90º más cercano a la orientación inicial del coche. Con ella, la odometría se pasa a posición a lo largo de la calle, posición lateral y ángulo respecto a la calle; en este mundo la calle va según el eje x, así que son la x, la y y el yaw del coche. Al arrancar, el coche mide con el LiDAR a qué distancia está el lateral de los coches aparcados: la menor distancia hacia la derecha entre los puntos que tiene a menos de 15 m por delante o por detrás (4,43 m desde la salida por defecto). Con ella calcula la posición lateral a la que tiene que ir para quedar a 1,6 m de ellos.

En el estado ALINEAR, `seguir_carril` avanza a 0,8 m/s con dos controles proporcionales en cascada: el error lateral decide el ángulo con el que acercarse (0,5 rad por metro de error, como mucho 20º) y el error de orientación decide la velocidad de giro. De lejos se acerca con 20º y, según llega, el ángulo baja y el coche queda paralelo a la calle. Cuando el error lateral baja de 10 cm, pasa a buscar el hueco. La posición lateral objetivo se mide una sola vez con el LiDAR y después se mantiene con la odometría, así que el coche no se mete en los huecos al pasar junto a ellos.

### Problemas y soluciones

- El coche se quedaba demasiado lejos de los coches aparcados: La primera versión se colocaba a 3,1 m de ellos, demasiado lejos para luego aparcar, y la bajamos a 2 m. Al hacer la maniobra la bajamos a 1,6 m, porque cuanto más cerca de la fila empieza, más adentro acaba el coche. A 1,6 m quedan 0,68 m entre nuestro lateral y los retrovisores de los coches aparcados.

## PASO 3: BÚSQUEDA DEL HUECO

En el estado BUSCAR_HUECO el coche sigue su línea y mira si tiene a la derecha un rectángulo libre un poco más grande que él. Si ningún punto del LiDAR cae dentro, hay hueco. Solo cuenta lo que hay dentro del rectángulo, así que da igual que haya coche delante, detrás, en los dos lados o en ninguno. El bordillo no cuenta porque queda por debajo de 0,3 m, y una pared o una farola al fondo del hueco quedarían más allá de 3,6 m, fuera de la franja.

Al encontrarlo, `inicio_del_hueco` guarda la posición a lo largo de la calle del morro del coche de detrás, que es el punto de la fila más cercano por detrás de nuestro coche. Si no hay coche detrás, toma el principio del rectángulo libre.

### Problemas y soluciones

- La maniobra empezaba a distancias distintas del coche de detrás: Al principio contábamos el sitio para la maniobra desde el punto donde se detectaba el hueco, pero ese punto depende de cómo llegue el coche al hueco, y en algunas pruebas la maniobra empezaba demasiado cerca del coche de detrás. Ahora se cuenta desde su morro, que no se mueve.

## PASO 4: MANIOBRA DE APARCAMIENTO

En COLOCARSE el coche sigue avanzando hasta que su centro ha pasado 11,5 m del morro del coche de detrás. En ese momento, con el coche alineado a 1,6 m de la fila, se fijan dos referencias laterales para toda la maniobra: `lateral_fondo`, lo más adentro que pueden llegar las esquinas del coche, y `lateral_aparcado`, donde debe acabar el centro del coche.

ATRAS_DERECHA va marcha atrás con el volante a tope a la derecha, para que la trasera entre en el hueco, hasta que el coche forma 30º con la calle. ATRAS_IZQUIERDA sigue marcha atrás con el volante a la izquierda para enderezarlo, y ADELANTE_DERECHA avanza con el volante a la derecha cuando ya no cabe más atrás.

ATRAS_DERECHA y ATRAS_IZQUIERDA paran si detras_bloqueado(): hay algo a menos de 0,6 m del parachoques trasero o la esquina trasera derecha llega a lateral_fondo. ADELANTE_DERECHA vuelve a ATRAS_IZQUIERDA si tiene el coche de delante a menos de 0,6 m, así que el coche alterna adelante y atrás hasta quedar recto (menos de 1º).

### Problemas y soluciones

- El coche se subía a la acera: En una primera versión la curva marcha atrás llegaba a 43º y la trasera acababa en la acera. La dejamos en 30º y añadimos un límite de fondo. Como el bordillo queda por debajo del filtro de altura del LiDAR, la posición de la esquina trasera derecha se calcula con la odometría y las medidas del coche.

```python
def esquina_trasera():
    """Posicion lateral de la esquina trasera derecha"""
    a = angulo_calle()
    return posicion_lateral() - TRASERA * math.sin(a) - MEDIO_ANCHO * math.cos(a)
```

## PASO 5: AJUSTES Y CENTRADO

Después de la maniobra el coche queda recto, pero normalmente algo por fuera de lateral_aparcado. En COMPROBAR, si el centro está más de 10 cm por fuera, el coche hace una "ese" hacia delante para meterse más, como mucho 3 veces: AJUSTE_ATRAS retrocede recto hasta el fondo del hueco para tener sitio delante, AJUSTE_DERECHA avanza con el volante a la derecha hasta girar 6º y AJUSTE_IZQUIERDA avanza con el volante a la izquierda hasta volver a estar recto. Al final, CENTRAR lo mueve recto hasta tener el mismo espacio delante que detrás y APARCADO lo para.

### Problemas y soluciones

- El coche se quedaba un poco fuera del hueco: Probamos a añadir después de ATRAS_DERECHA un tramo recto marcha atrás (estado METER) para meter más la trasera, pero no funcionó: con un tramo corto el coche hacía las mismas eses, y con uno largo chocaba con el coche de delante. Ir recto con el coche a 30º mete la trasera, pero también lo acerca al coche de detrás, y luego le queda menos sitio para enderezarse. El problema de fondo es geométrico, por eso la solución fueron las eses del final.

## VÍDEO

[grabacion-de-pantalla-desde-2026-10-23-11-10-52_X7z9pj14.webm](https://github.com/user-attachments/assets/6bdcb560-0943-4831-9eb8-77f522ad88c1)


## IMAGEN DEL COCHE APARCADO

<img width="917" height="712" alt="Captura desde 2026-10-23 11-23-23" src="https://github.com/user-attachments/assets/6bd44f69-9039-463c-a405-c998d8995e0f" />
















