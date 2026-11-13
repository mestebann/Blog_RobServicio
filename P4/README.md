# Blog_RobServicio

El objetivo es que un robot de servicio mueva de forma autónoma las 6 estanterías del almacén Warehouse 1 de un sitio a otro. Las rutas se planifican con la biblioteca OMPL y el robot las sigue con un pilotaje reactivo basado solo en su posición. Hay tres casos: en el 1, el robot holonómico se planifica como un punto con los obstáculos engordados; en el 2, con su forma real y sin engordar los obstáculos, teniendo en cuenta que a la vuelta lleva la estantería encima; y en el 3 se usa el robot Ackermann, que es rectangular y no puede girar en el sitio. El código es una máquina de estados y la práctica la he dividido en cinco pasos: el mapa y los traslados, la planificación con OMPL y un paso por cada caso.

`([PLAN → TURN ⇄ DRIVE → STOP → PLATFORM] x 2 → EXIT) x 6 → DONE` en los casos 1 y 2

`([PLAN → FOLLOW ⇄ STOP → PLATFORM] x 2 → EXIT) x 6 → DONE` en el caso 3

Lo que va entre corchetes se hace dos veces en cada traslado: una para ir a por la estantería y levantarla, y otra para llevarla a su destino y dejarla.

## PASO 1: MAPA Y TRASLADOS

El mapa del almacén es una imagen de 415 x 279 píxeles de 5 cm, en la que los píxeles oscuros son obstáculos. La función `world_to_pixel` pasa una posición del mundo a su píxel: en la imagen, el eje x del mundo va hacia arriba y el y hacia la izquierda.

Las 6 estanterías empiezan en fila, en x = 3,84 m. Las 1, 2 y 3 se llevan a una zona libre en x = −1,5 m, y después las 4, 5 y 6 se colocan en los huecos que han dejado las tres primeras.

Cuando el robot llega a la estantería o a su destino, `STOP` espera a que esté parado del todo y `PLATFORM` sube o baja la plataforma con `HAL.lift()` o `HAL.putdown()`, esperando 2 s. Como pide el enunciado, ahí mismo se actualiza el mapa, sin quitar nada a mano: al levantar la estantería se borra su sitio y al dejarla se dibujan sus 4 patas donde está el robot. Después de dejarla, `EXIT` saca al robot marcha atrás 2,04 m para salir de debajo.

Las medidas de los robots son las de su modelo URDF/Xacro: la plataforma del holonómico mide 0,55 x 0,55 m y el Ackermann 1,15 x 0,53 m. La estantería mide 2,06 x 0,88 m.

### Problemas y soluciones

- Las medidas del enunciado no eran las de los robots: el enunciado dice que el holonómico mide 0,3 x 0,3 m y el Ackermann 0,72 x 0,32 m, pero en Gazebo son más grandes. Con esas medidas el planificador daría por buenos huecos por los que el robot real no cabe. Se usan las del modelo URDF/Xacro y con ellas funcionan los tres casos.

- Las esperas no duraban lo previsto: Gazebo va más lento que el tiempo real (entre 0,3 y 0,6 veces), así que los tiempos contados en vueltas del bucle no coincidían con los de la simulación. Ahora se usa el tiempo simulado de `HAL.getSimTime()`. El enunciado dice que sus campos son sec y nanosec, pero el programa daba error porque se llaman seconds y nanoseconds.

## PASO 2: PLANIFICACIÓN CON OMPL

En el estado `PLAN` el robot pide a OMPL una ruta hasta la estantería, si va sin carga, o hasta su destino, si la lleva encima. OMPL se usa como viene en la guía de la práctica: un espacio `SE2StateSpace` con los límites del mapa, un `SpaceInformation` con la función que dice si un estado es válido, los estados de inicio y fin con `setX`, `setY` y `setYaw`, un `ProblemDefinition` y el planificador RRT*. La ruta se saca con `getSolutionPath()` y `printAsMatrix()`, que da una línea (x, y, yaw) por cada punto.

Cada fichero tiene su propia función de validez, `is_valid`, y en el caso 3 cambia también el espacio de estados. OMPL busca durante 3 s y, si no encuentra ruta, se le pide otra vez, hasta 5 veces. Para que no llene la terminal con sus mensajes, se dejan solo sus avisos con `ou.setLogLevel(ou.LOG_WARN)`.

OMPL no planifica hasta el destino, sino hasta un punto de espera que está detrás de él, a 2,04 m (3 m en el caso 3). Desde ahí el robot entra en línea recta, alineado con la estantería o con su hueco, y al dejarla sale marcha atrás por la misma recta.

### Problemas y soluciones

- OMPL no aceptaba la orientación π: las estanterías 1, 2 y 3 se dejan mirando hacia −x, con yaw = π, que se sale de los límites del espacio SE2. Se usa −π, que es la misma orientación.

- OMPL daba por buenas rutas que no llegaban al destino: `solve()` también devuelve True cuando solo encuentra una ruta aproximada, que se queda cerca. Ahora solo se acepta la ruta si además `pdef.hasExactSolution()` es True.

- Las rutas daban rodeos: con RRTConnect salían rutas con muchas vueltas que pasaban a 5 cm de las paredes. Con RRT* salen más directas en el mismo tiempo.

## PASO 3: CASO 1, ROBOT PUNTUAL CON OBSTÁCULOS ENGORDADOS

En el caso 1 el robot se trata como un punto y, para que quepa, se engordan los obstáculos del mapa con un círculo usando `cv2.dilate`. Sin carga se engordan 0,49 m: la mitad de la diagonal del robot (0,39 m) más 0,1 m de margen. Con la estantería encima se engordan 1,2 m, algo más que la mitad de la diagonal de la estantería (1,12 m). Así la geometría es distinta a la ida y a la vuelta. El mapa engordado se calcula de nuevo en cada planificación, así que siempre tiene las estanterías donde están.

La función de validez solo mira si el píxel del punto está libre en el mapa engordado; la orientación no cuenta.

Para seguir la ruta, en cada punto el robot gira en el sitio hacia el siguiente `(TURN)` y luego avanza recto hasta él `(DRIVE)`. Mientras avanza, apunta a un punto del tramo que está 0,4 m por delante, para volver a la línea si se desvía. Sin carga va como mucho a 0,5 m/s, con la estantería a 0,3 m/s y en el último tramo a 0,2 m/s.

### Problemas y soluciones

- OMPL no encontraba ruta hasta la estantería: al engordar las patas 0,49 m, el hueco de 0,8 m que hay entre ellas se cierra y el centro de la estantería queda dentro de la zona engordada. Por eso OMPL planifica solo hasta el punto de espera, fuera de la estantería, y el último tramo lo hace el robot en línea recta. Se usa también en los otros dos casos, porque así el robot siempre entra alineado entre las patas.

## PASO 4: CASO 2, ROBOT CUADRADO SIN ENGORDAR LOS OBSTÁCULOS

En el caso 2 el mapa no se engorda y se comprueba la forma real del robot: el cuadrado de 0,55 x 0,55 m sin carga y el rectángulo de 2,06 x 0,88 m de la estantería cuando la lleva encima. La función `robot_free` llena esa forma de puntos cada 2,5 cm, los gira con la orientación del robot y mira si alguno cae en un obstáculo. Aquí la orientación sí importa: con la estantería encima, el robot puede chocar al girar aunque su centro esté libre.

El robot sigue la ruta igual que en el caso 1, con `TURN` y `DRIVE`.

### Problemas y soluciones

- OMPL daba rutas con las que el robot podía chocar al girar: entre dos estados, OMPL comprueba el camino cambiando x, y y yaw a la vez, pero el robot primero gira en el sitio y luego va recto. Con la estantería encima, que al girar barre un círculo de 1,12 m de radio, una ruta válida para OMPL podía chocar al hacerla. Se resolvió mediante la función `path_free` que comprueba la ruta tal y como la hace el robot y, si choca, se pide otra.

## PASO 5: CASO 3, ROBOT ACKERMANN

El Ackermann no puede girar en el sitio, así que la ruta tiene que respetar su radio de giro. Antes de programarlo se probó en Gazebo con un programa aparte: su radio de giro mínimo es de 0,82 m en el eje trasero, las ruedas tardan entre 1 y 2 s en girar lo que se les pide y, con v = 0, no se mueve aunque se le mande girar.

Para planificar se usa `ReedsSheppStateSpace` con un radio de 1,3 m, así que las rutas son curvas de ese radio y rectas, hacia delante y marcha atrás. Las curvas de Reeds-Shepp son las que sigue el centro del eje trasero, que está 0,244 m por detrás de la posición que da `getPose3d()`, así que se planifica con el eje trasero y no con el centro del coche. La función de validez, `car_free`, comprueba el rectángulo del coche (1,15 x 0,53 m), o el de la estantería si la lleva, con 0,15 m de margen por cada lado; el mapa sigue sin engordar.

La ruta se divide en tramos hacia delante y tramos marcha atrás, y el robot se para `(STOP)` entre uno y otro. En `FOLLOW` sigue cada tramo con persecución pura: apunta a un punto de la ruta que está a 0,6 m y calcula el arco que lleva su eje trasero hasta él (curvatura = 2 · distancia lateral / distancia²). La velocidad de giro es la velocidad por la curvatura. Va a 0,2 m/s y frena al acercarse al final de cada tramo.

### Problemas y soluciones

- El coche se salía de la ruta en las curvas: con radio 1 m y 0,3 m/s se separaba hasta 0,5 m, porque las ruedas tardan en girar y, cuando por fin giran, el coche ya ha avanzado. Se sube el radio a 1,3 m, se baja la velocidad a 0,2 m/s y cada tramo empieza casi parado (0,01 m/s) y acelera poco a poco durante 3 s, para que las ruedas giren antes de que avance mucho.

- OMPL no veía algunas patas: en las curvas, OMPL solo comprueba más o menos cada 27 cm, y una pata es más fina. Ahora se rellena la ruta con un punto cada 5 cm y se comprueban todos; si alguno choca, se pide otra ruta.

- El coche rozaba las patas: las rutas que pasaban justas, sumadas al error al seguirlas, hacían que tocara alguna pata. Se añade al rectángulo del coche 0,15 m de margen por cada lado.

- El coche entraba torcido bajo la estantería: la recta final solo tenía dos puntos, el de espera y el destino, y la persecución pura apuntaba directamente al final. Además, con 2,04 m de recta el morro llegaba a las patas a los 0,47 m, cuando todavía estaba corrigiendo. Se rellena la recta con un punto cada 5 cm y se alarga a 3 m.

## VÍDEOS

(Los vídeos de los tres casos están acelerados en x2)

### CASO 1

[Caso1.webm](https://github.com/user-attachments/assets/a9c86f32-1897-465b-aabd-8e44dff0a74a)

### CASO 2

[Caso2.webm](https://github.com/user-attachments/assets/11303384-df07-439b-a84a-df1ab0dffce6)

### CASO 3

[Caso3.webm](https://github.com/user-attachments/assets/45268301-e942-43a0-8013-290a3697db77)
