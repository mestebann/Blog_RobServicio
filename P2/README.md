# Blog_RobServicio

El objetivo es que un dron encuentre a las personas perdidas en el mar reconociendo sus caras con la cámara ventral y guarde sus posiciones para un rescate posterior. Partimos de las coordenadas GPS de la barca de salvamento y de las de la zona donde se cree que están los supervivientes. La práctica la he dividido en cinco pasos: llegar a la zona, barrerla en espiral, detectar las caras, guardar cada persona una sola vez y simular la batería del dron.

## PASO 1: COORDENADAS Y VUELO A LA ZONA

La API del dron no entiende coordenadas GPS, así que pasamos las del enunciado a UTM, que están en metros. Restando la posición de la barca, que es el punto de despegue, obtenemos dónde están los supervivientes respecto al dron. El dron despega a 8 m de altura y vuela hasta la zona con un control de posición bloqueante (`go_to`), que manda la referencia hasta estar a menos de 0,5 m del objetivo. Durante todo el vuelo mantenemos la orientación inicial, para que la imagen de la cámara ventral no gire respecto al mapa.

```python
BARCA_UTM = (430492.16, 4459162.06)
SUPERVIVIENTES_UTM = (430532.50, 4459131.78)
```
### Problemas y soluciones

- El dron no iba a donde estaban las personas: En la primera versión usamos los ejes que indica el enunciado para el control de posición (x = Norte, y = Oeste) y el dron acababa lejos de la gente. Parecía un error al pasar los grados, minutos y segundos a metros, pero esa conversión estaba bien: el fallo estaba en los ejes. En el mundo del ejercicio, la barca está en el origen y las personas entre x = 25 y 42 m e y = -31 y -40 m, así que esta versión del simulador usa los ejes de Gazebo: x = Este, y = Norte. La posición inicial del dron, (0, 0, 1.4), también lo confirmaba: está sobre la cubierta de la barca y la altura se mide desde el agua. Con los ejes corregidos, el objetivo pasa a (40.3, -30.3) y el dron llega a la zona.

```python
def utm_to_local(este, norte):
    # Marco del mundo en Gazebo: x = Este, y = Norte, con origen en la barca
    return este - BARCA_UTM[0], norte - BARCA_UTM[1]
```

## PASO 2: BÚSQUEDA EN ESPIRAL

Para barrer la zona, el dron recorre una espiral cuadrada hacia fuera, con tramos de 1, 1, 2, 2, 3, 3... pasos girando al este, norte, oeste y sur. La separación entre vueltas sale de lo que ve la cámara ventral: a 8 m de altura cubre unos 9 x 6,8 m de agua, y dejamos la separación en el 80 % del lado corto (5,4 m) para que las pasadas se solapen un 20 %. La espiral da siempre vueltas completas, para que la zona barrida quede simétrica.

```python
HUELLA = 2 * ALTURA * np.tan(CAMPO_VISION / 2) * 240 / 320   # lado corto de lo que ve la cámara (m)
SEPARACION = 0.8 * HUELLA      # separación entre vueltas (20 % de solape)
```

### Problemas y soluciones

- A 3 m de altura la cámara veía muy poco: Al principio el dron volaba a 3 m y la cámara solo cubría unos 3,4 x 2,5 m de agua, así que habría hecho falta una espiral muy larga. Subimos a 8 m después de comprobar que el detector sigue encontrando las caras a esa altura.

- La espiral no empezaba en el sitio correcto: Centrada en el punto GPS, la espiral empezaba arriba a la izquierda de las personas, porque ese punto está en el borde noroeste del grupo, y su extremo superior izquierdo barría agua sin nadie. Desplazamos el centro 7 m al este y 5 m al sur. Al quedar centrada, basta con un radio de 10 m en lugar de 15: la espiral pasa de 13 a 9 puntos y de unos 230 a 110 m, la mitad de tiempo.

```python
RADIO_BUSQUEDA = 10.0          # media anchura de la zona a barrer (m)
DESPLAZAMIENTO_BUSQUEDA = (-7.0, -5.0)  # centro de la espiral respecto al punto GPS
```

## PASO 3: DETECCIÓN DE CARAS

Las caras se detectan con el clasificador en cascada de Haar de OpenCV (haarcascade_frontalface_default.xml), con la misma estructura que el tutorial de OpenCV: cargamos el clasificador con load, pasamos cada imagen a gris con cvtColor, buscamos las caras con detectMultiScale y dibujamos una elipse sobre cada una antes de mostrar la imagen. La función detect_and_display hace lo mismo que detectAndDisplay del tutorial y además devuelve el centro de cada cara en píxeles. Solo se buscan caras durante la espiral, no en el trayecto desde la barca.

```python
clasificador_caras = cv2.CascadeClassifier()

if not clasificador_caras.load("/resources/exercises/rescue_people/haarcascade_frontalface_default.xml"):
    print("--(!)Error loading face cascade")
    exit(0)
```

### Problemas y soluciones

- Caras giradas: Las personas flotan en cualquier orientación y el Haar solo encuentra caras casi derechas (unos ±15º). Girar el dron costaba tiempo de vuelo y complicaba pasar de píxeles al mapa, así que giramos la imagen: la probamos girada de 30 en 30º (12 giros), pasamos el Haar en cada una y deshacemos el giro para tener el centro de cada cara en la imagen original.

## PASO 4: GUARDAR CADA PERSONA UNA SOLA VEZ

Cada cara detectada se pasa a coordenadas del mapa a partir de la posición del dron, su altura y el campo de visión de la cámara. Como la cámara ventral mira hacia abajo, arriba en la imagen es el morro del dron y la izquierda es su izquierda. Si ya hay una cara guardada a menos de 2 m, es la misma persona y solo afinamos su posición con la media de todas las veces que se ha visto; si no, es una persona nueva. Cuando hay 6 personas confirmadas, el dron deja la espiral, vuelve a la barca y aterriza.

### Problemas y soluciones

- La misma cara se veía muchas veces: Cada cara aparece en varias pasadas de la espiral, y a veces en dos giros de la misma imagen. Comparamos posiciones en el mapa y no imágenes: dos detecciones a menos de 2 m son la misma persona.

```python
for p in personas:
    if np.hypot(p[0] - x, p[1] - y) < DIST_MISMA_PERSONA:
        p[0] = (p[0] * p[2] + x) / (p[2] + 1)
        p[1] = (p[1] * p[2] + y) / (p[2] + 1)
        p[2] += 1
```

- Un falso positivo suelto podía contar como persona: Si el Haar se equivocaba en una sola imagen, ese punto contaría como una persona más y el dron podría volver a la barca sin haber encontrado a todos. Por eso una cara solo cuenta cuando se ha visto 3 veces.

```python
VECES_CONFIRMAR = 3      # veces que hay que ver una cara para darla por buena
```

## PASO 5: BATERÍA SIMULADA

Simulamos la batería con un contador que empieza en el 100 % y baja un 0,5 % por cada metro recorrido. Si durante la búsqueda baja al 40 %, el dron guarda dónde está, vuelve a la barca y aterriza para "recargar": en tierra la batería sube un 20 % por segundo hasta el 100 %. Después despega, vuelve al punto guardado y sigue el mismo tramo de la espiral donde lo dejó. Las personas encontradas siguen guardadas, así que no se cuentan otra vez.

### Problemas y soluciones

- La batería no podía bajar con el tiempo: Mientras se detectan caras el simulador va más lento, así que con un contador por tiempo la batería dependería de lo rápido que sea el ordenador. La hacemos bajar con los metros recorridos.

- Cuándo volver a recargar: El punto de la espiral más lejano está a 60 m de la barca, lo que gasta un 30 % de batería. Con el umbral en el 40 % el dron siempre llega a la barca con margen.

- Ver la batería en todo momento: Queríamos ver el porcentaje constantemente en la terminal sin llenarla de líneas. La consola del ejercicio es un xterm, así que la batería se escribe siempre en la última línea y se reescribe en el sitio: `\r` vuelve al principio de la línea y `\033[K` la borra. Los demás mensajes también empiezan por `BORRAR_LINEA`, así que ocupan la línea de la batería y esta vuelve a pintarse debajo.

```python
BORRAR_LINEA = "\r\033[K"  # vuelve al principio de la línea y la borra
print(f"{BORRAR_LINEA}Batería: {bateria:.0f} %", end="", flush=True)
```

## VÍDEO

[Vídeo de la ejecución](https://drive.google.com/file/d/1VMD0OPV1KGCFolIfKEdEBqSJsddgoTTW/view?usp=sharing)
