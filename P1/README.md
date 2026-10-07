# Blog_RobServicio

El objetivo es que un robot aspirador limpie la mayor superficie posible de una casa. El robot está autolocalizado y tenemos el plano de la casa en una imagen. La práctica se divide en tres pasos: registrar el mapa, construir una rejilla de ocupación y planificar la ruta con BSA para después recorrerla.

## PASO 1: REGISTRO DE LOS PUNTOS

En este primer paso se relacionan las coordenadas de Gazebo (metros) con los píxeles del mapa. Llevamos el robot a 10 posiciones de la casa, apuntamos su pose `(x, y)` y estimamos a ojo su píxel (columna, fila). Con esos puntos ajustamos por mínimos cuadrados una transformación afín. El resultado es una escala de unos 100 px/m (1 px = 1 cm), con el mapa girado 180º respecto a Gazebo.

### Problemas y soluciones

- Error al estimar los píxeles a ojo: Algunos puntos tenían hasta 15 px de error. Al usar 10 puntos repartidos por toda la casa, los mínimos cuadrados compensan unos errores con otros (error medio de 8 px).
  
- Desplazamiento de unos 20 cm: había un error sistemático en las filas estimadas. Lo corregimos buscando el desplazamiento que mejor pega el láser a las paredes del mapa (+1 px en columnas y +21 en filas). El error láser-mapa bajó de 6,2 a 2,2 px.

```python
coef_col[2] = coef_col[2] + 1    
coef_fila[2] = coef_fila[2] + 21
```

- El mapa parecía descuadrado: El robot aparece unos 18 cm más arriba que en nuestro mapa. El problema está en su dibujo, que coloca la esquina del icono en la posición del robot y no su centro. El láser confirma nuestra calibración, así que no la cambiamos.

- El láser no está en el centro del robot: Al buscar la corrección proyectábamos el láser desde el centro del robot, pero el láser va 13,9 cm por delante de él. Teniéndolo en cuenta, la corrección en columnas pasa de +1 a +15 px, es decir, el robot estaba unos 15 cm desplazado de donde creía el código. Esta era la causa principal de que se enganchara en algunos picos. Además, pasamos a usar el mapa nuevo `(/workspace/code/mapgrannyannie.png)`, con el que la corrección en filas queda en +20.

```python
coef_col[2] = coef_col[2] + 15
coef_fila[2] = coef_fila[2] + 20
```

## PASO 2: OCUPACIÓN DE LA REJILLA

En este paso se divide la casa en celdas algo más pequeñas que el robot (30 cm) y decidir cuáles están ocupadas. Para cada celda pasamos sus 4 esquinas a píxeles, barremos los píxeles del rectángulo que forman y, si hay suficientes negros (20), la celda se marca como ocupada.

### Problemas y soluciones

- El robot rozaba las paredes y se atascaba: El robot mide unos 35-40 cm, así que en una celda libre pegada a una pared su cuerpo se salía de la celda y rozaba. Lo solucionamos mirando también una franja de `MARGEN` píxeles alrededor de cada celda. Equivale a engordar los obstáculos.

```python
n_negros = np.sum(pixeles_negros[fila_min - MARGEN:fila_max + MARGEN,
                                 col_min - MARGEN:col_max + MARGEN])
if n_negros >= MINIMO_PIXELES_NEGROS:
    celda_ocupada[i][j] = True
```

- Un pasillo quedaba cerrado: Con el margen, el pasillo de 60 cm que rodea el mueble de la habitación de arriba a la derecha salía entero ocupado, porque las fronteras de las celdas caían junto a la pared. Probamos a desplazar la rejilla de 1 en 1 cm y elegimos la posición que abre el pasillo y deja más celdas libres.

```python
X_ESQUINA_REJILLA = -4.43
Y_ESQUINA_REJILLA = -3.55
```

- Una casilla junto al sofá de la izquierda: Con todo lo anterior, el robot aún se enganchaba en la esquina de ese sofá. En esa zona el sofá real está unos 9 cm más abajo de lo que indica el mapa con nuestra calibración, así que la casilla (15, 28) parecía libre, pero su centro quedaba a solo 15 cm del sofá y el robot mide unos 17 cm de radio. La marcamos a mano como ocupada.

```python
CASILLAS_BLOQUEADAS = [(15, 28)]

if n_negros >= MINIMO_PIXELES_NEGROS or (i, j) in CASILLAS_BLOQUEADAS:
    celda_ocupada[i][j] = True
```

## PASO 3: PLANIFICACIÓN DEL BSA Y NAVEGACIÓN

En este paso se calcula con BSA una ruta que pase por todas las celdas libres y después recorrerla.

- El robot sigue recto hasta toparse con una celda ocupada o visitada y entonces gira en el orden oeste → norte → este → sur.
- Las vecinas libres que no elige son posibles puntos de retorno.
- Cuando se queda sin salida (punto crítico), va al punto de retorno más cercano y empieza otra espiral.

La planificación se pinta en el navegador: verde son las celdas visitadas, rojo los puntos críticos y azul los puntos de retorno. Después, un control local lleva al robot por la ruta.

### Problemas y soluciones

- No hacía espirales: Primero hacía zigzag, porque la prioridad era fija respecto al mapa. Después, con la regla del artículo original (girar hacia el lado de referencia), bordeaba toda la casa en un solo anillo. La solución fue seguir recto y girar siempre en el mismo orden, empezando por la dirección actual.

```python
k = DIRECCIONES.index(direccion)
orden_prioridad = DIRECCIONES[k:] + DIRECCIONES[:k]   
```

- La vuelta al punto de retorno debía ser directa, no por cuadrículas: La búsqueda en anchura da un camino celda a celda. Lo simplificamos: desde cada punto saltamos al más lejano que se ve en línea recta sin pisar celdas ocupadas.

- El robot oscilaba de izquierda a derecha y avanzaba muy lento: Apuntaba al centro de cada celda con una ganancia demasiado alta. Lo arreglamos de dos formas:

    - Nos quedamos solo con los puntos donde hay que girar, para que en los tramos rectos apunte al final del tramo.
    - Bajamos la ganancia a `KP = 1` y hacemos que frene al acercarse a cada punto.

```python
HAL.setV(min(VELOCIDAD_MAXIMA, distancia))    
HAL.setW(KP * error_orientacion)              
```

- Se quedaba atascado contra alguna pared: Donde la pared real está más cerca de lo que dice el mapa, el robot empujaba sin llegar al centro de la celda. Ahora, si el láser ve una pared delante a menos de 25 cm, damos la celda por alcanzada aunque no esté justo en su centro.

```python
distancia_delante = HAL.getLaserData().values[90]
elif abs(error_orientacion) <= ANGULO_MAX and distancia_delante < DIST_PARED:
    HAL.setV(0)     
    n = n + 1
```

- Recortaba las esquinas: Dábamos cada punto por alcanzado cuando el robot estaba a menos de 20 cm, así que empezaba a girar hacia el siguiente antes de tiempo y, al doblar, rozaba la mesa del comedor. Bajamos la tolerancia a 5 cm.

```python
TOLERANCIA = 0.05         # a esta distancia (m) damos el punto por alcanzado
```

- Se quedaba atascado en algunos picos: Aún con el margen y el láser, durante la ejecución del vídeo y las pruebas previas, el robot se quedaba atascado en los mismos 4 picos, es decir, en esquinas salientes de paredes y muebles. Se debía sobre todo al descuadre entre el mapa y la casa real: en esas zonas el robot creía que tenía hueco para pasar y rozaba la esquina con un lateral. Además, el láser solo mira al frente, así que no detecta el pico. Cuando ocurría, movimos el robot a mano en Gazebo con la herramienta de traslación, desplazándolo a lo largo de sus ejes x e y hasta despegarlo. Después continuaba la ruta con normalidad. Al final vimos que el descuadre venía sobre todo del error de calibración del láser (paso 1). Con la calibración corregida, sin recortar las esquinas y con la casilla del sofá bloqueada (paso 2), el robot ya no se atasca en ningún sitio y además limpia debajo de la mesa baja. Durante las pruebas llegamos a rellenar la mesa en el mapa para que no entrase, pero con la calibración corregida ya no hace falta.

## VÍDEO Y MAPA BARRIDO 

[vídeo del funcionamiento](https://drive.google.com/file/d/1WHyEMaoOQjbOmiB-o_O3Paaxa037edG1/view?usp=sharing)

### Mapa barrido al terminar

<img width="589" height="593" alt="Captura desde 2026-10-07 19-37-25" src="https://github.com/user-attachments/assets/503f1cd8-f6dd-465f-ae83-e50b2010cfd5" />




