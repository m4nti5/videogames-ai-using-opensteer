opensteer es una librería de código libre que se puede obtener de: http://opensteer.sourceforge.net/ contiene librerías con implementaciones de diversos comportamientos de inteligencia artificial.

Utilizando parte de las librerías, con motivos académicos se reimplementaron los comportamientos utilizando librerías que ya poseen implementaciones de herramientas necesarias (librerías de vectores y opengl) siguiendo los algoritmos del libro Artificial Intelligence for Games de Ian Millington y John Funge. Hay 2 implementaciones:

- Comportamientos cinemáticos
- Comportamientos gobernados o "steering behaviours"

Si se presiona el botón F12 se cambiará entre los comportamientos cinemáticos o los gobernados. Presionando F1 y F2 se puede cambiar entre los comportamientos como tal. Se tienen los siguientes comportamientos implementados:

Comportamientos cinemáticos:
	Seek 
	Flee
	Arrive
	Wander

Comportamientos gobernados:
	Seek
	Flee
	Arrive
	Align
	Velocity Match
	Face
	Look where you're going
	Wander
	Pursue
	Evade
	Separation
	Follow Path
	Collision avoidance
	Blended steering
	Priority blending

Los objetos con triángulos azules son los controlados por los comportamientos mencionados y poseen una etiqueta identificando el comportamiento que poseen en ese momento. El objeto con triángulo rojo puede ser controlado por el usuario, sus controles son:

w incrementa la aceleración hacia adelante
s incrementa la aceleración hacia atrás
a incrementa la aceleración hacia la izquierda
d incrementa la aceleración hacia la derecha
barra espaciadora para saltar
f para lanzar un proyectil

Estos controles son relativos a la dirección del triángulo. Actualmente los proyectiles no tienen efectos sobre los agentes controlados automáticamente ni sobre el objeto 

