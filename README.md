# 🐢 MINI-PROJECT-1_EQUIPO2

Proyecto en ROS 2 que utiliza `turtlesim` para dibujar texto ingresado por terminal.

---

## 📌 DESCRIPCIÓN

Este nodo de ROS 2 permite dibujar cadenas de texto en el simulador `turtlesim`, utilizando desplazamientos y rotaciones reales para imitar la escritura a mano.

Cada carácter se compone mediante líneas rectas y/o curvas, y se traza con un controlador PID que garantiza movimientos suaves y precisos.

El sistema trabaja sobre una cuadrícula flexible definida por dos parámetros:

- `max_x`: ancho de cada letra  
- `max_y`: alto de cada letra  

Cuando la tortuga alcanza el borde derecho del lienzo, el sistema salta automáticamente a la siguiente línea.

---

## ▶️ USO
Debes tener 2 terminales abiertos.
En uno se lanzará la tortuga y en otro se lanzará el nodo que publica las órdenes a la tortuga.

### 1. Ejecutar el simulador `turtlesim`

En la primera terminal ejecuta:

```bash
ros2 run turtlesim turtlesim_node
```

### 2. Ejecutar roborescueminiproject1
En la segunda terminal:

```bash
ros2 run roborescueminiproject1 drawer "TEXTO A ESCRIBIR"
```

### ✏️ MODIFICAR TAMAÑO DE LETRA
Para cambiar el tamaño de las letras:

Abre el archivo drawer.py con un editor de texto (por ejemplo, VS Code).

Modifica las siguientes variables dentro del constructor (__init__):

self.max_x = 0.5  # Ancho de cada letra

self.max_y = 1.0  # Alto de cada letra

### 🔤 CARACTERES SOPORTADOS
El sistema actualmente admite:

Letras mayúsculas: A – Z

Espacios

Símbolos especiales: < y 3 (corazón)

## 🎥 VÍDEO MOSTRANDO FUNCIONAMIENTO

https://github.com/user-attachments/assets/0db97f9e-eb98-4484-b361-f50bc464caef

https://github.com/user-attachments/assets/3d9e40b0-300a-442b-bd5d-6eb53160eb41

