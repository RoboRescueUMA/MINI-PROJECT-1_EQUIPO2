# Importación de librerías necesarias
import rclpy                             # Cliente de ROS 2 en Python
from rclpy.node import Node              # Clase base para crear nodos en ROS 2
from turtlesim.srv import SetPen         # Servicio para configurar el lápiz de la tortuga
from turtlesim.msg import Pose           # Mensaje de posición de la tortuga
from geometry_msgs.msg import Twist      # Mensaje de velocidades lineales y angulares
import sys                               # Para acceder a los argumentos del terminal
import time                              # Para control de tiempos
import math                              # Funciones matemáticas (hipotenusa, atan2, etc.)

LETTER_SPACING = 0.5                     # Espacio horizontal entre letras

# Clase principal que hereda de Node
class TurtleWriter(Node):
    def __init__(self, text):
        super().__init__('turtle_writer')  # Nombre del nodo

        # Declaración de parámetros configurables (velocidades)
        self.declare_parameter('speed_linear', 2.0)
        self.declare_parameter('speed_angular', 5.0)

        #Declaración de parámetros configurables (tamaño letra)
        self.max_x = 0.5
        self.max_y = 1.0

        # Obtener los valores de los parámetros
        self.speed_linear = self.get_parameter('speed_linear').get_parameter_value().double_value
        self.speed_angular = self.get_parameter('speed_angular').get_parameter_value().double_value

        # Cliente para el servicio de configuración del lápiz
        self.cli_pen = self.create_client(SetPen, '/turtle1/set_pen')

        # Publicador de velocidades para mover la tortuga
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Suscripción a la posición de la tortuga
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Inicialización de la pose
        self.pose = Pose()
        self.pose_received = False  # Aún no se ha recibido la primera pose

        # Esperar a que el servicio de lápiz esté disponible
        while not self.cli_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio set_pen...')

        # Mapa de caracteres a métodos
        self.char_map = {
            'A': self.draw_A,
            'B': self.draw_B,
            'C': self.draw_C,
            'D': self.draw_D,
            'E': self.draw_E,
            'F': self.draw_F,
            'G': self.draw_G,
            'H': self.draw_H,
            'I': self.draw_I,
            'J': self.draw_J,
            'K': self.draw_K,
            'L': self.draw_L,
            'M': self.draw_M,
            'N': self.draw_N,
            'O': self.draw_O,
            'P': self.draw_P,
            'Q': self.draw_Q,
            'R': self.draw_R,
            'S': self.draw_S,
            'T': self.draw_T,
            'U': self.draw_U,
            'V': self.draw_V,
            'W': self.draw_W,
            'X': self.draw_X,
            'Y': self.draw_Y,
            'Z': self.draw_Z,
            '<': self.draw_less_than,
            '3': self.draw_3,
            ' ': self.draw_space,
        }

        # Esperar a recibir la primera pose antes de empezar a dibujar
        while not self.pose_received:
            rclpy.spin_once(self)

        # Ir a la posición inicial
        self.call_pen(0, 0, 0, 3, 1)  # Apaga el lápiz
        self.get_logger().info("Moviéndose a la posición inicial (0.5, 6.0)...")
        self.linea_y = 10 - self.max_y
        self.go_to(0.5, self.linea_y)

        # Llamar al método principal de dibujo
        self.draw_text(text.upper())

    # Callback que actualiza la pose de la tortuga
    def pose_callback(self, msg):
        self.pose = msg
        self.pose_received = True

    # Llama al servicio de lápiz con los parámetros dados
    def call_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = self.cli_pen.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def go_to(self, target_x, target_y):
        """Control PID completo para moverse en línea recta hacia un punto (x, y) con orientación precisa."""

        # PID para distancia
        Kp_dist = 1.5
        Ki_dist = 0.1
        Kd_dist = 0.05

        # PID para ángulo
        Kp_ang = 4.0
        Ki_ang = 0.2
        Kd_ang = 0.2

        max_lin_speed = self.speed_linear
        max_ang_speed = self.speed_angular

        distance_tolerance = 0.03
        timeout = 10.0  # máximo tiempo para intentar moverse

        # Variables PID
        prev_dist_error = 0.0
        int_dist_error = 0.0

        prev_ang_error = 0.0
        int_ang_error = 0.0

        twist = Twist()
        t_start = time.time()

        dt = 0.02  # periodo de control (50 Hz)

        while rclpy.ok():
            # Cálculo del error de distancia
            dx = target_x - self.pose.x
            dy = target_y - self.pose.y
            distance = math.hypot(dx, dy)

            if distance < distance_tolerance or time.time() - t_start > timeout:
                break  # llegó o se acabó el tiempo

            # Error angular
            angle_to_target = math.atan2(dy, dx)
            ang_error = (angle_to_target - self.pose.theta + math.pi) % (2 * math.pi) - math.pi

            # Derivadas
            d_dist_error = (distance - prev_dist_error) / dt
            d_ang_error = (ang_error - prev_ang_error) / dt

            # Acumuladores integrales
            int_dist_error += distance * dt
            int_ang_error += ang_error * dt

            # Salidas PID
            linear_output = (Kp_dist * distance +
                            Ki_dist * int_dist_error +
                            Kd_dist * d_dist_error)

            angular_output = (Kp_ang * ang_error +
                            Ki_ang * int_ang_error +
                            Kd_ang * d_ang_error)

            # Aplicar límites
            twist.linear.x = min(linear_output, max_lin_speed)
            twist.angular.z = max(min(angular_output, max_ang_speed), -max_ang_speed)

            # Si está muy desalineada, que solo gire
            if abs(ang_error) > math.pi / 4:
                twist.linear.x = 0.0

            self.publisher_.publish(twist)

            # Actualizar errores
            prev_dist_error = distance
            prev_ang_error = ang_error

            rclpy.spin_once(self)
            time.sleep(dt)

        # Detener
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


    # Mueve la tortuga a la derecha para dejar espacio a la siguiente letra
    def move_right(self, step=LETTER_SPACING):
        self.call_pen(0, 0, 0, 3, 1)  # Apaga el lápiz
        next_x = self.pose.x + step
        next_y = self.pose.y

        # Si se sale del borde, salta a la siguiente línea
        if next_x + self.max_x > 11.0:
            next_x = 0.5
            next_y = self.linea_y - (self.max_y + 1)
            self.get_logger().warn("¡Cambio de línea!")

            if next_y < 1.0:
                self.get_logger().warn("¡No hay más espacio en el lienzo!")
                return

            self.linea_y = next_y

        self.go_to(next_x, next_y)

    # Dibuja el texto completo carácter a carácter
    def draw_text(self, text):
        for char in text:
            if char in self.char_map:
                self.get_logger().info(f'Dibujando {char}')
                self.char_map[char]()  # Llama a la función de dibujo correspondiente
            else:
                self.get_logger().warn(f'Caracter no soportado: {char}')
            self.move_right()  # Se mueve a la posición de la siguiente letra

    # Dibuja un espacio (no hace nada)
    def draw_space(self):
        pass

#A partir de aquí las letras!!!!!!!!!
    def draw_A(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # PenOn
        self.call_pen(255, 255, 255, 3, 0)

        # Línea ascendente izquierda
        self.go_to(base_x, base_y + self.max_y)

        # Línea horizontal superior
        self.go_to(base_x + self.max_x, base_y + self.max_y)

        # Línea descendente derecha
        self.go_to(base_x + self.max_x, base_y)

        # Línea horizontal en el medio
        self.call_pen(0, 0, 0, 3, 1)  # Apaga
        self.go_to(base_x + self.max_x, base_y + (self.max_y/2.0))
        self.call_pen(255, 255, 255, 3, 0)  # Enciende
        self.go_to(base_x, base_y + (self.max_y/2.0))

        # Vuelve al punto base sin pintar
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + (self.max_x), base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra A dibujada correctamente.')

    def draw_B(self):
        base_x = self.pose.x
        base_y = self.pose.y
        w = self.max_x
        h = self.max_y

        mid_y = base_y + h / 2
        right_x = base_x + w

        # Apaga lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Enciende lápiz
        self.call_pen(255, 255, 255, 3, 0)

        # Línea vertical izquierda
        self.go_to(base_x, base_y + h)

        # Diagonal a parte superior derecha
        self.go_to(right_x, base_y + 3*h/4)

        # Diagonal de regreso al centro
        self.go_to(base_x, mid_y)

        # Diagonal a parte inferior derecha
        self.go_to(right_x, base_y + h/4)

        # Diagonal de regreso a la base
        self.go_to(base_x, base_y)

        # Apaga lápiz y vuelve a base derecha
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + w, base_y)
        self.call_pen(0, 0, 255, 3, 0)

        self.get_logger().info('Letra B dibujada correctamente.')

    def draw_C(self):
            base_x = self.pose.x
            base_y = self.pose.y

            # Apaga lápiz y vuelve al punto base
            self.call_pen(0, 0, 0, 3, 1)
            self.go_to(base_x + self.max_x, base_y)

            #Secuencia C
            self.call_pen(255, 255, 255, 3, 0) #PenOn
            self.go_to(base_x, base_y)
            self.go_to(base_x, base_y + self.max_y)
            self.go_to(base_x + self.max_x, base_y + self.max_y)

            # Apaga lápiz y vuelve a la base derecha para la siguiente letra
            self.call_pen(0, 0, 0, 3, 1)
            self.go_to(base_x + self.max_x, base_y)
            self.call_pen(255, 255, 255, 3, 0)

            self.get_logger().info('Letra C dibujada correctamente.')

    def draw_D(self):
        base_x = self.pose.x
        base_y = self.pose.y
        radius = self.max_x
        steps = 10  # Cuantos segmentos para simular la curva

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Enciende el lápiz
        self.call_pen(255, 255, 255, 3, 0)

        # Línea vertical izquierda
        self.go_to(base_x, base_y + self.max_y)

        # Simular curva derecha (mitad de un círculo)
        for i in range(steps + 1):
            theta = math.pi * i / steps  # De 0 a PI
            x = base_x + radius * math.sin(theta)
            y = base_y + (self.max_y/2.0) + radius * math.cos(theta)
            self.go_to(x, y)

        # Apaga el lápiz
        self.call_pen(0, 0, 0, 3, 1)

        # Vuelve a la base derecha inferior para la siguiente letra
        self.go_to(base_x + radius, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra D dibujada correctamente.')

    def draw_E(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apagar lápiz y moverse al punto inicial
        self.call_pen(0, 0, 0, 3, 1)    # Pen off
        self.go_to(base_x, base_y)
        time.sleep(0.1)

        # Encender lápiz para línea vertical izquierda
        self.call_pen(255, 255, 255, 3, 0)  # Pen on
        self.go_to(base_x, base_y + self.max_y)
        time.sleep(0.1)

        # Levantar lápiz para mover sin dibujar al inicio línea horizontal superior
        self.call_pen(0, 0, 0, 3, 1)    # Pen off
        self.go_to(base_x, base_y + self.max_y)
        time.sleep(0.1)

        # Bajar lápiz y dibujar línea horizontal superior
        self.call_pen(255, 255, 255, 3, 0)  # Pen on
        self.go_to(base_x + self.max_x, base_y + self.max_y)
        time.sleep(0.1)

        # Levantar lápiz y moverse sin dibujar al inicio línea horizontal media
        self.call_pen(0, 0, 0, 3, 1)    # Pen off
        self.go_to(base_x, base_y + (self.max_y/2.0))
        time.sleep(0.1)

        # Bajar lápiz y dibujar línea horizontal media
        self.call_pen(255, 255, 255, 3, 0)  # Pen on
        self.go_to(base_x + self.max_x, base_y + (self.max_y/2.0))
        time.sleep(0.1)

        # Levantar lápiz y moverse sin dibujar a la base para línea inferior
        self.call_pen(0, 0, 0, 3, 1)    # Pen off
        self.go_to(base_x, base_y)
        time.sleep(0.1)

        # Bajar lápiz y dibujar línea horizontal inferior
        self.call_pen(255, 255, 255, 3, 0)  # Pen on
        self.go_to(base_x + self.max_x, base_y)
        time.sleep(0.1)

        # Apagar lápiz y volver a posición base
        self.call_pen(0, 0, 0, 3, 1)    # Pen off
        self.go_to(base_x + self.max_x, base_y)
        time.sleep(0.1)

        self.get_logger().info('Letra E dibujada correctamente.')

    def draw_F(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Enciende el lápiz
        self.call_pen(255, 255, 255, 3, 0)

        # Línea vertical izquierda
        self.go_to(base_x, base_y + self.max_y)

        # Línea horizontal superior
        self.go_to(base_x + self.max_x, base_y + self.max_y)

        # Apaga lápiz, se reposiciona para la línea media
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y + (self.max_y/2.0))

        # Enciende lápiz y dibuja línea media (más corta)
        self.call_pen(255, 255, 255, 3, 0)
        self.go_to(base_x + self.max_x, base_y + (self.max_y/2.0))

        # Apaga lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra F dibujada correctamente.')

    def draw_G(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        #Secuencia G
        self.go_to(base_x + self.max_x, base_y + self.max_y)
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x, base_y + self.max_y)
        self.go_to(base_x, base_y)
        self.go_to(base_x + self.max_x, base_y)
        self.go_to(base_x + self.max_x, base_y + (self.max_y/2))
        self.go_to(base_x + (self.max_x/2), base_y + (self.max_y/2))

        # Apaga lápiz y vuelve al punto base derecho
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra G dibujada correctamente.')

    def draw_H(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Enciende el lápiz
        self.call_pen(255, 255, 255, 3, 0)

        # Lado izquierdo (línea vertical completa)
        self.go_to(base_x, base_y + self.max_y)

        # Apaga lápiz, baja al centro de la H
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y + (self.max_y/2.0))

        # Enciende lápiz, trazo horizontal
        self.call_pen(255, 255, 255, 3, 0)
        self.go_to(base_x + self.max_x, base_y + (self.max_y/2.0))

        # Apaga lápiz, sube al tope derecho
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y + self.max_y)

        # Enciende lápiz, baja línea derecha completa
        self.call_pen(255, 255, 255, 3, 0)
        self.go_to(base_x + self.max_x, base_y)

        # Apaga lápiz y vuelve a base derecha
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra H dibujada correctamente.')

    def draw_I(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Secuencia I
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(0, 0, 0, 3, 1) #PenOff
        self.go_to(base_x, base_y + self.max_y)
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x + self.max_x, base_y + self.max_y)
        self.call_pen(0, 0, 0, 3, 1) #PenOff
        self.go_to(base_x + (self.max_x/2.0), base_y + self.max_y)
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x + (self.max_x/2.0), base_y)

       
        # Apaga lápiz y vuelve a base derecha
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)
        self.get_logger().info('Letra I dibujada correctamente.')

    def draw_J(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y + self.max_y)

        # Secuencia J
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x + self.max_x, base_y + self.max_y)
        self.call_pen(0, 0, 0, 3, 1) #PenOff
        self.go_to(base_x + (self.max_x/2.0), base_y + self.max_y)
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x + (self.max_x/2.0), base_y)
        self.go_to(base_x, base_y)
        

        # Apaga lápiz y vuelve a base derecha
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)
        self.get_logger().info('Letra J dibujada correctamente.')

    def draw_K(self):
            base_x = self.pose.x
            base_y = self.pose.y

            # Apaga el lápiz y vuelve al punto base
            self.call_pen(0, 0, 0, 3, 1)
            self.go_to(base_x, base_y)

            # Secuencia K
            self.call_pen(255, 255, 255, 3, 0) #PenOn
            self.go_to(base_x, base_y + self.max_y)
            self.call_pen(0, 0, 0, 3, 1) #PenOff
            self.go_to(base_x + self.max_x, base_y + self.max_y)
            self.call_pen(255, 255, 255, 3, 0) #PenOn
            self.go_to(base_x, base_y + (self.max_y/2.0))
            self.go_to(base_x + self.max_x, base_y)
        
            # Apaga lápiz y vuelve a base derecha
            self.call_pen(0, 0, 0, 3, 1)
            self.go_to(base_x + self.max_x, base_y)
            self.call_pen(255, 255, 255, 3, 0)
            self.get_logger().info('Letra I dibujada correctamente.')

    def draw_L(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y mueve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Línea vertical hacia arriba
        self.call_pen(255, 255, 255, 3, 0)  # PenOn
        self.go_to(base_x, base_y + self.max_y)

        # Apaga lápiz y baja sin trazar
        self.call_pen(0, 0, 0, 3, 1)  # PenOff
        self.go_to(base_x, base_y)

        # Línea horizontal hacia la derecha (base)
        self.call_pen(255, 255, 255, 3, 0)  # PenOn
        self.go_to(base_x + self.max_x, base_y)

        # Apaga lápiz y vuelve a base derecha
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra L dibujada correctamente.')

    def draw_M(self):
            base_x = self.pose.x
            base_y = self.pose.y

            # Apaga lápiz y vuelve al punto base
            self.call_pen(0, 0, 0, 3, 1)
            self.go_to(base_x, base_y)

            #Secuencia M
            self.call_pen(255, 255, 255, 3, 0) #PenOn
            self.go_to(base_x, base_y + self.max_y)
            self.go_to(base_x + (self.max_x/2.0), base_y + (self.max_y/2.0))
            self.go_to(base_x + self.max_x, base_y + self.max_y)
            self.go_to(base_x + self.max_x, base_y)

            # Apaga lápiz y vuelve a la base derecha para la siguiente letra
            self.call_pen(0, 0, 0, 3, 1)
            self.go_to(base_x + self.max_x, base_y)
            self.call_pen(255, 255, 255, 3, 0)

            self.get_logger().info('Letra M dibujada correctamente.')

    def draw_N(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Enciende lápiz para dibujar la línea izquierda vertical
        self.call_pen(255, 255, 255, 3, 0)
        self.go_to(base_x, base_y + self.max_y)

        # Dibuja diagonal desde arriba izquierda a base derecha
        self.go_to(base_x + self.max_x, base_y)

        # Dibuja línea vertical derecha
        self.go_to(base_x + self.max_x, base_y + self.max_y)

        # Apaga lápiz y vuelve a la base derecha para siguiente letra
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra N dibujada correctamente.')

    def draw_O(self):
        base_x = self.pose.x
        base_y = self.pose.y

        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)
        time.sleep(0.1)

        self.call_pen(255, 255, 255, 3, 0) #Pen On
        time.sleep(0.1)

        # Base de derecha a izquierda
        self.go_to(base_x + self.max_x, base_y)
        time.sleep(0.1)

        # Vertical derecha
        self.go_to(base_x + self.max_x, base_y + self.max_y)
        time.sleep(0.1)

        # Línea superior de izquierda a derecha
        self.go_to(base_x, base_y + self.max_y)
        time.sleep(0.1)

        # Vertical izquierda
        self.go_to(base_x, base_y)
        time.sleep(0.1)

        # Reset posición
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra O dibujada correctamente.')

    def draw_P(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Dibuja línea vertical izquierda (base de la P)
        self.call_pen(255, 255, 255, 3, 0)  # PenOn
        self.go_to(base_x, base_y + self.max_y)

        # Dibuja línea horizontal superior (parte redondeada superior)
        self.go_to(base_x + self.max_x, base_y + self.max_y)

        # Dibuja línea vertical derecha superior
        self.go_to(base_x + self.max_x, base_y + (self.max_y/2.0))

        # Dibuja línea horizontal media (cierra la parte superior de la P)
        self.go_to(base_x, base_y + (self.max_y/2.0))

        # Apaga lápiz y vuelve a la base derecha para siguiente letra
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra P dibujada correctamente.')

    def draw_Q(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Dibuja la "O" (círculo rectangular)
        self.call_pen(255, 255, 255, 3, 0)  # PenOn
        self.go_to(base_x + self.max_x, base_y)
        self.go_to(base_x + self.max_x, base_y + self.max_y)
        self.go_to(base_x, base_y + self.max_y)
        self.go_to(base_x, base_y)
        
        # Traza la cola de la Q
        self.call_pen(0, 0, 0, 3, 1)  # PenOff
        self.go_to(base_x + (self.max_x/2.0), base_y + (self.max_y/2.0))
        self.call_pen(255, 255, 255, 3, 0)  # PenOn
        self.go_to(base_x + (self.max_x+self.max_x*0.1), base_y - (self.max_y*0.1))

        # Apaga lápiz y vuelve a la base derecha para la siguiente letra
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra Q dibujada correctamente.')

    def draw_R(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Secuencia R
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x, base_y + self.max_y)
        self.go_to(base_x + self.max_x, base_y + self.max_y)
        self.go_to(base_x + self.max_x, base_y + (self.max_y/2.0))
        self.go_to(base_x, base_y + (self.max_y/2.0))
        self.go_to(base_x + self.max_x, base_y)

       
        # Apaga lápiz y vuelve a base derecha
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)
        self.get_logger().info('Letra R dibujada correctamente.')

    def draw_S(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apagar lápiz y mover a posición inicial (arriba derecha)
        self.call_pen(0, 0, 0, 3, 1)  # Pen off
        self.go_to(base_x + self.max_x, base_y + self.max_y)
        time.sleep(0.1)

        # Línea horizontal superior (de derecha a izquierda)
        self.call_pen(255, 255, 255, 3, 0)  # Pen on
        self.go_to(base_x, base_y + self.max_y)
        time.sleep(0.1)

        # Línea vertical descendente corta izquierda (media)
        self.call_pen(255, 255, 255, 3, 0)  # Pen on
        self.go_to(base_x, base_y + (self.max_y/2.0))
        time.sleep(0.1)

        # Línea horizontal media (de izquierda a derecha)
        self.call_pen(255, 255, 255, 3, 0)  # Pen on
        self.go_to(base_x + self.max_x, base_y + (self.max_y/2.0))
        time.sleep(0.1)

        # Línea vertical descendente corta derecha (media baja)
        self.call_pen(255, 255, 255, 3, 0)  # Pen on
        self.go_to(base_x + self.max_x, base_y + 0.0)
        time.sleep(0.1)

        # Línea horizontal inferior (de derecha a izquierda)
        self.call_pen(255, 255, 255, 3, 0)  # Pen on
        self.go_to(base_x, base_y)
        time.sleep(0.1)

        # Apagar lápiz y volver a base
        self.call_pen(0, 0, 0, 3, 1)    # Pen off
        self.go_to(base_x + self.max_x, base_y)
        time.sleep(0.1)

        self.get_logger().info('Letra S dibujada correctamente.')

    def draw_T(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        #Secuencia T
        self.go_to(base_x, base_y + self.max_y)
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x + self.max_x, base_y + self.max_y)
        self.call_pen(0, 0, 0, 3, 1) #PenOff
        self.go_to(base_x + (self.max_x/2), base_y + self.max_y)
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x + (self.max_x/2), base_y )

        # Apaga lápiz y vuelve a la base derecha para la siguiente letra
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra T dibujada correctamente.')

    def draw_U(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        #Secuencia U
        self.go_to(base_x + self.max_x, base_y + self.max_y)
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x + self.max_x, base_y)
        self.go_to(base_x, base_y)
        self.go_to(base_x, base_y + self.max_y )

        # Apaga lápiz y vuelve a la base derecha para la siguiente letra
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra U dibujada correctamente.')

    def draw_V(self):
            base_x = self.pose.x
            base_y = self.pose.y

            # Apaga lápiz y vuelve al punto base
            self.call_pen(0, 0, 0, 3, 1)
            self.go_to(base_x, base_y)

            #Secuencia V
            self.go_to(base_x, base_y + self.max_y)
            self.call_pen(255, 255, 255, 3, 0) #PenOn
            self.go_to(base_x + (self.max_x/2), base_y)
            self.go_to(base_x + self.max_x, base_y + self.max_y)

            # Apaga lápiz y vuelve a la base derecha para la siguiente letra
            self.call_pen(0, 0, 0, 3, 1)
            self.go_to(base_x + self.max_x, base_y)
            self.call_pen(255, 255, 255, 3, 0)

            self.get_logger().info('Letra V dibujada correctamente.')

    def draw_W(self):
            base_x = self.pose.x
            base_y = self.pose.y

            paso = self.max_x/4.0

            # Apaga lápiz y vuelve al punto base
            self.call_pen(0, 0, 0, 3, 1)
            self.go_to(base_x, base_y)

            #Secuencia W
            self.go_to(base_x, base_y + self.max_y)
            self.call_pen(255, 255, 255, 3, 0) #PenOn
            self.go_to(base_x + paso, base_y)
            self.go_to(base_x + 2*paso, base_y + (self.max_y/2.0))
            self.go_to(base_x + 3*paso, base_y)
            self.go_to(base_x + self.max_x, base_y + self.max_y)

            # Apaga lápiz y vuelve a la base derecha para la siguiente letra
            self.call_pen(0, 0, 0, 3, 1)
            self.go_to(base_x + self.max_x, base_y)
            self.call_pen(255, 255, 255, 3, 0)

            self.get_logger().info('Letra W dibujada correctamente.')

    def draw_X(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # Dibuja la diagonal de izquierda abajo a derecha arriba
        self.call_pen(255, 255, 255, 3, 0)  # PenOn
        self.go_to(base_x + self.max_x, base_y + self.max_y)

        # Apaga lápiz para moverse sin pintar
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)

        # Dibuja la diagonal de derecha abajo a izquierda arriba
        self.call_pen(255, 255, 255, 3, 0)  # PenOn
        self.go_to(base_x, base_y + self.max_y)

        # Apaga lápiz y vuelve a la base derecha para la siguiente letra
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra X dibujada correctamente.')

    def draw_Y(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y)

        # PenOn
        self.call_pen(255, 255, 255, 3, 0)

        # Brazo izquierdo de la Y (mitad superior)
        self.go_to(base_x + (self.max_x/2), base_y + (self.max_y/2.0))
        self.go_to(base_x, base_y + self.max_y)

        # Apaga lápiz y mueve al inicio del brazo derecho
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y + self.max_y)

        # PenOn y traza brazo derecho
        self.call_pen(255, 255, 255, 3, 0)
        self.go_to(base_x + (self.max_x/2), base_y + (self.max_y/2.0))

        # Apaga lápiz y vuelve a base derecha
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra Y dibujada correctamente.')

    def draw_Z(self):
        base_x = self.pose.x
        base_y = self.pose.y

        # Apaga el lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y + self.max_y)

        # PenOn y línea superior horizontal
        self.call_pen(255, 255, 255, 3, 0)
        self.go_to(base_x + self.max_x, base_y + self.max_y)

        # Diagonal descendente
        self.go_to(base_x, base_y)

        # Línea inferior horizontal
        self.go_to(base_x + self.max_x, base_y)

        # Apaga lápiz y vuelve a base derecha
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Letra Z dibujada correctamente.')

    def draw_less_than(self):
                base_x = self.pose.x
                base_y = self.pose.y

                # Apaga lápiz y vuelve al punto base
                self.call_pen(0, 0, 0, 3, 1)
                self.go_to(base_x + self.max_x , base_y + self.max_y)

                #Secuencia <
                self.call_pen(255, 255, 255, 3, 0) #PenOn
                self.go_to(base_x, base_y + (self.max_y/2.0))
                self.go_to(base_x + self.max_x, base_y)
                

                # Apaga lápiz y vuelve a la base derecha para la siguiente letra
                self.call_pen(0, 0, 0, 3, 1)
                self.go_to(base_x + self.max_x, base_y)
                self.call_pen(255, 255, 255, 3, 0)

                self.get_logger().info('Caracter < dibujado correctamente.')

    def draw_3(self):
        base_x = self.pose.x
        base_y = self.pose.y
        paso = self.max_y/4.0

        # Apaga lápiz y vuelve al punto base
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x, base_y + self.max_y)

        #Secuencia <
        self.call_pen(255, 255, 255, 3, 0) #PenOn
        self.go_to(base_x + self.max_x, base_y + 3*paso)
        self.go_to(base_x, base_y + 2*paso)
        self.go_to(base_x + self.max_x, base_y + paso)
        self.go_to(base_x, base_y)        

            # Apaga lápiz y vuelve a la base derecha para la siguiente letra
        self.call_pen(0, 0, 0, 3, 1)
        self.go_to(base_x + self.max_x, base_y)
        self.call_pen(255, 255, 255, 3, 0)

        self.get_logger().info('Numero 3 dibujado correctamente.')


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Uso: ros2 run roborescueminiproject1 drawer \"TEXTO\"")
        return

    texto = sys.argv[1]
    writer = TurtleWriter(texto)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
