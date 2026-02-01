#!/usr/bin/env python3
# ============================================================
# Nodo ROS 2 con comunicación bidireccional UART
#
# - Lee por UART: temp,u,ref
# - Publica:
#     /temp        (Float32)
#     /u           (Float32)
#     /ref         (Float32)
#     /temp_u_ref  (String: "temp,u,ref")
# - Recibe por ROS:
#     /ref_cmd     (Float32)
# - Envía por UART comandos de referencia en formato:
#     REF:<valor>\n
#
# Frecuencia aproximada de lectura: 100 Hz
# ============================================================

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String
import serial

# ------------------------------------------------------------
# Configuración UART
# ------------------------------------------------------------
PORT = "/dev/serial0"     # Puerto serial (Raspberry Pi)
BAUD = 57600              # Velocidad en baudios

# ------------------------------------------------------------
# Definición del nodo ROS 2
# ------------------------------------------------------------
class TempURefNode(Node):
    """
    Nodo ROS 2 que:
    - Lee temperatura, señal de control y referencia desde UART
    - Publica los valores en tópicos ROS 2
    - Recibe comandos de referencia desde ROS y los envía por UART
    """

    def __init__(self):
        # Inicialización del nodo
        super().__init__('temp_u_ref_node')

        # ----------------------------------------------------
        # Publicadores ROS 2
        # ----------------------------------------------------
        self.pub_temp = self.create_publisher(Float32, 'temp', 10)
        self.pub_u    = self.create_publisher(Float32, 'u', 10)
        self.pub_ref  = self.create_publisher(Float32, 'ref', 10)

        # Publicador de la línea completa como string
        self.pub_line = self.create_publisher(String, 'temp_u_ref', 10)

        # ----------------------------------------------------
        # Suscriptor para comandos de referencia
        # ----------------------------------------------------
        self.sub_ref_cmd = self.create_subscription(
            Float32,
            '/ref_cmd',
            self.on_ref_cmd,
            10
        )

        # Último comando enviado (para evitar spam por UART)
        self.last_ref_cmd = None

        # ----------------------------------------------------
        # Inicialización del puerto serial
        # ----------------------------------------------------
        self.get_logger().info(f"Abriendo UART: {PORT} @ {BAUD}")
        self.ser = serial.Serial(PORT, BAUD, timeout=1)

        # Pausa breve para estabilizar UART
        time.sleep(0.2)

        # ----------------------------------------------------
        # Temporizador (~100 Hz)
        # ----------------------------------------------------
        self.timer = self.create_timer(0.01, self.timer_callback)

    # --------------------------------------------------------
    # Callback del suscriptor /ref_cmd
    # --------------------------------------------------------
    def on_ref_cmd(self, msg: Float32):
        """
        Recibe un nuevo valor de referencia desde ROS,
        lo limita a un rango seguro y lo envía por UART.
        """

        ref_cmd = float(msg.data)

        # Seguridad: limitar rango permitido
        if ref_cmd < 0.0:
            ref_cmd = 0.0
        if ref_cmd > 120.0:
            ref_cmd = 120.0

        # Evitar spam UART:
        # solo enviar si cambia más de 0.05
        if self.last_ref_cmd is None or abs(ref_cmd - self.last_ref_cmd) > 0.05:
            cmd = f"REF:{ref_cmd:.2f}\n"
            try:
                self.ser.write(cmd.encode())
                self.last_ref_cmd = ref_cmd
                self.get_logger().info(f"Enviado por UART -> {cmd.strip()}")
            except Exception as e:
                self.get_logger().error(f"Error enviando REF por UART: {e}")

    # --------------------------------------------------------
    # Callback del temporizador
    # --------------------------------------------------------
    def timer_callback(self):
        """
        Lee una línea desde UART, la convierte a float
        y publica los valores en los tópicos ROS 2.
        """

        # Lectura de una línea desde UART
        line = self.ser.readline()

        # Si no hay datos, salir
        if not line:
            return

        try:
            # Decodifica y separa valores
            # Formato esperado: temp,u,ref
            temp, u, ref = map(
                float,
                line.decode(errors="replace").strip().split(',')
            )

            # Publicación de la línea completa como String
            s = String()
            s.data = f"{temp:.2f},{u:.2f},{ref:.2f}"
            self.pub_line.publish(s)

        except ValueError:
            # Línea inválida o incompleta
            return

        # Mensaje reutilizable Float32
        m = Float32()

        # Publicación de temperatura
        m.data = temp
        self.pub_temp.publish(m)

        # Publicación de señal de control
        m.data = u
        self.pub_u.publish(m)

        # Publicación de referencia
        m.data = ref
        self.pub_ref.publish(m)

# ------------------------------------------------------------
# Función principal
# ------------------------------------------------------------
def main():
    # Inicialización de ROS 2
    rclpy.init()

    # Creación del nodo
    node = TempURefNode()

    try:
        # Mantiene el nodo activo
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Permite salir con Ctrl+C
        pass
    finally:
        # Cierre ordenado de recursos
        try:
            node.ser.close()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()

# ------------------------------------------------------------
# Punto de entrada
# ------------------------------------------------------------
if __name__ == '__main__':
    main()
