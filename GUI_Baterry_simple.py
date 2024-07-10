#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from tkinter import Tk, Label, StringVar
import threading

class PowerCalculator(Node):

    def __init__(self):
        super().__init__('power_calculator')

        # Variables para almacenar voltaje, corriente y tiempo
        self.voltage = 0.0
        self.current = 0.0
        self.elapsed_time = 0.0

        # Suscriptores
        self.voltage_sub_ = self.create_subscription(BatteryState, 'voltaje', self.voltage_callback, 10)
        self.current_sub_ = self.create_subscription(BatteryState, 'corriente', self.current_callback, 10)

        # Timer para calcular y actualizar resultados periódicamente
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1 segundos de intervalo

        # Tiempo de inicio
        self.start_time = self.get_clock().now()

    def voltage_callback(self, msg):
        self.voltage = msg.voltage
        voltage_var.set(f"{self.voltage:.2f}")

    def current_callback(self, msg):
        self.current = msg.current
        current_var.set(f"{self.current:.2f}")

    def timer_callback(self):
        # Calcular tiempo transcurrido
        current_time = self.get_clock().now()
        self.elapsed_time = (current_time - self.start_time).nanoseconds / 1e9 / 3600  # Convertir a horas

        # Calcular potencia
        power = self.voltage * self.current

        # Calcular capacidad usada y energía usada
        capacity_used = self.current * self.elapsed_time
        energy_used = power * self.elapsed_time

        # Actualizar valores en la interfaz gráfica
        power_var.set(f"{power:.2f}")
        capacity_var.set(f"{capacity_used:.2f}")
        energy_var.set(f"{energy_used:.2f}")

def ros_thread():
    rclpy.init()
    power_calculator = PowerCalculator()
    rclpy.spin(power_calculator)
    power_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Crear la interfaz gráfica de tkinter
    root = Tk()
    root.title("Power Calculator")

    voltage_var = StringVar()
    current_var = StringVar()
    power_var = StringVar()
    capacity_var = StringVar()
    energy_var = StringVar()

    Label(root, text="Voltage (V):").grid(row=0, column=0)
    Label(root, textvariable=voltage_var).grid(row=0, column=1)

    Label(root, text="Current (A):").grid(row=1, column=0)
    Label(root, textvariable=current_var).grid(row=1, column=1)

    Label(root, text="Power (W):").grid(row=2, column=0)
    Label(root, textvariable=power_var).grid(row=2, column=1)

    Label(root, text="Capacity Used (Ah):").grid(row=3, column=0)
    Label(root, textvariable=capacity_var).grid(row=3, column=1)

    Label(root, text="Energy Used (Wh):").grid(row=4, column=0)
    Label(root, textvariable=energy_var).grid(row=4, column=1)

    # Crear un hilo separado para ROS 2
    threading.Thread(target=ros_thread, daemon=True).start()

    # Iniciar el bucle principal de tkinter
    root.mainloop()
