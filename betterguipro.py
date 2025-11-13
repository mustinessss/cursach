# -*- coding: utf-8 -*-
"""
Created on Fri Nov 14 01:29:42 2025

@author: ashst
"""

import tkinter as tk
import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle
import tkinter.messagebox

class TrilaterationApp:
    def __init__(self):
        self.ser = None
        self.running = False
        self.distances = [0, 0, 0]  # Инициализируем нулями

        # Известные точки
        self.point1 = (0, 250)
        self.point2 = (250, 250)
        self.point3 = (250, 0)

        # Создание графика
        self.fig = Figure(figsize=(5, 5), dpi=120)
        self.ax = self.fig.add_subplot(111)

        # Отображение известных точек
        self.ax.plot(self.point1[0], self.point1[1], 'rs', markersize=8, label='Динамик 1')
        self.ax.plot(self.point2[0], self.point2[1], 'bo', markersize=8, label='Динамик 2')
        self.ax.plot(self.point3[0], self.point3[1], 'g^', markersize=8, label='Динамик 3')

        # Отображение орбит
        self.circle1 = Circle(self.point1, 0, fill=False, color='r', linestyle='--', alpha=0.7)
        self.circle2 = Circle(self.point2, 0, fill=False, color='b', linestyle='--', alpha=0.7)
        self.circle3 = Circle(self.point3, 0, fill=False, color='g', linestyle='--', alpha=0.7)
        self.ax.add_patch(self.circle1)
        self.ax.add_patch(self.circle2)
        self.ax.add_patch(self.circle3)

        # Отображение неизвестной точки
        self.unknown_point, = self.ax.plot([], [], 'ko', markersize=8, label='Микрофон')

        # Настройка графика
        self.ax.set_aspect('equal')
        self.ax.set_xlim([-50, 300])
        self.ax.set_ylim([-50, 300])
        self.ax.set_xlabel('X (мм)')
        self.ax.set_ylabel('Y (мм)')
        self.ax.legend()
        self.ax.grid(True, alpha=0.3)  # Добавляем сетку для удобства

        # Создание GUI
        self.root = tk.Tk()
        self.root.geometry("800x600")
        self.root.title("Обработка данных учебного макета")
        
        # Получение доступных портов
        self.available_ports = [port.device for port in serial.tools.list_ports.comports()]
        print("Доступные порты:", self.available_ports)  # Для отладки

        # Элементы управления
        self.coordinate_units = tk.Label(self.root, text="COM порт:")
        self.coordinate_units.place(x=612, y=20)

        self.port_var = tk.StringVar(self.root)
        self.port_var.set(self.available_ports[0] if self.available_ports else "")
        self.port_menu = tk.OptionMenu(self.root, self.port_var, *self.available_ports)
        self.port_menu.config(width=20, height=2)
        self.port_menu.place(x=610, y=50)

        # Кнопки управления
        self.start_button = tk.Button(self.root, text="Старт", command=self.start_program)
        self.start_button.config(width=10, height=2)
        self.start_button.place(x=612, y=100)

        self.pause_button = tk.Button(self.root, text="Пауза", command=self.pause_program, state=tk.DISABLED)
        self.pause_button.config(width=10, height=2)
        self.pause_button.place(x=693, y=100)

        self.stop_button = tk.Button(self.root, text="Стоп", command=self.stop_program, state=tk.DISABLED)
        self.stop_button.config(width=22, height=2)
        self.stop_button.place(x=611, y=520)

        # Калибровка
        self.slider_var = tk.DoubleVar()
        self.slider_var.set(1.0)

        self.calibration_label = tk.Label(self.root, text="Калибровка:")
        self.calibration_label.place(x=612, y=150)

        self.slider = tk.Scale(self.root, variable=self.slider_var, orient=tk.HORIZONTAL, 
                              from_=0.5, to=1.5, resolution=0.01, command=self.slider_changed)
        self.slider.config(width=20, length=160)
        self.slider.place(x=611, y=170)

        # Координаты
        self.coordinate_label = tk.Label(self.root, text="Координата точки:")
        self.coordinate_label.place(x=612, y=220)

        self.coordinate_value = tk.StringVar()
        self.coordinate_value.set("(0.00 мм, 0.00 мм)")
        self.coordinate_text = tk.Label(self.root, textvariable=self.coordinate_value, font=("Arial", 10))
        self.coordinate_text.place(x=612, y=240)

        # Статус
        self.status_var = tk.StringVar()
        self.status_var.set("Ожидание подключения")
        self.status_label = tk.Label(self.root, textvariable=self.status_var, fg="blue")
        self.status_label.place(x=612, y=280)

        # График
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().place(x=0, y=0)

    def start_program(self):
        selected_port = self.port_var.get()
        if selected_port:
            try:
                self.ser = serial.Serial(selected_port, 9600, timeout=1)
                self.ser.reset_input_buffer()
                
                self.start_button.config(state=tk.DISABLED)
                self.pause_button.config(state=tk.NORMAL)
                self.stop_button.config(state=tk.NORMAL)
                self.running = True
                self.status_var.set("Подключено")
                self.update_plot()
                
            except serial.SerialException as e:
                error_msg = f"Ошибка подключения к {selected_port}:\n{str(e)}"
                print(error_msg)
                self.status_var.set("Ошибка подключения")
                tk.messagebox.showerror("Ошибка", error_msg)

    def pause_program(self):
        self.running = not self.running
        if self.running:
            self.pause_button.config(text="Пауза")
            self.status_var.set("Работает")
        else:
            self.pause_button.config(text="Продолжить")
            self.status_var.set("На паузе")

    def stop_program(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.start_button.config(state=tk.NORMAL)
        self.pause_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.DISABLED)
        self.status_var.set("Отключено")

    def update_plot(self):
        if self.running and self.ser:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf8', errors='ignore').strip()
                    print(f"Получены данные: '{data}'")
                    
                    if data and ',' in data:
                        try:
                            distances_str = data.split(',')
                            if len(distances_str) == 3:
                                self.distances = [
                                    float(distances_str[0]) * self.slider_var.get(),
                                    float(distances_str[1]) * self.slider_var.get(),
                                    float(distances_str[2]) * self.slider_var.get()
                                ]
                                print(f"Расстояния: {self.distances}")
                                
                                result = self.trilaterate(self.point1, self.point2, self.point3, 
                                                        self.distances[0], self.distances[1], self.distances[2])
                                
                                print(f"Координаты: ({result[0]:.2f}, {result[1]:.2f})")
                                
                                # Обновление графики
                                self.circle1.set_radius(self.distances[0])
                                self.circle2.set_radius(self.distances[1])
                                self.circle3.set_radius(self.distances[2])
                                self.unknown_point.set_data([result[0]], [result[1]])
                                self.coordinate_value.set(f"({result[0]:.2f} мм, {result[1]:.2f} мм)")
                                
                                self.canvas.draw_idle()
                                
                        except ValueError as e:
                            print(f"Ошибка преобразования: {e}")
                
            except Exception as e:
                print(f"Ошибка: {e}")
                self.status_var.set("Ошибка чтения")
        
        if self.running:
            self.root.after(50, self.update_plot)  # Уменьшил задержку для отзывчивости

    def trilaterate(self, p1, p2, p3, d1, d2, d3):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        A = 2 * (x2 - x1)
        B = 2 * (y2 - y1)
        C = d1**2 - d2**2 - x1**2 + x2**2 - y1**2 + y2**2
        D = 2 * (x3 - x2)
        E = 2 * (y3 - y2)
        F = d2**2 - d3**2 - x2**2 + x3**2 - y2**2 + y3**2

        try:
            x = (C*E - F*B) / (E*A - B*D)
            y = (C*D - A*F) / (B*D - A*E)
            return x, y
        except ZeroDivisionError:
            return 0, 0  # Возвращаем центр при ошибке вычислений

    def slider_changed(self, value):
        self.slider_var.set(float(value))

    def start(self):
        self.root.mainloop()

# Запуск приложения
app = TrilaterationApp()
app.start()