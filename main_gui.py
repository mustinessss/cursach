import tkinter as tk
from tkinter import messagebox, filedialog
import serial
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle
import threading
import queue
from game_part import MazeGame, MazeLoader, SimpleMaze

class TrilaterationApp:
    def __init__(self):
        self.ser = None
        self.running = False
        self.distances = [0, 0, 0]
        self.data_queue = queue.Queue()

        # Известные точки
        self.point1 = (0, 250)
        self.point2 = (250, 250)
        self.point3 = (250, 0)

        # Создание графика
        self.fig = Figure(figsize=(5, 5), dpi=120)
        self.ax = self.fig.add_subplot(111)

        # Отображение известных точек
        self.ax.plot(self.point1[0], self.point1[1], 'rs', markersize=8)
        self.ax.plot(self.point2[0], self.point2[1], 'bo', markersize=8)
        self.ax.plot(self.point3[0], self.point3[1], 'g^', markersize=8)

        # Отображение орбит
        self.circle1 = Circle(self.point1, 0, fill=False, color='r', linestyle='--', alpha=0.7)
        self.circle2 = Circle(self.point2, 0, fill=False, color='b', linestyle='--', alpha=0.7)
        self.circle3 = Circle(self.point3, 0, fill=False, color='g', linestyle='--', alpha=0.7)
        self.ax.add_patch(self.circle1)
        self.ax.add_patch(self.circle2)
        self.ax.add_patch(self.circle3)

        # Отображение неизвестной точки
        self.unknown_point, = self.ax.plot([0], [0], 'ko', markersize=10, alpha=0.0)

        # Настройка графика
        self.ax.set_aspect('equal')
        self.ax.set_xlim([-50, 300])
        self.ax.set_ylim([-50, 300])
        self.ax.set_xlabel('X (мм)')
        self.ax.set_ylabel('Y (мм)')
        self.ax.grid(True, alpha=0.3)

        # Создание GUI
        self.root = tk.Tk()
        self.root.geometry("800x600")
        self.root.title("Обработка данных учебного макета")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Получение доступных портов (защищенный импорт)
        try:
            from serial.tools import list_ports
            ports = list_ports.comports()
        except Exception:
            ports = []
        self.available_ports = [port.device for port in ports]
        print("Доступные порты:", self.available_ports)

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
                              from_=0.5, to=1.5, resolution=0.01)
        self.slider.config(width=20, length=160)
        self.slider.place(x=611, y=170)

        # Координаты
        self.coordinate_label = tk.Label(self.root, text="Координата точки:")
        self.coordinate_label.place(x=612, y=220)

        self.coordinate_value = tk.StringVar()
        self.coordinate_value.set("(0.00 мм, 0.00 мм)")
        self.coordinate_text = tk.Label(self.root, textvariable=self.coordinate_value, font=("Arial", 12))
        self.coordinate_text.place(x=612, y=240)

        # Статус
        self.status_var = tk.StringVar()
        self.status_var.set("Ожидание подключения")
        self.status_label = tk.Label(self.root, textvariable=self.status_var, fg="blue")
        self.status_label.place(x=612, y=280)

        # Обозначения под статусом
        self.legend_speaker1 = tk.Label(self.root, text="■ Динамик 1", fg="red", font=("Arial", 12))
        self.legend_speaker1.place(x=612, y=310)

        self.legend_speaker2 = tk.Label(self.root, text="■ Динамик 2", fg="blue", font=("Arial", 12))
        self.legend_speaker2.place(x=612, y=330)

        self.legend_speaker3 = tk.Label(self.root, text="■ Динамик 3", fg="green", font=("Arial", 12))
        self.legend_speaker3.place(x=612, y=350)

        self.legend_microphone = tk.Label(self.root, text="● Микрофон", fg="black", font=("Arial", 12))
        self.legend_microphone.place(x=612, y=370)

        # Выбор лабиринта и кнопка старта игры
        self.maze_label = tk.Label(self.root, text="Лабиринт:")
        self.maze_label.place(x=612, y=400)

        self.maze_var = tk.StringVar(self.root)
        self.maze_var.set("")
        self.maze_menu = tk.OptionMenu(self.root, self.maze_var, "")
        self.maze_menu.config(width=20)
        self.maze_menu.place(x=612, y=420)

        self.load_maze_button = tk.Button(self.root, text="Загрузить лабиринт", command=self.on_load_maze_button)
        self.load_maze_button.config(width=20, height=1)
        self.load_maze_button.place(x=612, y=450)

        self.start_game_button = tk.Button(self.root, text="Запустить игру", command=self.on_start_game_button)
        self.start_game_button.config(width=20, height=2)
        self.start_game_button.place(x=612, y=480)

        # График
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().place(x=0, y=0)

    def start_program(self):
        selected_port = self.port_var.get()
        if not selected_port:
            messagebox.showwarning("COM порт", "Выберите COM-порт из списка.")
            return

        if selected_port:
            try:
                self.ser = serial.Serial(
                    port=selected_port,
                    baudrate=115000,
                    timeout=0.1,
                    write_timeout=1,
                    inter_byte_timeout=0.1
                )
                self.ser.reset_input_buffer()
                
                self.start_button.config(state=tk.DISABLED)
                self.pause_button.config(state=tk.NORMAL)
                self.stop_button.config(state=tk.NORMAL)
                self.running = True
                self.status_var.set("Подключено")
                
                self.reading_thread = threading.Thread(target=self.read_serial_data, daemon=True)
                self.reading_thread.start()
                
                self.update_plot()
                
            except serial.SerialException as e:
                error_msg = f"Ошибка подключения к {selected_port}:\n{str(e)}"
                print(error_msg)
                self.status_var.set("Ошибка подключения")
                messagebox.showerror("Ошибка подключения", error_msg + "\nПопробуйте выбрать другой COM-порт.")

    def read_serial_data(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf8', errors='ignore').strip()
                    if data:
                        self.data_queue.put(data)
            except Exception as e:
                print(f"Ошибка чтения из порта: {e}")
                break
            threading.Event().wait(0.01)

    def update_plot(self):
        if self.running:
            while not self.data_queue.empty():
                try:
                    data = self.data_queue.get_nowait()
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
                                
                                # Обновление точки микрофона
                                self.unknown_point.set_data([result[0]], [result[1]])
                                self.unknown_point.set_alpha(1.0)
                                self.unknown_point.set_markersize(10)
                                
                                self.coordinate_value.set(f"({result[0]:.2f} мм, {result[1]:.2f} мм)")
                                self.canvas.draw_idle()
                                
                        except ValueError as e:
                            print(f"Ошибка преобразования: {e}")
                            
                except queue.Empty:
                    break
        
        if self.running:
            self.root.after(100, self.update_plot)

    def pause_program(self):
        self.running = not self.running
        if self.running:
            self.pause_button.config(text="Пауза")
            self.status_var.set("Работает")
            self.update_plot()
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

    def on_closing(self):
        self.stop_program()
        self.root.destroy()

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
            return 0, 0

    def slider_changed(self, value):
        pass

    def update_maze_menu(self):
        """Обновляет опции в выпадающем списке лабиринтов"""
        if not hasattr(self, 'game') or not self.game:
            return
        names = self.game.get_maze_names()
        menu = self.maze_menu['menu']
        menu.delete(0, 'end')
        for name in names:
            menu.add_command(label=name, command=lambda n=name: self.maze_var.set(n))
        if names:
            self.maze_var.set(names[0])

    def on_start_game_button(self):
        """Обработчик кнопки запуска игры"""
        if not hasattr(self, 'game') or not self.game:
            messagebox.showwarning("Игра", "Инициализируйте игру сначала.")
            return
        maze_name = self.maze_var.get()
        if not maze_name:
            messagebox.showwarning("Игра", "Выберите лабиринт.")
            return
        ok = self.game.select_maze(maze_name)
        if not ok:
            messagebox.showerror("Игра", f"Не удалось выбрать лабиринт: {maze_name}")
            return
        started = self.game.start_game()
        if started:
            self.status_var.set(f"Игра: {maze_name}")

    def on_load_maze_button(self):
        """Обработчик кнопки загрузки лабиринта из файла"""
        if not hasattr(self, 'game') or not self.game:
            messagebox.showwarning("Загрузка лабиринта", "Инициализируйте игру сначала.")
            return
        
        # Открываем диалог выбора файла
        file_path = filedialog.askopenfilename(
            title="Выберите файл лабиринта",
            filetypes=[("JSON файлы", "*.json"), ("Все файлы", "*.*")],
            initialdir="."
        )
        
        if not file_path:
            return  # Пользователь отменил выбор
        
        # Загружаем лабиринт
        maze = MazeLoader.load_maze(file_path)
        if maze:
            # Регистрируем лабиринт в игре
            self.game.register_maze(maze)
            # Обновляем меню лабиринтов
            self.update_maze_menu()
            # Выбираем загруженный лабиринт
            self.maze_var.set(maze.name)
            messagebox.showinfo("Загрузка лабиринта", f"Лабиринт '{maze.name}' успешно загружен!")
        else:
            messagebox.showerror("Ошибка загрузки", f"Не удалось загрузить лабиринт из файла:\n{file_path}")

    def start(self):
        self.root.mainloop()

    

    def on_game_win(self):
        """Обработчик победы игры"""
        try:
            self.status_var.set("Победа!")
            messagebox.showinfo("Игра", "Вы выиграли!")
        except Exception:
            pass
        try:
            if hasattr(self, 'game'):
                self.game.reset_game()
        except Exception:
            pass

    def on_game_lose(self):
        """Обработчик поражения в игре"""
        try:
            self.status_var.set("Поражение")
            messagebox.showwarning("Игра", "Вы столкнулись со стеной!")
        except Exception:
            pass
        try:
            if hasattr(self, 'game'):
                self.game.reset_game()
        except Exception:
            pass

    # Инициализация игры и загрузка лабиринтов в конструкторе
    def initialize_game(self):
        self.game = MazeGame(
            self.ax, 
            on_win_callback=self.on_game_win,
            on_lose_callback=self.on_game_lose
        )
        loader = MazeLoader()
        custom_mazes = loader.load_from_folder('my_mazes')
        for name, maze in custom_mazes.items():
            self.game.register_maze(maze)
        # Обновляем меню лабиринтов в интерфейсе
        try:
            self.update_maze_menu()
        except Exception:
            pass

    # Переопределяем update_plot для интеграции с игрой
    def update_plot(self):
        if self.running:
            while not self.data_queue.empty():
                try:
                    data = self.data_queue.get_nowait()
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
                                
                                self.circle1.set_radius(self.distances[0])
                                self.circle2.set_radius(self.distances[1])
                                self.circle3.set_radius(self.distances[2])
                                
                                self.unknown_point.set_data([result[0]], [result[1]])
                                self.unknown_point.set_alpha(1.0)
                                self.unknown_point.set_markersize(10)
                                
                                self.coordinate_value.set(f"({result[0]:.2f} мм, {result[1]:.2f} мм)")
                                self.canvas.draw_idle()

                                # Интеграция с игрой
                                if hasattr(self, 'game') and self.game.is_game_active():
                                    self.game.update_player(result)
                                
                        except ValueError as e:
                            print(f"Ошибка преобразования: {e}")
                            
                except queue.Empty:
                    break
        
        if self.running:
            self.root.after(100, self.update_plot)

# Запуск приложения
app = TrilaterationApp()
app.initialize_game()
app.start()