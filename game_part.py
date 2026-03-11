print('hello')
import numpy as np
from matplotlib.patches import Circle
import json
import os
from abc import ABC, abstractmethod
# ====================== БАЗОВЫЙ КЛАСС ЛАБИРИНТА ======================

class BaseMaze(ABC):
    @property
    @abstractmethod
    def name(self):
        return "Базовый лабиринт"
    
    @property
    @abstractmethod
    def walls(self):
        return []
    
    @property
    @abstractmethod
    def start_pos(self):
        return (30, 30)
    
    @property
    @abstractmethod
    def finish_pos(self):
        return (220, 220)
    
    @property
    def start_radius(self):
        return 12
    
    @property
    def finish_radius(self):
        return 12
    
    @property
    def wall_color(self):
        return 'black'
    
    @property
    def wall_thickness(self):
        return 3
    
    def validate(self):
        """Проверка корректности лабиринта"""
        # Проверяем, что старт и финиш не на стенах
        for wall in self.walls:
            if self._point_on_segment(self.start_pos, wall[0], wall[1]):
                print(f"Предупреждение: старт {self.start_pos} близко к стене {wall}")
            if self._point_on_segment(self.finish_pos, wall[0], wall[1]):
                print(f"Предупреждение: финиш {self.finish_pos} близко к стене {wall}")
        return True
    
    def _point_on_segment(self, p, a, b, tolerance=5):
        """Проверка, лежит ли точка на отрезке (с допуском)"""
        p = np.array(p, dtype=float)
        a = np.array(a, dtype=float)
        b = np.array(b, dtype=float)
        
        # Векторные вычисления
        ab = b - a
        ap = p - a
        
        # Проекция точки на прямую
        t = np.dot(ap, ab) / np.dot(ab, ab)
        
        # Если проекция вне отрезка - точка не на отрезке
        if t < 0 or t > 1:
            return False
        
        # Ближайшая точка на отрезке
        projection = a + t * ab
        
        # Расстояние от точки до проекции
        distance = np.linalg.norm(p - projection)
        
        return distance < tolerance

# ====================== КОНКРЕТНЫЕ ЛАБИРИНТЫ ======================

class SimpleMaze(BaseMaze):
    @property
    def name(self):
        return "Простой лабиринт"
    
    @property
    def walls(self):
        return [
            # Внешние стены
            [[10, 10], [240, 10]],    # Северная
            [[10, 240], [240, 240]],  # Южная
            [[10, 10], [10, 240]],    # Западная
            [[240, 10], [240, 240]],  # Восточная
            
            # Внутренние стены - простой коридор
            [[80, 10], [80, 180]],
            [[160, 70], [160, 240]],
            [[120, 120], [200, 120]],
        ]
    
    @property
    def start_pos(self):
        return (30, 30)
    
    @property
    def finish_pos(self):
        return (210, 210)

class HardMaze(BaseMaze):
    """Сложный лабиринт с большим количеством препятствий"""
    
    @property
    def name(self):
        return "Сложный лабиринт"
    
    @property
    def walls(self):
        return [
            # Внешние стены
            [[10, 10], [240, 10]],
            [[10, 240], [240, 240]],
            [[10, 10], [10, 240]],
            [[240, 10], [240, 240]],
            
            # Лабиринт внутри
            [[40, 10], [40, 150]],
            [[100, 90], [100, 200]],
            [[160, 40], [160, 160]],
            [[200, 120], [200, 240]],
            
            # Горизонтальные перегородки
            [[70, 70], [180, 70]],
            [[30, 150], [130, 150]],
            [[150, 200], [220, 200]],
        ]
    
    @property
    def start_pos(self):
        return (25, 25)
    
    @property
    def finish_pos(self):
        return (225, 225)
    
    @property
    def wall_color(self):
        return 'darkred'

class MazeWithIsland(BaseMaze):
    """Лабиринт с островком посередине"""
    
    @property
    def name(self):
        return "Лабиринт с островом"
    
    @property
    def walls(self):
        return [
            # Внешние стены
            [[0, 0], [250, 0]],
            [[0, 250], [250, 250]],
            [[0, 0], [0, 250]],
            [[250, 0], [250, 250]],
            
            # Остров в центре
            [[100, 100], [150, 100]],
            [[150, 100], [150, 150]],
            [[150, 150], [100, 150]],
            [[100, 150], [100, 100]],
            
            # Проходы
            [[50, 50], [200, 50]],
            [[50, 200], [200, 200]],
        ]
    
    @property
    def start_pos(self):
        return (30, 220)
    
    @property
    def finish_pos(self):
        return (220, 30)

# ====================== КЛАСС ИГРЫ ======================

class MazeGame:
    """
    Основной класс для управления игрой в лабиринт
    Отвечает за:
    - Загрузку и переключение лабиринтов
    - Отрисовку лабиринта на графике
    - Детекцию столкновений со стенами
    - Проверку победы/поражения
    """
    
    def __init__(self, ax, on_win_callback=None, on_lose_callback=None):
        """
        Параметры:
        - ax: объект осей matplotlib для отрисовки
        - on_win_callback: функция, вызываемая при победе
        - on_lose_callback: функция, вызываемая при поражении
        """
        self.ax = ax
        self.on_win = on_win_callback
        self.on_lose = on_lose_callback
        
        # Доступные лабиринты
        self.available_mazes = {}
        self.current_maze = None
        self.current_maze_name = None
        
        # Состояние игры
        self.game_active = False
        self.player_pos = None
        self.player_radius = 6  # Радиус игрока для детекции столкновений (мм)
        
        # Графические элементы
        self.player_dot = None
        self.maze_elements = []  # Стены и зоны для последующего удаления
        
        # Загружаем встроенные лабиринты
        self._register_builtin_mazes()
    
    def _register_builtin_mazes(self):
        """Регистрирует встроенные лабиринты"""
        self.register_maze(SimpleMaze())
        self.register_maze(HardMaze())
        self.register_maze(MazeWithIsland())
    
    def register_maze(self, maze_instance):
        """Добавляет лабиринт в список доступных"""
        self.available_mazes[maze_instance.name] = maze_instance
    
    def get_maze_names(self):
        """Возвращает список названий доступных лабиринтов"""
        return list(self.available_mazes.keys())
    
    def select_maze(self, maze_name):
        """Выбирает лабиринт по имени для использования"""
        if maze_name in self.available_mazes:
            self.current_maze = self.available_mazes[maze_name]
            self.current_maze.validate()
            self.current_maze_name = maze_name
            print(f"Выбран лабиринт: {maze_name}")
            return True
        print(f"Лабиринт '{maze_name}' не найден")
        return False
    
    def start_game(self):
        """Начинает новую игру с выбранным лабиринтом"""
        if not self.current_maze:
            print("Сначала выберите лабиринт!")
            return False
        
        # Сбрасываем состояние
        self.game_active = True
        self.player_pos = self.current_maze.start_pos
        
        # Очищаем предыдущий лабиринт с графика
        self._clear_maze_from_axes()
        
        # Рисуем новый лабиринт
        self._draw_maze()
        self._draw_start_finish()
        
        # Создаем точку игрока
        if self.player_dot:
            self.player_dot.remove()
        
        self.player_dot, = self.ax.plot(
            [self.player_pos[0]], [self.player_pos[1]],
            'yo', markersize=12, markeredgecolor='black',
            zorder=10, label='Микрофон'
        )
        
        # Обновляем график
        self.ax.figure.canvas.draw_idle()
        print(f"Игра начата! Старт: {self.player_pos}")
        return True
    
    def _clear_maze_from_axes(self):
        """Удаляет все элементы лабиринта с графика"""
        for element in self.maze_elements:
            try:
                element.remove()
            except:
                pass
        self.maze_elements.clear()
    
    def _draw_maze(self):
        """Отрисовывает стены лабиринта"""
        maze = self.current_maze
        
        for wall in maze.walls:
            x_coords = [wall[0][0], wall[1][0]]
            y_coords = [wall[0][1], wall[1][1]]
            
            line = self.ax.plot(
                x_coords, y_coords,
                color=maze.wall_color,
                linewidth=maze.wall_thickness,
                solid_capstyle='round',
                zorder=2
            )[0]
            self.maze_elements.append(line)
    
    def _draw_start_finish(self):
        """Отрисовывает зоны старта и финиша"""
        maze = self.current_maze
        
        # Старт - зеленая полупрозрачная зона
        start_circle = Circle(
            maze.start_pos, maze.start_radius,
            color='lightgreen', alpha=0.4,
            zorder=1, label='Старт'
        )
        self.ax.add_patch(start_circle)
        self.maze_elements.append(start_circle)
        
        # Финиш - красная полупрозрачная зона
        finish_circle = Circle(
            maze.finish_pos, maze.finish_radius,
            color='salmon', alpha=0.4,
            zorder=1, label='Финиш'
        )
        self.ax.add_patch(finish_circle)
        self.maze_elements.append(finish_circle)
    
    def update_player(self, new_pos):
        """
        Обновляет позицию игрока и проверяет состояние игры
        
        Параметры:
        - new_pos: tuple (x, y) новые координаты микрофона
        
        Возвращает:
        - 'ongoing' - игра продолжается
        - 'win' - победа
        - 'lose' - поражение
        - 'inactive' - игра не активна
        """
        if not self.game_active or not self.current_maze:
            return 'inactive'
        
        # Проверка на корректность координат
        if not isinstance(new_pos, (tuple, list)) or len(new_pos) != 2:
            return 'ongoing'
        
        # Проверка столкновения со стенами
        if self._check_collision(new_pos):
            self.game_active = False
            if self.on_lose:
                self.on_lose()
            return 'lose'
        
        # Обновляем позицию игрока на графике
        self.player_pos = new_pos
        if self.player_dot:
            self.player_dot.set_data([new_pos[0]], [new_pos[1]])
        
        # Проверка достижения финиша
        if self._check_win(new_pos):
            self.game_active = False
            if self.on_win:
                self.on_win()
            return 'win'
        
        # Обновляем график (не слишком часто)
        if np.random.random() < 0.1:  # Примерно каждый 10-й раз
            self.ax.figure.canvas.draw_idle()
        
        return 'ongoing'
    
    def _check_collision(self, point):
        """
        Проверяет столкновение точки со стенами
        Возвращает True, если расстояние до любой стены < радиуса игрока
        """
        for wall in self.current_maze.walls:
            distance = self._distance_point_to_segment(
                point, wall[0], wall[1]
            )
            if distance < self.player_radius:
                return True
        return False
    
    def _distance_point_to_segment(self, p, a, b):
        """
        Вычисляет минимальное расстояние от точки p до отрезка ab
        """
        p = np.array(p, dtype=float)
        a = np.array(a, dtype=float)
        b = np.array(b, dtype=float)
        
        # Вектор отрезка
        ab = b - a
        
        # Если отрезок вырожден в точку
        if np.linalg.norm(ab) < 1e-6:
            return np.linalg.norm(p - a)
        
        # Проекция точки на прямую
        ap = p - a
        t = np.dot(ap, ab) / np.dot(ab, ab)
        
        # Ближайшая точка на отрезке
        if t < 0:
            closest = a
        elif t > 1:
            closest = b
        else:
            closest = a + t * ab
        
        return np.linalg.norm(p - closest)
    
    def _check_win(self, point):
        """Проверяет, достиг ли игрок финиша"""
        distance = np.linalg.norm(
            np.array(point) - np.array(self.current_maze.finish_pos)
        )
        return distance < self.current_maze.finish_radius
    
    def get_player_pos(self):
        """Возвращает текущую позицию игрока"""
        return self.player_pos
    
    def is_game_active(self):
        """Возвращает, активна ли игра"""
        return self.game_active
    
    def reset_game(self):
        """Сбрасывает игру в начальное состояние"""
        if self.current_maze:
            self.game_active = False
            self.player_pos = self.current_maze.start_pos
            if self.player_dot:
                self.player_dot.set_data([self.player_pos[0]], [self.player_pos[1]])
                self.ax.figure.canvas.draw_idle()
            print("Игра сброшена")

# ====================== ЗАГРУЗЧИК ЛАБИРИНТОВ ИЗ ФАЙЛОВ ======================

class MazeLoader:
    """
    Класс для загрузки лабиринтов из JSON-файлов
    Позволяет сохранять и загружать пользовательские лабиринты
    """
    
    @staticmethod
    def save_maze(maze, filename):
        """
        Сохраняет лабиринт в JSON файл
        
        Параметры:
        - maze: экземпляр BaseMaze
        - filename: имя файла для сохранения
        """
        maze_data = {
            'name': maze.name,
            'walls': [[list(wall[0]), list(wall[1])] for wall in maze.walls],
            'start_pos': list(maze.start_pos),
            'finish_pos': list(maze.finish_pos),
            'start_radius': maze.start_radius,
            'finish_radius': maze.finish_radius,
            'wall_color': maze.wall_color,
            'wall_thickness': maze.wall_thickness
        }
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(maze_data, f, indent=2, ensure_ascii=False)
        
        print(f"Лабиринт сохранен в {filename}")
    
    @staticmethod
    def load_maze(filename):
        """
        Загружает лабиринт из JSON файла
        
        Параметры:
        - filename: имя файла для загрузки
        
        Возвращает:
        - экземпляр DynamicMaze или None при ошибке
        """
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                maze_data = json.load(f)
            
            # Создаем динамический класс лабиринта
            class DynamicMaze(BaseMaze):
                @property
                def name(self):
                    return maze_data.get('name', 'Загруженный лабиринт')
                
                @property
                def walls(self):
                    return maze_data.get('walls', [])
                
                @property
                def start_pos(self):
                    return tuple(maze_data.get('start_pos', [30, 30]))
                
                @property
                def finish_pos(self):
                    return tuple(maze_data.get('finish_pos', [220, 220]))
                
                @property
                def start_radius(self):
                    return maze_data.get('start_radius', 12)
                
                @property
                def finish_radius(self):
                    return maze_data.get('finish_radius', 12)
                
                @property
                def wall_color(self):
                    return maze_data.get('wall_color', 'black')
                
                @property
                def wall_thickness(self):
                    return maze_data.get('wall_thickness', 3)
            
            maze = DynamicMaze()
            maze.validate()
            print(f"Лабиринт загружен из {filename}")
            return maze
            
        except Exception as e:
            print(f"Ошибка загрузки лабиринта из {filename}: {e}")
            return None
    
    @staticmethod
    def load_from_folder(folder_path='mazes'):
        """
        Загружает все JSON-лабиринты из папки
        
        Параметры:
        - folder_path: путь к папке с лабиринтами
        
        Возвращает:
        - словарь {имя_лабиринта: экземпляр Maze}
        """
        mazes = {}
        
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print(f"Создана папка {folder_path}")
            return mazes
        
        for filename in os.listdir(folder_path):
            if filename.endswith('.json'):
                filepath = os.path.join(folder_path, filename)
                maze = MazeLoader.load_maze(filepath)
                if maze:
                    mazes[maze.name] = maze
        
        return mazes

# ====================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ======================

def create_sample_maze():
    """Создает пример лабиринта для демонстрации"""
    walls = [
        [[20, 20], [230, 20]],
        [[20, 230], [230, 230]],
        [[20, 20], [20, 230]],
        [[230, 20], [230, 230]],
        [[100, 20], [100, 150]],
        [[150, 100], [150, 230]],
    ]
    
    class SampleMaze(BaseMaze):
        @property
        def name(self):
            return "Образец"
        
        @property
        def walls(self):
            return walls
        
        @property
        def start_pos(self):
            return (40, 40)
        
        @property
        def finish_pos(self):
            return (210, 210)
    
    return SampleMaze()

# Для быстрого тестирования модуля
if __name__ == "__main__":
    print("=" * 50)
    print("Модуль game_part.py")
    print("=" * 50)
    
    # Создаем тестовый лабиринт
    test_maze = SimpleMaze()
    print(f"Лабиринт: {test_maze.name}")
    print(f"Стен: {len(test_maze.walls)}")
    print(f"Старт: {test_maze.start_pos}")
    print(f"Финиш: {test_maze.finish_pos}")
    
    # Проверка загрузчика
    loader = MazeLoader()
    print("\nЗагрузчик лабиринтов готов")
    
    print("\nМодуль успешно загружен!")