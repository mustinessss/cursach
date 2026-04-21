import subprocess
import sys
import os

def check_python_version():
    """Проверяет версию Python (нужна 3.6+)"""
    if sys.version_info < (3, 6):
        return False
    return True

def check_and_install_packages():
    """Проверяет и устанавливает необходимые пакеты"""
    packages = ['pyserial', 'matplotlib', 'numpy']
    missing_packages = []

    for pkg in packages:
        try:
            __import__(pkg.replace('-', '_'))
        except ImportError:
            missing_packages.append(pkg)

    if missing_packages:
        print(f"Устанавливаю пакеты: {', '.join(missing_packages)}")
        try:
            subprocess.check_call([sys.executable, '-m', 'pip', 'install'] + missing_packages)
            print("Пакеты установлены успешно")
        except subprocess.CalledProcessError as e:
            print(f"Ошибка установки пакетов: {e}")
            return False
    else:
        print("Все пакеты уже установлены")
    return True

def create_shortcut():
    """Создает ярлык для запуска main_gui.py"""
    try:
        import winshell
        from win32com.client import Dispatch
    except ImportError:
        print("Устанавливаю pywin32 для создания ярлыка")
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'pywin32'])
        import winshell
        from win32com.client import Dispatch

    desktop = winshell.desktop()
    path = os.path.join(desktop, "Мой Проект.lnk")
    target = os.path.join(os.getcwd(), "dist", "main_gui.exe")

    shell = Dispatch('WScript.Shell')
    shortcut = shell.CreateShortCut(path)
    shortcut.Targetpath = target
    shortcut.WorkingDirectory = os.getcwd()
    shortcut.IconLocation = target
    shortcut.save()

    print(f"Ярлык создан: {path}")

def main():
    print("Проверка системы...")

    if not check_python_version():
        print("Требуется Python 3.6 или выше")
        return

    if not check_and_install_packages():
        return

    create_shortcut()

    print("Установка завершена. Ярлык создан на рабочем столе.")

    # Запускаем приложение
    print("Запуск main_gui.exe...")
    subprocess.call([os.path.join("dist", "main_gui.exe")])

if __name__ == '__main__':
    main()