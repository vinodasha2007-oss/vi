# cfg.py - конфигурация для симуляции посадки ракеты

class RocketCFG:
    # Массовые характеристики
    mass_full = 267.0  # кг (полная масса)
    mass_empty = 249.0  # кг (пустая масса без топлива)
    mass_fuel = 13.0  # кг (масса топлива)

    # Тяговые характеристики
    max_thrust = 10000.0  # Н (максимальная тяга)
    min_thrust = 500.0  # Н (минимальная тяга)

    # Высоты управления (плавное снижение скорости)
    height_very_high = 200.0   # очень высоко
    height_high = 150.0        # высоко
    height_medium = 100.0      # средне
    height_low_medium = 60.0   # ниже среднего
    height_low = 30.0          # низко
    height_very_low = 15.0     # очень низко
    height_critical = 8.0      # критично близко

    # Целевые скорости на разных высотах
    target_speed_very_high = -65.0
    target_speed_high = -50.0
    target_speed_medium = -35.0
    target_speed_low_medium = -20.0
    target_speed_low = -10.0
    target_speed_very_low = -5.0
    target_speed_critical = -3.0
    target_speed_landing = -1.5   # м/с (мягкая посадка)

    # Параметры PD-регулятора
    Kp = 1.2  # пропорциональный коэффициент
    Kd = 0.7  # дифференциальный коэффициент

    # Расход топлива
    fuel_consumption_rate = 0.8  # кг/с

    # Аэродинамические параметры
    drag_coefficient = 0.75
    air_density = 1.225  # кг/м³
    effective_area = 0.05  # м²

    # Начальные условия
    start_height = 300.0  # м
    start_velocity = -80.0  # м/с

    # Геометрия ракеты (из OBJ файла)
    rocket_height = 3.0  # м
    rocket_radius = 0.7  # м


class Simcfg:
    step = 1 / 240.0  # шаг симуляции (сек)
    gravity = [0, 0, -9.81]  # гравитация
    duration = 30.0  # максимальная длительность симуляции (сек)
