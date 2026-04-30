import numpy as np
from cfg import RocketCFG, Simcfg


class RocketPhysics:
    def __init__(self):
        self.current_mass = RocketCFG.mass_full
        self.fuel_burned = 0.0
        self.engine_on = True
        self.initial_mass = RocketCFG.mass_full
        self.max_thrust = RocketCFG.max_thrust

    def calculate_thrust(self, t: float, height: float, velocity: float) -> np.ndarray:
        """Расчет тяги с учетом этапа посадки"""

        # Если топливо кончилось
        if self.fuel_burned >= RocketCFG.mass_fuel:
            self.engine_on = False
            return np.array([0, 0, 0])

        # Расчет силы тяги на основе высоты
        thrust_force = 0

        if height > RocketCFG.braking_start_height:
            # Свободное падение - двигатель выключен
            thrust_force = 0
        elif height > RocketCFG.precision_start_height:
            # Основное торможение - максимальная тяга
            thrust_force = RocketCFG.max_thrust
        elif height > RocketCFG.landing_height:
            # Точное торможение - уменьшаем тягу пропорционально высоте
            ratio = (height - RocketCFG.landing_height) / (RocketCFG.precision_start_height - RocketCFG.landing_height)
            thrust_force = RocketCFG.min_thrust + (RocketCFG.max_thrust - RocketCFG.min_thrust) * ratio
        else:
            # Финальная посадка - поддерживаем мягкое снижение
            velocity_error = velocity - RocketCFG.target_landing_velocity
            thrust_force = max(RocketCFG.min_thrust * 0.5,
                               (self.current_mass * abs(Simcfg.gravity[2]) +
                                abs(velocity_error) * 100))
            thrust_force = min(thrust_force, RocketCFG.max_thrust * 0.3)

        # Если двигатель выключен по времени
        if t >= RocketCFG.burn_time:
            thrust_force = 0
            self.engine_on = False

        # Обновление массы (расход топлива)
        if thrust_force > 0 and self.engine_on:
            fuel_used = RocketCFG.fuel_consumption_rate * Simcfg.step
            self.fuel_burned += fuel_used
            self.current_mass = max(RocketCFG.mass_empty,
                                    RocketCFG.mass_full - self.fuel_burned)

        return np.array([0, 0, thrust_force])

    def calculate_drag(self, velocity: np.ndarray, height: float) -> np.ndarray:
        """Расчет сопротивления воздуха с учетом плотности по высоте"""
        velocity_magnitude = np.linalg.norm(velocity)

        if velocity_magnitude < 0.1:
            return np.array([0, 0, 0])

        # Простая модель плотности воздуха (экспоненциальное уменьшение с высотой)
        density = RocketCFG.air_density * np.exp(-height / 8000.0)

        # Сила сопротивления: F = 0.5 * ρ * Cd * A * v²
        drag_magnitude = 0.5 * density * RocketCFG.drag_coefficient * RocketCFG.effective_area * velocity_magnitude ** 2

        # Направление против скорости
        drag_force = -drag_magnitude * (velocity / velocity_magnitude)

        return drag_force

    def calculate_gravity(self) -> np.ndarray:
        """Расчет силы тяжести"""
        return np.array([0, 0, self.current_mass * Simcfg.gravity[2]])

    def total_force(self, t: float, position: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """Суммарная сила, действующая на ракету"""
        height = position[2]

        thrust = self.calculate_thrust(t, height, velocity[2])
        drag = self.calculate_drag(velocity, height)
        gravity = self.calculate_gravity()

        return thrust + drag + gravity

    def get_current_mass(self) -> float:
        return self.current_mass

    def get_fuel(self) -> float:
        """Возвращает оставшееся топливо"""
        return RocketCFG.mass_fuel - self.fuel_burned


# Функции для совместимости с pybulletSIM.py
_rocket_physics = None


def get_physics():
    global _rocket_physics
    if _rocket_physics is None:
        _rocket_physics = RocketPhysics()
    return _rocket_physics


def reset_state():
    global _rocket_physics
    _rocket_physics = RocketPhysics()


def get_current_mass():
    return get_physics().get_current_mass()


def get_fuel():
    return get_physics().get_fuel()


def total_force(t, vel, mass, height):
    """Совместимая функция для pybulletSIM.py"""
    physics = get_physics()
    physics.current_mass = mass
    position = np.array([0, 0, height])
    velocity = np.array([0, 0, vel])
    force = physics.total_force(t, position, velocity)
    return force
