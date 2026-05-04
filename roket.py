import numpy as np
from cfg import RocketCFG, Simcfg


class RocketPhysics:
    def __init__(self):
        self.current_mass = RocketCFG.mass_full
        self.fuel_burned = 0.0
        self.prev_thrust = 0.0

    def calculate_thrust(self, t: float, height: float, velocity: float) -> np.ndarray:
        g = abs(Simcfg.gravity[2])
        m = self.current_mass

        # Плавное изменение целевой скорости в зависимости от высоты
        if height > RocketCFG.height_very_high:
            v_target = RocketCFG.target_speed_very_high
        elif height > RocketCFG.height_high:
            v_target = RocketCFG.target_speed_high
        elif height > RocketCFG.height_medium:
            v_target = RocketCFG.target_speed_medium
        elif height > RocketCFG.height_low_medium:
            v_target = RocketCFG.target_speed_low_medium
        elif height > RocketCFG.height_low:
            v_target = RocketCFG.target_speed_low
        elif height > RocketCFG.height_very_low:
            v_target = RocketCFG.target_speed_very_low
        elif height > RocketCFG.height_critical:
            v_target = RocketCFG.target_speed_critical
        else:
            v_target = RocketCFG.target_speed_landing

        # PD-регулятор (плавный контроль)
        error_v = v_target - velocity
        a_req = RocketCFG.Kp * error_v - RocketCFG.Kd * velocity + g

        # Ограничение ускорения (избегаем рывков)
        a_req = np.clip(a_req, -15.0, 15.0)

        thrust = m * (g + a_req)

        # Учёт остатка топлива
        fuel_left = RocketCFG.mass_fuel - self.fuel_burned
        if fuel_left <= 0:
            thrust = 0.0
        else:
            fuel_ratio = max(0, min(1, fuel_left / RocketCFG.mass_fuel))
            thrust *= fuel_ratio

        # Финальный дожим перед землёй (гарантирует мягкую посадку)
        if height < 5 and velocity < 0:
            required_decel = (velocity ** 2) / (2 * max(height, 0.2))
            landing_thrust = m * (g + required_decel)
            thrust = max(thrust, landing_thrust)

        # Ограничения тяги
        thrust = max(0, min(thrust, RocketCFG.max_thrust))

        # ЗАПРЕТ НА ПОЛЁТ ВВЕРХ - ракета только снижается
        if velocity > -0.3 and height < 15:
            # Удерживаем тягу чуть меньше силы тяжести
            thrust = min(thrust, m * g * 0.92)

        # Расход топлива (пропорционален тяге)
        throttle = thrust / RocketCFG.max_thrust if RocketCFG.max_thrust > 0 else 0
        fuel_used = RocketCFG.fuel_consumption_rate * throttle * Simcfg.step
        self.fuel_burned += fuel_used

        self.current_mass = max(
            RocketCFG.mass_empty,
            RocketCFG.mass_full - self.fuel_burned
        )

        # Сглаживание тяги (убирает резкие перепады)
        alpha = 0.45 if height > RocketCFG.height_low else 0.7
        thrust = alpha * thrust + (1 - alpha) * self.prev_thrust
        self.prev_thrust = thrust

        return np.array([0, 0, thrust])

    def calculate_drag(self, velocity: np.ndarray, height: float) -> np.ndarray:
        v = velocity
        speed = np.linalg.norm(v)

        if speed < 0.01:
            return np.zeros(3)

        # Плотность воздуха зависит от высоты
        rho = RocketCFG.air_density * np.exp(-height / 8000.0)
        drag_direction = -v / speed
        drag_magnitude = 0.5 * rho * RocketCFG.drag_coefficient * RocketCFG.effective_area * speed ** 2
        drag = drag_direction * drag_magnitude

        return drag

    def calculate_gravity(self) -> np.ndarray:
        return np.array([0, 0, self.current_mass * Simcfg.gravity[2]])

    def total_force(self, t: float, velocity: np.ndarray, height: float) -> np.ndarray:
        thrust = self.calculate_thrust(t, height, velocity[2])
        drag = self.calculate_drag(velocity, height)
        gravity = self.calculate_gravity()
        return thrust + drag + gravity

    def get_current_mass(self) -> float:
        return self.current_mass

    def reset_state(self):
        self.current_mass = RocketCFG.mass_full
        self.fuel_burned = 0.0
        self.prev_thrust = 0.0
