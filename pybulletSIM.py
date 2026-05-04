# pybulletSIM.py - альтернативная симуляция (не используется в main.py)
# Этот файл оставлен для совместимости, основная симуляция в main.py

import pybullet as p
import pybullet_data
import numpy as np
from load_rocket import RocketLoader
from cfg import Simcfg, RocketCFG
import roket


class PyBulletSimulation:
    """Альтернативная симуляция с использованием PyBullet физики"""

    def __init__(self, gui=True):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        self.rocket_id = None
        self.time = 0
        self.history = {'time': [], 'height': [], 'velocity': [], 'thrust': []}

        self.physics = roket.RocketPhysics()
        self.physics.reset_state()

    def setup_environment(self, urdf_path="rocket.urdf"):
        """Настройка среды с вашей 3D моделью"""
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, 0)  # Гравитация из roket.py
        p.loadURDF("plane.urdf", useFixedBase=True)

        # Настройка камеры
        p.resetDebugVisualizerCamera(
            cameraDistance=50,
            cameraYaw=25,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, RocketCFG.start_height]
        )

        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

        # Маркеры высот
        for h in [300, 250, 200, 150, 100, 50, 30, 15, 8, 3]:
            p.addUserDebugText(f"{h}м", [0, 0, h], [0.6, 0.6, 0.6], 0.7)

        # Посадочная площадка
        pad_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[6, 6, 0.05],
                                         rgbaColor=[0.55, 0.55, 0.55, 1.0])
        pad_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=pad_visual,
                                     basePosition=[0, 0, 0.01])

        # Красный круг
        for r in [3.5, 2.5, 1.5, 0.8]:
            circle_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=0.02,
                                                rgbaColor=[0.9, 0.2, 0.1, 1.0])
            circle_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=circle_visual,
                                            basePosition=[0, 0, 0.02])

        p.addUserDebugText("🎯 ЦЕНТР ПОСАДКИ", [0, 0, 0.5], [1, 1, 0], 1.2)

        # Загрузка ракеты
        loader = RocketLoader(self.client)
        try:
            self.rocket_id = loader.load(urdf_path)
            print(f"✅ Загружена 3D модель из {urdf_path}")
        except:
            print(f"❌ Не удалось загрузить {urdf_path}")
            return

        if self.rocket_id is None:
            raise RuntimeError("Не удалось загрузить ракету")

        # Начальная позиция и скорость
        p.resetBasePositionAndOrientation(self.rocket_id, [0, 0, RocketCFG.start_height], [0, 0, 0, 1])
        p.resetBaseVelocity(self.rocket_id, [0, 0, RocketCFG.start_velocity])

        print(f"\n✅ АЛЬТЕРНАТИВНАЯ СИМУЛЯЦИЯ ГОТОВА")
        print(f"   Высота: {RocketCFG.start_height}м, Скорость: {RocketCFG.start_velocity}м/с")

    def step(self):
        """Один шаг симуляции"""
        if self.rocket_id is None:
            return

        pos, _ = p.getBasePositionAndOrientation(self.rocket_id)
        vel, _ = p.getBaseVelocity(self.rocket_id)

        height = pos[2]
        velocity = vel[2]

        # Расчёт сил через roket.py
        force = self.physics.total_force(self.time, np.array([0, 0, velocity]), height)

        # Применение силы
        p.applyExternalForce(self.rocket_id, -1, force, [0, 0, 0], p.LINK_FRAME)

        # История
        self.history['time'].append(self.time)
        self.history['height'].append(height)
        self.history['velocity'].append(velocity)
        self.history['thrust'].append(force[2])

        p.stepSimulation()
        self.time += Simcfg.step

    def run_simulation(self, duration=None):
        """Запуск симуляции"""
        if duration is None:
            duration = Simcfg.duration

        steps = int(duration / Simcfg.step)

        print("\n" + "=" * 60)
        print("🚀 АЛЬТЕРНАТИВНАЯ СИМУЛЯЦИЯ ПОСАДКИ")
        print("=" * 60)
        print(f"{'Время':6} | {'Высота':8} | {'Скорость':9} | {'Тяга':8} | {'Топливо':6}")
        print("-" * 55)

        for i in range(steps):
            self.step()

            if i % 120 == 0 and len(self.history['time']) > 0:
                fuel_left = RocketCFG.mass_fuel - self.physics.fuel_burned
                print(f"{self.time:5.1f} | {self.history['height'][-1]:8.1f} | "
                      f"{self.history['velocity'][-1]:8.2f} | {self.history['thrust'][-1]:7.0f} | {max(0, fuel_left):6.1f}")

            if len(self.history['height']) > 0 and self.history['height'][-1] <= 0.3:
                speed = abs(self.history['velocity'][-1])
                print("-" * 55)
                print(f"\n🪂 ПОСАДКА!")
                print(f"   Скорость касания: {speed:.2f} м/с")
                if speed < 2:
                    print("   ✅ ИДЕАЛЬНАЯ МЯГКАЯ ПОСАДКА!")
                elif speed < 5:
                    print("   ✅ ХОРОШАЯ ПОСАДКА")
                else:
                    print("   ⚠️ ЖЕСТКАЯ ПОСАДКА")
                break

        return self.history

    def dispose(self):
        """Отключение"""
        try:
            p.disconnect()
        except:
            pass


# Функции для совместимости
_physics_instance = None


def get_physics():
    global _physics_instance
    if _physics_instance is None:
        _physics_instance = roket.RocketPhysics()
    return _physics_instance


def reset_state():
    get_physics().reset_state()


def get_current_mass():
    return get_physics().get_current_mass()


def get_fuel():
    return RocketCFG.mass_fuel - get_physics().fuel_burned


def total_force(t, vel, mass, height):
    return get_physics().total_force(t, vel, height)
