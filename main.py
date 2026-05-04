import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import matplotlib

# Настройка matplotlib
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'WenQuanYi Zen Hei']
matplotlib.rcParams['axes.unicode_minus'] = False

from cfg import RocketCFG, Simcfg
from roket import RocketPhysics


class RocketLandingSimulation:
    def __init__(self, gui=True):
        self.gui = gui
        self.client = None
        self.rocket_id = None
        self.physics = RocketPhysics()

        self.time_data = []
        self.height_data = []
        self.velocity_data = []
        self.thrust_data = []
        self.mass_data = []

        self.sim_time = 0.0
        self.is_landed = False
        self.landing_velocity = 0.0
        self.landing_complete = False
        self.ground_level = 0.0
        self.simulation_error = False

        self.rocket_height = 3.0
        self.rocket_radius = 0.7
        self.rocket_bottom_offset = 1.5

        self.contact_count = 0
        self.landing_triggered = False
        self.stable_landing_frames = 0

        self.decoration_bodies = []
        self.debug_lines = []

    def setup_pybullet(self):
        try:
            if self.gui:
                self.client = p.connect(p.GUI)
                p.resetDebugVisualizerCamera(
                    cameraDistance=8,
                    cameraYaw=25,
                    cameraPitch=-20,
                    cameraTargetPosition=[0, 0, RocketCFG.start_height - 50],
                    physicsClientId=self.client
                )
                p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            else:
                self.client = p.connect(p.DIRECT)

            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(*Simcfg.gravity)
            p.setTimeStep(Simcfg.step, physicsClientId=self.client)

            self.ground_id = p.loadURDF("plane.urdf", useFixedBase=True)
            p.changeVisualShape(self.ground_id, -1, rgbaColor=[0.4, 0.5, 0.3, 1.0])

            p.setPhysicsEngineParameter(
                numSolverIterations=150,
                useSplitImpulse=True,
                splitImpulsePenetrationThreshold=0.0005,
                contactBreakingThreshold=0.0005,
                restitutionVelocityThreshold=0.05,
                physicsClientId=self.client
            )

            self.create_landing_pad()

            print("PyBullet успешно настроен")

        except Exception as e:
            print(f"Ошибка настройки PyBullet: {e}")
            self.simulation_error = True
            raise

    def create_landing_pad(self):
        try:
            for body in self.decoration_bodies:
                try:
                    p.removeBody(body)
                except:
                    pass
            self.decoration_bodies = []

            for line in self.debug_lines:
                try:
                    p.removeUserDebugItem(line)
                except:
                    pass
            self.debug_lines = []

            pad_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[5, 5, 0.05],
                rgbaColor=[0.6, 0.6, 0.6, 1.0]
            )
            pad_body = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=pad_visual,
                basePosition=[0, 0, 0],
                baseOrientation=[0, 0, 0, 1]
            )
            self.decoration_bodies.append(pad_body)

            circle_visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=3,
                length=0.1,
                rgbaColor=[0.95, 0.1, 0.1, 1.0]
            )
            circle_body = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=circle_visual,
                basePosition=[0, 0, 0.03],
                baseOrientation=[0, 0, 0, 1]
            )
            self.decoration_bodies.append(circle_body)

            print("Посадочная площадка создана")

        except Exception as e:
            print(f"Ошибка создания площадки: {e}")

    def load_rocket(self):
        try:
            urdf_path = "rocket.urdf"
            if not os.path.exists(urdf_path):
                print(f"Создаем простую серую ракету")
                self.create_simple_rocket()
            else:
                start_pos = [0, 0, RocketCFG.start_height]
                start_orient = p.getQuaternionFromEuler([0, 0, 0])

                self.rocket_id = p.loadURDF(
                    urdf_path,
                    start_pos,
                    start_orient,
                    useFixedBase=False,
                    physicsClientId=self.client
                )

                try:
                    p.changeVisualShape(self.rocket_id, -1, rgbaColor=[0.7, 0.7, 0.7, 1.0])
                except:
                    pass

                p.changeDynamics(
                    self.rocket_id, -1,
                    restitution=0.05,
                    lateralFriction=0.5,
                    linearDamping=0.2,
                    angularDamping=0.2,
                    contactStiffness=200000,
                    contactDamping=50000,
                    physicsClientId=self.client
                )

            if self.rocket_id is None or self.rocket_id < 0:
                raise Exception("Не удалось создать ракету!")

            p.resetBaseVelocity(
                self.rocket_id,
                linearVelocity=[0, 0, RocketCFG.start_velocity],
                physicsClientId=self.client
            )

            print(f"Серая ракета успешно создана")

        except Exception as e:
            print(f"Ошибка загрузки ракеты: {e}")
            self.simulation_error = True
            raise

    def create_simple_rocket(self):
        try:
            body_collision = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=self.rocket_radius,
                height=self.rocket_height
            )

            body_visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=self.rocket_radius,
                length=self.rocket_height,
                rgbaColor=[0.7, 0.7, 0.7, 1.0]
            )

            nose_visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.5,
                length=1.0,
                rgbaColor=[0.8, 0.8, 0.8, 1.0]
            )

            self.rocket_id = p.createMultiBody(
                baseMass=RocketCFG.mass_full,
                baseCollisionShapeIndex=body_collision,
                baseVisualShapeIndex=body_visual,
                basePosition=[0, 0, RocketCFG.start_height],
                baseOrientation=[0, 0, 0, 1]
            )

            p.changeDynamics(
                self.rocket_id, -1,
                restitution=0.05,
                lateralFriction=0.5,
                linearDamping=0.2,
                angularDamping=0.2,
                contactStiffness=200000,
                contactDamping=50000,
                physicsClientId=self.client
            )

            if self.gui:
                nose_body = p.createMultiBody(
                    baseMass=0,
                    baseVisualShapeIndex=nose_visual,
                    basePosition=[0, 0, RocketCFG.start_height + 1.5],
                    baseOrientation=[0, 0, 0, 1]
                )
                self.decoration_bodies.append(nose_body)

            print(f"Простая серая ракета создана")

        except Exception as e:
            print(f"Ошибка создания ракеты: {e}")
            self.simulation_error = True
            raise

    def get_bottom_height(self):
        position, _ = p.getBasePositionAndOrientation(self.rocket_id)
        bottom_height = position[2] - (self.rocket_height / 2)
        return bottom_height

    def check_ground_contact(self):
        if self.rocket_id is None:
            return False
        contact_points = p.getContactPoints(self.rocket_id, self.ground_id)
        return len(contact_points) > 0

    def finalize_landing(self):
        velocity, _ = p.getBaseVelocity(self.rocket_id)
        self.landing_velocity = abs(velocity[2])

        final_center_height = self.rocket_height / 2

        p.resetBasePositionAndOrientation(
            self.rocket_id,
            [0, 0, final_center_height],
            [0, 0, 0, 1]
        )
        p.resetBaseVelocity(self.rocket_id, [0, 0, 0], [0, 0, 0])

        print(f"\n=== ПОСАДКА ВЫПОЛНЕНА ===")
        print(f"Время: {self.sim_time:.2f} с")
        print(f"Высота нижней точки: 0.000 м")
        print(f"Скорость посадки: {self.landing_velocity:.2f} м/с")

        target_speed = abs(RocketCFG.target_speed_landing)
        if self.landing_velocity <= target_speed:
            print(f"РЕЗУЛЬТАТ: МЯГКАЯ ПОСАДКА [OK] (скорость {self.landing_velocity:.2f} <= {target_speed} м/с)")
        else:
            print(f"РЕЗУЛЬТАТ: ЖЕСТКАЯ ПОСАДКА [FAIL] (скорость {self.landing_velocity:.2f} > {target_speed} м/с)")

        if len(self.height_data) > 0:
            self.height_data[-1] = 0.0

        self.landing_complete = True

    def update_forces(self):
        if self.rocket_id is None or self.landing_complete:
            return

        try:
            position, orientation = p.getBasePositionAndOrientation(self.rocket_id)
            velocity, angular_vel = p.getBaseVelocity(self.rocket_id)

            height = position[2]
            vel_z = velocity[2]

            bottom_height = self.get_bottom_height()

            # ЗАЩИТА ОТ УХОДА ПОД ЗЕМЛЮ
            if bottom_height < -0.05:
                correct_height = self.rocket_height / 2
                p.resetBasePositionAndOrientation(
                    self.rocket_id,
                    [0, 0, correct_height],
                    [0, 0, 0, 1],
                    physicsClientId=self.client
                )
                p.resetBaseVelocity(self.rocket_id, [0, 0, 0], [0, 0, 0])
                bottom_height = 0
                print("⚠️ КОРРЕКЦИЯ: ракета возвращена на землю")

            has_contact = self.check_ground_contact()

            if bottom_height <= 0.1 or has_contact:
                self.stable_landing_frames += 1

                if self.stable_landing_frames >= 5 and not self.landing_triggered:
                    self.landing_triggered = True
                    self.is_landed = True
                    self.finalize_landing()
                    return
            else:
                if self.stable_landing_frames > 0:
                    self.stable_landing_frames = 0

            if bottom_height < 1.0 and bottom_height > 0.1 and not self.landing_triggered:
                brake_force = np.array([0, 0, 50000])
                p.applyExternalForce(
                    self.rocket_id, -1, brake_force, [0, 0, 0],
                    p.WORLD_FRAME, physicsClientId=self.client
                )

            if not self.landing_triggered:
                current_mass = self.physics.get_current_mass()
                p.changeDynamics(self.rocket_id, -1, mass=current_mass)

                vel_array = np.array([0, 0, vel_z])
                force = self.physics.total_force(self.sim_time, vel_array, height)

                p.applyExternalForce(
                    self.rocket_id, -1, force, [0, 0, 0],
                    p.WORLD_FRAME, physicsClientId=self.client
                )

                self.time_data.append(self.sim_time)
                self.height_data.append(bottom_height)
                self.velocity_data.append(vel_z)
                self.thrust_data.append(force[2])
                self.mass_data.append(current_mass)

                if self.gui and len(self.time_data) % 30 == 0:
                    thrust_status = "ВКЛЮЧЕН" if abs(force[2]) > 500 else "ВЫКЛЮЧЕН"
                    contact_status = "ЗЕМЛЯ" if has_contact else "ВОЗДУХ"
                    print(
                        f"Время: {self.sim_time:.2f}с | Нижняя точка: {bottom_height:.3f}м | "
                        f"Скорость: {vel_z:.2f}м/с | Тяга: {force[2]:.0f}Н | "
                        f"{thrust_status} | {contact_status}"
                    )

        except Exception as e:
            print(f"Ошибка в update_forces: {e}")

    def update_camera(self):
        if not self.gui or self.rocket_id is None or self.landing_complete:
            return

        try:
            position, _ = p.getBasePositionAndOrientation(self.rocket_id)
            height = position[2]

            if height > 100:
                camera_distance = 25
                camera_pitch = -35
            elif height > 50:
                camera_distance = 20
                camera_pitch = -30
            elif height > 20:
                camera_distance = 15
                camera_pitch = -25
            elif height > 5:
                camera_distance = 12
                camera_pitch = -22
            else:
                camera_distance = 8
                camera_pitch = -18

            p.resetDebugVisualizerCamera(
                cameraDistance=camera_distance,
                cameraYaw=30,
                cameraPitch=camera_pitch,
                cameraTargetPosition=[0, 0, max(height, 1.5)],
                physicsClientId=self.client
            )
        except:
            pass

    def cleanup(self):
        try:
            for line in self.debug_lines:
                try:
                    p.removeUserDebugItem(line)
                except:
                    pass
            self.debug_lines = []

            for body in self.decoration_bodies:
                try:
                    p.removeBody(body)
                except:
                    pass
            self.decoration_bodies = []

            if self.client is not None:
                p.disconnect(physicsClientId=self.client)
                self.client = None

        except Exception as e:
            print(f"Ошибка при очистке: {e}")

    def run_simulation(self):
        print("СИМУЛЯЦИЯ ПОСАДКИ РАКЕТЫ")
        print(f"Высота ракеты: {self.rocket_height} м")
        print(f"Начальная высота (центр масс): {RocketCFG.start_height} м")
        print(f"Начальная высота (нижняя точка): {RocketCFG.start_height - self.rocket_height / 2:.2f} м")
        print(f"Масса ракеты: {RocketCFG.mass_full} кг")
        print(f"Максимальная тяга: {RocketCFG.max_thrust} Н")
        print(f"Допустимая скорость посадки: {abs(RocketCFG.target_speed_landing)} м/с")

        try:
            self.setup_pybullet()
            self.load_rocket()

            if self.simulation_error:
                print("Ошибка инициализации")
                return

            max_steps = 100000
            step_count = 0

            while step_count < max_steps and not self.landing_complete and not self.simulation_error:
                self.update_forces()

                if self.simulation_error or self.landing_complete:
                    break

                p.stepSimulation()
                self.update_camera()
                self.sim_time += Simcfg.step
                step_count += 1

                if self.gui:
                    time.sleep(Simcfg.step)

        except KeyboardInterrupt:
            print("\nПрервано")
        except Exception as e:
            print(f"\nОшибка: {e}")
            import traceback
            traceback.print_exc()
        finally:
            if len(self.time_data) > 0:
                self.show_results()
            self.cleanup()

    def show_results(self):
        if len(self.time_data) == 0:
            print("Нет данных для отображения")
            return

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Результаты посадки ракеты', fontsize=16)

        ax1.plot(self.time_data, self.height_data, 'b-', linewidth=2)
        ax1.axhline(y=0, color='r', linestyle='--', linewidth=2, label='Уровень земли')
        ax1.set_xlabel('Время (с)')
        ax1.set_ylabel('Высота (м)')
        ax1.set_title('Высота нижней точки ракеты')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        if len(self.height_data) > 0:
            ax1.annotate(f'Конечная высота: {self.height_data[-1]:.3f} м',
                         xy=(self.time_data[-1], self.height_data[-1]),
                         xytext=(10, 10), textcoords='offset points',
                         fontsize=10, color='blue',
                         bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))

        ax2.plot(self.time_data, self.velocity_data, 'r-', linewidth=2)
        target_speed = abs(RocketCFG.target_speed_landing)
        ax2.axhline(y=target_speed, color='g', linestyle='--', label=f'Макс. скорость посадки ({target_speed} м/с)')
        ax2.axhline(y=-target_speed, color='g', linestyle='--')
        ax2.axhline(y=0, color='k', linestyle='-', linewidth=1, alpha=0.5)
        ax2.set_xlabel('Время (с)')
        ax2.set_ylabel('Скорость (м/с)')
        ax2.set_title('Вертикальная скорость')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        ax3.plot(self.time_data, self.thrust_data, 'g-', linewidth=2)
        ax3.set_xlabel('Время (с)')
        ax3.set_ylabel('Тяга (Н)')
        ax3.set_title('Тяга двигателя')
        ax3.grid(True, alpha=0.3)

        ax4.plot(self.time_data, self.mass_data, 'm-', linewidth=2)
        ax4.set_xlabel('Время (с)')
        ax4.set_ylabel('Масса (кг)')
        ax4.set_title('Масса ракеты')
        ax4.grid(True, alpha=0.3)

        target_speed = abs(RocketCFG.target_speed_landing)
        is_soft_landing = self.landing_velocity <= target_speed

        result_text = f"""
        РЕЗУЛЬТАТЫ ПОСАДКИ:
        • Время посадки: {self.sim_time:.2f} с
        • Скорость посадки: {self.landing_velocity:.2f} м/с
        • Допустимая скорость: {target_speed} м/с
        • Конечная высота (нижняя точка): {self.height_data[-1]:.3f} м
        • Конечная масса: {self.mass_data[-1]:.1f} кг
        • Расход топлива: {RocketCFG.mass_fuel - (self.mass_data[-1] - RocketCFG.mass_empty):.1f} кг
        """

        if is_soft_landing:
            result_text += "\n    СТАТУС: УСПЕШНАЯ МЯГКАЯ ПОСАДКА [OK]"
            color = 'lightgreen'
        else:
            result_text += f"\n    СТАТУС: ЖЕСТКАЯ ПОСАДКА [FAIL]"
            color = 'lightcoral'

        fig.text(0.02, 0.02, result_text, fontsize=10,
                 bbox=dict(boxstyle="round", facecolor=color, alpha=0.9))

        plt.tight_layout()
        plt.show()


def main():
    print("ЗАПУСК СИМУЛЯТОРА ПОСАДКИ РАКЕТЫ")
    print("=" * 60)

    sim = None
    try:
        sim = RocketLandingSimulation(gui=True)
        sim.run_simulation()
    except Exception as e:
        print(f"Ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if sim is not None:
            sim.cleanup()
        print("\nСимуляция завершена")


if __name__ == "__main__":
    main()
