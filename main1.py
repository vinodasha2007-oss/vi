# main.py - УПРАВЛЯЕМОЕ СНИЖЕНИЕ С ДВИГАТЕЛЕМ
import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
import time
import os
from cfg import RocketCFG, Simcfg
from roket import calculate_drag, calculate_gravity, calculate_thrust


class RocketLandingSimulation:
    def __init__(self, gui=True):
        self.gui = gui
        self.client = None
        self.rocket_id = None
        self.ground_id = None

        self.time_data = []
        self.height_data = []
        self.velocity_data = []
        self.thrust_data = []
        self.mass_data = []

        self.sim_time = 0.0
        self.landing_complete = False
        self.landing_velocity = 0.0

        self.mass_current = RocketCFG.mass_full
        self.fuel_left = RocketCFG.mass_fuel
        self.engine_on = True

        self.rocket_height = 3.0
        self.stable_landing_frames = 0
        self.decoration_bodies = []

    def setup_pybullet(self):
        if self.gui:
            self.client = p.connect(p.GUI)
            p.resetDebugVisualizerCamera(20, 30, -30, [0, 0, RocketCFG.start_height])
        else:
            self.client = p.connect(p.DIRECT)

        current_dir = os.getcwd()
        p.setAdditionalSearchPath(current_dir)
        models_rocket_dir = os.path.join(current_dir, "models", "rocket")
        if os.path.exists(models_rocket_dir):
            p.setAdditionalSearchPath(models_rocket_dir)

        p.setGravity(*Simcfg.gravity)
        p.setTimeStep(Simcfg.step)

        # Земля
        ground_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[100, 100, 0.1])
        ground_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[100, 100, 0.1], rgbaColor=[0.2, 0.7, 0.2, 1])
        self.ground_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=ground_collision,
                                           baseVisualShapeIndex=ground_visual, basePosition=[0, 0, -0.1])

        # Площадка
        pad = p.createVisualShape(p.GEOM_BOX, halfExtents=[5, 5, 0.05], rgbaColor=[0.5, 0.5, 0.5, 1])
        self.decoration_bodies.append(p.createMultiBody(baseMass=0, baseVisualShapeIndex=pad, basePosition=[0, 0, 0]))
        circle = p.createVisualShape(p.GEOM_CYLINDER, radius=3, length=0.1, rgbaColor=[0.9, 0.2, 0.2, 1])
        self.decoration_bodies.append(p.createMultiBody(baseMass=0, baseVisualShapeIndex=circle, basePosition=[0, 0, 0.03]))

        print("Готово")

    def load_rocket(self):
        paths = ["models/rocket/model.urdf", "models/rocket/prototype.urdf", "rocket.urdf", "model.urdf"]
        urdf_path = next((p for p in paths if os.path.exists(p)), None)
        if not urdf_path:
            raise FileNotFoundError("URDF не найден")

        print(f"Загрузка: {urdf_path}")
        self.rocket_id = p.loadURDF(urdf_path, [0, 0, RocketCFG.start_height], [0, 0, 0, 1],
                                    useFixedBase=False, flags=p.URDF_USE_INERTIA_FROM_FILE)

        p.changeVisualShape(self.rocket_id, -1, rgbaColor=[0.7, 0.7, 0.7, 1.0])
        p.resetBaseVelocity(self.rocket_id, linearVelocity=[0, 0, RocketCFG.start_velocity_vertical])
        p.changeDynamics(self.rocket_id, -1, restitution=0.05, lateralFriction=0.5,
                         linearDamping=0.2, angularDamping=0.2,
                         contactStiffness=200000, contactDamping=50000)

    def get_bottom_height(self):
        pos, _ = p.getBasePositionAndOrientation(self.rocket_id)
        return pos[2] - self.rocket_height / 2

    def get_vertical_speed(self):
        vel, _ = p.getBaseVelocity(self.rocket_id)
        return vel[2]

    def check_ground_contact(self):
        return len(p.getContactPoints(self.rocket_id, self.ground_id)) > 0

    def update_mass(self, dt):
        if self.engine_on and self.fuel_left > 0 and self.sim_time < RocketCFG.burn_time:
            dm = RocketCFG.fuel_consumption_rate * dt
            self.fuel_left = max(0, self.fuel_left - dm)
            self.mass_current = RocketCFG.mass_empty + RocketCFG.mass_engine_wet + self.fuel_left
            p.changeDynamics(self.rocket_id, -1, mass=self.mass_current)
        elif self.sim_time >= RocketCFG.burn_time and self.engine_on:
            self.engine_on = False
            self.mass_current = RocketCFG.mass_empty + RocketCFG.mass_engine_dry
            p.changeDynamics(self.rocket_id, -1, mass=self.mass_current)

    def apply_forces(self):
        """Расчёт сил такой же как в большом main'е"""
        vel, _ = p.getBaseVelocity(self.rocket_id)
        height = self.get_bottom_height()
        speed = vel[2]

        vel_np = np.array(vel)
        drag = calculate_drag(vel_np)
        gravity = calculate_gravity(self.mass_current)
        thrust = calculate_thrust(self.sim_time, height, speed, self.mass_current)

        total = thrust + drag + gravity
        p.applyExternalForce(self.rocket_id, -1, total, [0, 0, 0], p.WORLD_FRAME)

        pos, _ = p.getBasePositionAndOrientation(self.rocket_id)
        p.resetBasePositionAndOrientation(self.rocket_id, pos, [0, 0, 0, 1])

        return thrust[2]  # Возвращаем величину тяги для графиков

    def update_camera(self):
        if not self.gui or self.rocket_id is None:
            return
        pos, _ = p.getBasePositionAndOrientation(self.rocket_id)
        height = pos[2]
        if height > 100: dist, pitch = 25, -35
        elif height > 50: dist, pitch = 20, -30
        elif height > 20: dist, pitch = 15, -25
        elif height > 5: dist, pitch = 12, -22
        else: dist, pitch = 8, -18
        p.resetDebugVisualizerCamera(dist, 30, pitch, [0, 0, max(height, 1.5)])

    def finalize_landing(self):
        velocity, _ = p.getBaseVelocity(self.rocket_id)
        self.landing_velocity = abs(velocity[2])
        p.resetBasePositionAndOrientation(self.rocket_id, [0, 0, self.rocket_height/2], [0, 0, 0, 1])
        p.resetBaseVelocity(self.rocket_id, [0, 0, 0], [0, 0, 0])
        print(f"\n=== ПОСАДКА === t={self.sim_time:.1f}с V={self.landing_velocity:.1f}м/с {'МЯГКАЯ' if self.landing_velocity<=2 else 'ЖЕСТКАЯ'} ===")
        self.landing_complete = True

    def check_landing(self):
        bottom = self.get_bottom_height()
        if bottom <= 0.1 or self.check_ground_contact():
            self.stable_landing_frames += 1
            if self.stable_landing_frames >= 5:
                self.finalize_landing()
                return True
        else:
            self.stable_landing_frames = 0
        return False

    def run_simulation(self):
        print(f"СИМУЛЯЦИЯ | H={RocketCFG.start_height}м V={RocketCFG.start_velocity_vertical}м/с")
        self.setup_pybullet()
        self.load_rocket()

        dt = Simcfg.step
        step = 0
        last_print = 0

        try:
            while not self.landing_complete and step < 100000:
                self.update_mass(dt)
                thrust = self.apply_forces()
                p.stepSimulation()

                height = self.get_bottom_height()
                speed = self.get_vertical_speed()

                if self.sim_time - last_print >= 0.5:
                    self.time_data.append(self.sim_time)
                    self.height_data.append(height)
                    self.velocity_data.append(speed)
                    self.thrust_data.append(thrust)
                    self.mass_data.append(self.mass_current)
                    print(f"t={self.sim_time:5.1f}s H={height:6.0f}м V={speed:+5.0f}м/с")
                    last_print = self.sim_time

                if step % 3 == 0:
                    self.update_camera()

                if self.check_landing():
                    break

                self.sim_time += dt
                step += 1
                if self.gui:
                    time.sleep(Simcfg.step)

        except KeyboardInterrupt:
            print("Стоп")

        if self.time_data:
            self.show_graphs()
        p.disconnect()

    def show_graphs(self):
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        ax1.plot(self.time_data, self.height_data, 'b-')
        ax1.axhline(y=0, color='r', linestyle='--')
        ax1.set_ylabel('Высота (м)')
        ax1.grid(True)
        ax2.plot(self.time_data, self.velocity_data, 'r-')
        ax2.set_xlabel('Время (с)')
        ax2.set_ylabel('Скорость (м/с)')
        ax2.grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    RocketLandingSimulation(gui=True).run_simulation()
