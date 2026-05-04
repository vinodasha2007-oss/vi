import pybullet as p
import pybullet_data
import numpy as np
from typing import Optional, Tuple


class RocketLoader:
    """Загрузчик URDF моделей ракеты"""

    def __init__(self, client):
        self.client = client

    def load(self, urdf_path: str, base_position=None, base_orientation=None) -> int:
        """Загрузка URDF файла"""
        if base_position is None:
            base_position = [0, 0, 0]
        if base_orientation is None:
            base_orientation = [0, 0, 0, 1]

        rocket_id = p.loadURDF(
            urdf_path,
            base_position,
            base_orientation,
            useFixedBase=False,
            physicsClientId=self.client
        )
        return rocket_id


class PyBulletSimulation:
    """Альтернативный класс симуляции (для совместимости)"""

    def __init__(self, gui: bool = True, debug: bool = False):
        self.debug = debug
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        self.rocket_id: Optional[int] = None
        self.ground_id: Optional[int] = None

    def setup_environment(self, urdf_path: str = "rocket.urdf"):
        """Настройка окружения"""
        from cfg import Simcfg

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(*Simcfg.gravity)
        self.ground_id = p.loadURDF("plane.urdf", useFixedBase=True)

        loader = RocketLoader(self.client)
        self.rocket_id = loader.load(urdf_path)

    def get_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Получение текущего состояния ракеты"""
        if self.rocket_id is None:
            raise RuntimeError("Ракета не загружена")

        position, orientation = p.getBasePositionAndOrientation(self.rocket_id)
        linear_vel, angular_vel = p.getBaseVelocity(self.rocket_id)

        return (
            np.array(position),
            np.array(orientation),
            np.array(linear_vel),
            np.array(angular_vel)
        )

    def set_position(self, position: list, orientation: list = [0, 0, 0, 1]):
        """Установка позиции ракеты"""
        if self.rocket_id is not None:
            p.resetBasePositionAndOrientation(self.rocket_id, position, orientation)

    def set_velocity(self, linear_vel: list = [0, 0, 0], angular_vel: list = [0, 0, 0]):
        """Установка скорости ракеты"""
        if self.rocket_id is not None:
            p.resetBaseVelocity(self.rocket_id, linear_vel, angular_vel)

    def apply_force(self, force: np.ndarray, position: list = [0, 0, 0], frame: int = p.WORLD_FRAME):
        """Приложение внешней силы"""
        if self.rocket_id is None:
            raise RuntimeError("Ракета не загружена")

        p.applyExternalForce(self.rocket_id, -1, force, position, frame)

    def step(self):
        """Шаг симуляции"""
        p.stepSimulation()

    def dispose(self):
        """Отключение от сервера"""
        if self.client is not None:
            p.disconnect(self.client)
