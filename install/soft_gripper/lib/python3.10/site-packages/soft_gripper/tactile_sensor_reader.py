import numpy as np

class TactileSensorData:
    def __init__(self, force_vector: np.ndarray, point_cloud):
        self.force_vector = force_vector  # np.array([x, y, z])
        self.point_cloud = point_cloud

class TactileSensorReader:
    def __init__(self):
        pass

    def read(self):
        # Dummy normal force between 2–4 N
        fz = 2.0 + 2.0 * np.random.rand()
        force_vector = np.array([0.0, 0.0, fz])

        class DummyPointCloud:
            max_motion = 1.5  # mm/s

        return TactileSensorData(force_vector, DummyPointCloud())

