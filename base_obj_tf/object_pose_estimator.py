"""3차원 점들로부터 평면 물체의 중심과 방향을 계산한다."""

import numpy as np


# 평면에서 4 mm 이내의 점만 같은 평면으로 본다.
RANSAC_DISTANCE = 0.004
RANSAC_ITERATIONS = 180
RANSAC_SAMPLE_SIZE = 5000
MIN_OBJECT_POINTS = 80


class ObjectPoseEstimator:
    """RANSAC으로 평면을 찾고 PCA로 중심과 세 축을 계산한다."""

    def __init__(self):
        self.previous_axes = None
        self.rng = np.random.default_rng(0)

    def estimate(self, points: np.ndarray):
        """camera 기준 3차원 점들을 4x4 물체 자세 행렬로 바꾼다."""
        if len(points) < MIN_OBJECT_POINTS:
            return None

        plane_mask = self._find_plane_mask(points)
        if plane_mask is None or plane_mask.sum() < MIN_OBJECT_POINTS:
            return None

        center, axes = self._calculate_object_axes(points[plane_mask])
        self.previous_axes = axes.copy()

        camera_object = np.eye(4)
        camera_object[:3, :3] = axes
        camera_object[:3, 3] = center
        return camera_object

    def _find_plane_mask(self, points: np.ndarray):
        """RANSAC으로 가장 많은 점을 포함하는 평면 영역을 찾는다."""
        sample = self._sample_points(points)
        best_plane = None
        best_count = 0

        for _ in range(RANSAC_ITERATIONS):
            plane = self._plane_from_three_points(sample)
            if plane is None:
                continue

            normal, offset = plane
            count = np.count_nonzero(
                self._plane_distances(sample, normal, offset) < RANSAC_DISTANCE
            )
            if count > best_count:
                best_plane = plane
                best_count = count

        if best_plane is None:
            return None

        normal, offset = best_plane
        for _ in range(2):
            plane_mask = (
                self._plane_distances(points, normal, offset)
                < RANSAC_DISTANCE
            )
            if plane_mask.sum() < 3:
                return None
            normal, offset = self._refine_plane(points[plane_mask])

        return self._plane_distances(points, normal, offset) < RANSAC_DISTANCE

    def _sample_points(self, points):
        if len(points) <= RANSAC_SAMPLE_SIZE:
            return points
        indices = self.rng.choice(len(points), RANSAC_SAMPLE_SIZE, replace=False)
        return points[indices]

    def _plane_from_three_points(self, points):
        selected = points[self.rng.choice(len(points), 3, replace=False)]
        normal = np.cross(selected[1] - selected[0], selected[2] - selected[0])
        length = np.linalg.norm(normal)
        if length < 1e-8:
            return None

        normal /= length
        offset = -np.dot(normal, selected[0])
        return normal, offset

    @staticmethod
    def _plane_distances(points, normal, offset):
        return np.abs(points @ normal + offset)

    @staticmethod
    def _refine_plane(points):
        center = points.mean(axis=0)
        _, _, vh = np.linalg.svd(points - center, full_matrices=False)
        normal = vh[-1]
        normal /= np.linalg.norm(normal)
        return normal, -np.dot(normal, center)

    def _calculate_object_axes(self, points: np.ndarray):
        """PCA로 물체 중심과 서로 직각인 x, y, z축을 구한다."""
        center = np.median(points, axis=0)
        covariance = np.cov(points - center, rowvar=False)
        _, eigenvectors = np.linalg.eigh(covariance)

        # eigh의 고윳값은 작은 순서다.
        # 평면에서 변화가 가장 작은 방향(평면의 법선)을 z축으로 사용한다.
        z_axis = eigenvectors[:, 0]
        x_axis = eigenvectors[:, 2]
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        axes = np.column_stack((x_axis, y_axis, z_axis))

        axes = self._stabilize_axis_signs(axes, center)
        return center, axes

    def _stabilize_axis_signs(self, axes, center):
        """PCA 축이 프레임마다 반대 방향으로 뒤집히는 것을 막는다."""
        if self.previous_axes is None:
            if np.dot(axes[:, 2], -center) < 0:
                axes[:, [1, 2]] *= -1
            if axes[0, 0] < 0:
                axes[:, [0, 1]] *= -1
            return axes

        # 오른손 좌표계를 유지하면서 가능한 네 가지 축 방향이다.
        sign_sets = np.array(
            [
                [1.0, 1.0, 1.0],
                [1.0, -1.0, -1.0],
                [-1.0, 1.0, -1.0],
                [-1.0, -1.0, 1.0],
            ]
        )
        candidates = [axes * signs for signs in sign_sets]
        scores = [
            np.trace(self.previous_axes.T @ candidate)
            for candidate in candidates
        ]
        return candidates[int(np.argmax(scores))]
