from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class CircleDetectionConfig:
    blur_kernel_size: int
    hough_dp: float
    hough_min_dist_px: float
    hough_param1: float
    hough_param2: float
    min_radius_px: int
    max_radius_px: int


@dataclass(frozen=True)
class SphereEstimate:
    center_px: np.ndarray
    radius_px: float
    center_3d_m: np.ndarray


@dataclass(frozen=True)
class MeasurementConfig:
    sphere_diameter_m: float
    click_to_circle_threshold_px: float
    click_search_margin_px: float
    max_candidate_spheres: int


@dataclass(frozen=True)
class MeasurementResult:
    overlay_bgr: np.ndarray
    distance_m: Optional[float]
    valid: bool
    message: str
    point_a_3d_m: Optional[np.ndarray] = None
    point_b_3d_m: Optional[np.ndarray] = None


class PetanqueMeasurements:
    """Compute click-based sphere distance measurements from a single RGB frame.

    Pipeline:
    1) detect spheres as image circles,
    2) infer each sphere 3D center from known diameter,
    3) associate each user click to the closest circle edge,
    4) reconstruct 3D click points and measure their Euclidean distance.
    """

    def __init__(
        self,
        *,
        intrinsics: CameraIntrinsics,
        detection_config: CircleDetectionConfig,
        measurement_config: MeasurementConfig,
    ) -> None:
        self._intrinsics = intrinsics
        self._detection_config = detection_config
        self._measurement_config = measurement_config

    def set_intrinsics(self, intrinsics: CameraIntrinsics) -> None:
        # Allows the caller to refresh intrinsics at runtime (e.g., per frame).
        self._intrinsics = intrinsics

    @staticmethod
    def estimate_intrinsics_from_image(
        *,
        image_width_px: int,
        image_height_px: int,
        assumed_hfov_deg: float,
    ) -> CameraIntrinsics:
        # Lightweight intrinsics estimate from frame geometry + assumed horizontal FOV.
        # This is useful for testing but less accurate than calibrated CameraInfo.
        width = max(1, int(image_width_px))
        height = max(1, int(image_height_px))
        hfov_deg = float(assumed_hfov_deg)
        if hfov_deg <= 1.0:
            hfov_deg = 1.0
        if hfov_deg >= 179.0:
            hfov_deg = 179.0

        hfov_rad = np.deg2rad(hfov_deg)
        fx = float(width / (2.0 * np.tan(hfov_rad / 2.0)))
        fy = fx
        cx = float((width - 1) / 2.0)
        cy = float((height - 1) / 2.0)
        return CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy)

    def process(
        self,
        *,
        image_bgr: np.ndarray,
        point_a_px: Tuple[float, float],
        point_b_px: Tuple[float, float],
    ) -> MeasurementResult:
        # Keep user clicks authoritative: we visualize and measure from them directly,
        # and only use detected spheres to infer metric depth/scale.
        print(
            "Processing petanque measurement with clicks at "
            f"({point_a_px[0]:.1f}, {point_a_px[1]:.1f}) and "
            f"({point_b_px[0]:.1f}, {point_b_px[1]:.1f})"
        )
        if cv2 is None:
            return MeasurementResult(
                overlay_bgr=image_bgr,
                distance_m=None,
                valid=False,
                message="opencv not available",
                point_a_3d_m=None,
                point_b_3d_m=None,
            )

        overlay = image_bgr.copy()
        spheres = self._detect_spheres(image_bgr)

        point_a = np.array(point_a_px, dtype=np.float64)
        point_b = np.array(point_b_px, dtype=np.float64)

        # Keep only circles that are spatially relevant to at least one click.
        # spheres = self._filter_spheres_for_clicks(
        #     spheres=spheres,
        #     point_a=point_a,
        #     point_b=point_b,
        # )

        self._draw_click_marker(overlay, point_a, color=(255, 100, 0))
        self._draw_click_marker(overlay, point_b, color=(0, 220, 255))

        if not spheres:
            return MeasurementResult(
                overlay_bgr=overlay,
                distance_m=None,
                valid=False,
                message="no spheres detected",
                point_a_3d_m=None,
                point_b_3d_m=None,
            )
        print(f"Detected {len(spheres)} candidate spheres:")
        for sphere in spheres:
            cv2.circle(
                overlay,
                (int(round(sphere.center_px[0])), int(round(sphere.center_px[1]))),
                int(round(sphere.radius_px)),
                (80, 180, 80),
                2,
                lineType=cv2.LINE_AA,
            )
        cv2.imshow("Petanque Measurement Overlay 1", cv2.resize(overlay, (0, 0), fx=0.5, fy=0.5))
        cv2.waitKey(1)

        match_a = self._match_click_to_sphere(point_a, spheres)
        match_b = self._match_click_to_sphere(point_b, spheres)
        if match_a is None or match_b is None:
            if match_a is None:
                print("Failed to associate click A to any sphere")
            if match_b is None:
                print("Failed to associate click B to any sphere")
            return MeasurementResult(
                overlay_bgr=overlay,
                distance_m=None,
                valid=False,
                message="click too far from sphere",
                point_a_3d_m=None,
                point_b_3d_m=None,
            )

        point_a_3d = self._reconstruct_click_point_3d(point_a, match_a)
        point_b_3d = self._reconstruct_click_point_3d(point_b, match_b)
        if point_a_3d is None or point_b_3d is None:
            if point_a_3d is None:
                print("Failed to reconstruct 3D point for click A")
            if point_b_3d is None:
                print("Failed to reconstruct 3D point for click B")
            return MeasurementResult(
                overlay_bgr=overlay,
                distance_m=None,
                valid=False,
                message="failed 3d reconstruction",
                point_a_3d_m=None,
                point_b_3d_m=None,
            )

        distance_m = float(np.linalg.norm(point_a_3d - point_b_3d))
        self._draw_distance_annotation(
            overlay=overlay,
            point_a=point_a,
            point_b=point_b,
            distance_m=distance_m,
        )
        cv2.imshow("Petanque Measurement Overlay", cv2.resize(overlay, (0, 0), fx=0.5, fy=0.5))
        cv2.waitKey(1)

        return MeasurementResult(
            overlay_bgr=overlay,
            distance_m=distance_m,
            valid=True,
            message="ok",
            point_a_3d_m=point_a_3d,
            point_b_3d_m=point_b_3d,
        )

    def _filter_spheres_for_clicks(
        self,
        *,
        spheres: List[SphereEstimate],
        point_a: np.ndarray,
        point_b: np.ndarray,
    ) -> List[SphereEstimate]:
        if not spheres:
            return []

        margin = float(self._measurement_config.click_search_margin_px)
        candidates: List[Tuple[float, SphereEstimate]] = []
        for sphere in spheres:
            distance_to_a = float(np.linalg.norm(point_a - sphere.center_px))
            distance_to_b = float(np.linalg.norm(point_b - sphere.center_px))

            # Keep circles whose support region can plausibly include a click.
            max_distance = float(sphere.radius_px) + margin
            if distance_to_a <= max_distance or distance_to_b <= max_distance:
                radial_error_a = abs(distance_to_a - sphere.radius_px)
                radial_error_b = abs(distance_to_b - sphere.radius_px)
                score = min(radial_error_a, radial_error_b)
                candidates.append((score, sphere))

        if not candidates:
            return []

        candidates.sort(key=lambda item: item[0])
        max_count = max(1, int(self._measurement_config.max_candidate_spheres))
        return [sphere for _, sphere in candidates[:max_count]]

    def _detect_spheres(self, image_bgr: np.ndarray) -> List[SphereEstimate]:
        # Detect circles and convert each image circle into a 3D sphere-center estimate.
        if cv2 is None:
            return []

        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, self._detection_config.blur_kernel_size)

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=float(self._detection_config.hough_dp),
            minDist=float(self._detection_config.hough_min_dist_px),
            param1=float(self._detection_config.hough_param1),
            param2=float(self._detection_config.hough_param2),
            minRadius=int(self._detection_config.min_radius_px),
            maxRadius=int(self._detection_config.max_radius_px),
        )

        if circles is None:
            return []

        out: List[SphereEstimate] = []
        sphere_radius_m = float(self._measurement_config.sphere_diameter_m) / 2.0
        focal_px = (float(self._intrinsics.fx) + float(self._intrinsics.fy)) / 2.0

        for x, y, radius_px in circles[0]:
            if radius_px <= 1e-6:
                continue

            # Pinhole relation for apparent sphere radius:
            # radius_px = f * radius_m / z  ->  z = f * radius_m / radius_px
            z = (focal_px * sphere_radius_m) / float(radius_px)
            x_cam = ((float(x) - float(self._intrinsics.cx)) / float(self._intrinsics.fx)) * z
            y_cam = ((float(y) - float(self._intrinsics.cy)) / float(self._intrinsics.fy)) * z
            out.append(
                SphereEstimate(
                    center_px=np.array([float(x), float(y)], dtype=np.float64),
                    radius_px=float(radius_px),
                    center_3d_m=np.array([x_cam, y_cam, z], dtype=np.float64),
                )
            )
        return out

    def _match_click_to_sphere(
        self,
        click_px: np.ndarray,
        spheres: List[SphereEstimate],
    ) -> Optional[SphereEstimate]:
        # Association metric is distance to circle circumference, not center distance,
        # so we match where the user is actually pointing on/near the sphere edge.
        best_sphere: Optional[SphereEstimate] = None
        best_error = float("inf")
        for sphere in spheres:
            radial_error = abs(np.linalg.norm(click_px - sphere.center_px) - sphere.radius_px)
            if radial_error < best_error:
                best_error = radial_error
                best_sphere = sphere
        print( f"click to circle threshold: {self._measurement_config.click_to_circle_threshold_px:.1f} px, ")
        print(f"best radial error: {best_error:.1f} px for circle at ({best_sphere.center_px[0]:.1f}, {best_sphere.center_px[1]:.1f}) with radius {best_sphere.radius_px:.1f} px" if best_sphere else "no matching sphere")
        if (
            best_sphere is None
            or best_error > float(self._measurement_config.click_to_circle_threshold_px)
        ):
            return None
        return best_sphere

    def _reconstruct_click_point_3d(
        self,
        click_px: np.ndarray,
        sphere: SphereEstimate,
    ) -> Optional[np.ndarray]:
        # Build the camera ray passing through the user click.
        ray = np.array(
            [
                (float(click_px[0]) - float(self._intrinsics.cx)) / float(self._intrinsics.fx),
                (float(click_px[1]) - float(self._intrinsics.cy)) / float(self._intrinsics.fy),
                1.0,
            ],
            dtype=np.float64,
        )
        ray /= np.linalg.norm(ray)

        center = sphere.center_3d_m
        sphere_radius_m = float(self._measurement_config.sphere_diameter_m) / 2.0

        # Intersect ray P(t)=t*ray with sphere ||P-C||^2 = R^2.
        # This yields a quadratic at^2 + bt + c = 0 with a=1 for normalized ray.
        b = -2.0 * float(np.dot(ray, center))
        c = float(np.dot(center, center) - sphere_radius_m * sphere_radius_m)
        discriminant = b * b - 4.0 * c

        if discriminant < 0.0:
            # Numerical/association fallback: use same ray direction but at sphere-center depth.
            z = float(center[2])
            if z <= 0.0:
                return None
            scale = z / ray[2]
            return ray * scale

        sqrt_disc = float(np.sqrt(discriminant))
        t1 = (-b - sqrt_disc) / 2.0
        t2 = (-b + sqrt_disc) / 2.0
        candidates = [t for t in (t1, t2) if t > 0.0]
        if not candidates:
            return None

        # Choose the nearest positive intersection along the viewing ray.
        t = min(candidates)
        return ray * t

    @staticmethod
    def _draw_click_marker(overlay: np.ndarray, point: np.ndarray, color: Tuple[int, int, int]) -> None:
        if cv2 is None:
            return
        p = (int(round(point[0])), int(round(point[1])))
        cv2.circle(overlay, p, 8, color, 3)

    @staticmethod
    def _draw_distance_annotation(
        *,
        overlay: np.ndarray,
        point_a: np.ndarray,
        point_b: np.ndarray,
        distance_m: float,
    ) -> None:
        # Visual output requested by UX: two-way arrow + metric label on the segment.
        if cv2 is None:
            return

        p1 = (int(round(point_a[0])), int(round(point_a[1])))
        p2 = (int(round(point_b[0])), int(round(point_b[1])))
        color = (0, 255, 255)

        cv2.arrowedLine(overlay, p1, p2, color, 2, line_type=cv2.LINE_AA, tipLength=0.03)
        cv2.arrowedLine(overlay, p2, p1, color, 2, line_type=cv2.LINE_AA, tipLength=0.03)

        mid_x = int(round((point_a[0] + point_b[0]) / 2.0))
        mid_y = int(round((point_a[1] + point_b[1]) / 2.0))
        distance_cm = distance_m * 100.0
        label = f"{distance_cm:.1f} cm"
        cv2.putText(
            overlay,
            label,
            (mid_x + 8, mid_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )
