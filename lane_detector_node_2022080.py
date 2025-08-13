#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import warnings
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

warnings.simplefilter('ignore', np.RankWarning)  # 다항식 경고 무시

class LaneEstimator:
    def __init__(self):
        rospy.init_node("lane_estimator_node")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.left_pub = rospy.Publisher('/lane_info/left', Float32MultiArray, queue_size=1)
        self.right_pub = rospy.Publisher('/lane_info/right', Float32MultiArray, queue_size=1)

        self.pixel_thresh = 50

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn("Image conversion error: %s" % str(e))
            return

        h, w = image.shape[:2]
        roi_x, roi_y, roi_w, roi_h = w // 2 - 300, h // 3, 600, 2 * h // 3
        roi = image[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        yellow_range = (np.array([10, 80, 80], dtype=np.uint8), np.array([30, 255, 255], dtype=np.uint8))
        white_range = (np.array([0, 0, 200], dtype=np.uint8), np.array([180, 50, 255], dtype=np.uint8))

        yellow_mask = self._apply_color_filter(hsv, yellow_range)
        white_mask = self._apply_color_filter(hsv, white_range)

        bird_img, mask_y_bird, mask_w_bird, M, M_inv = self._warp_perspective(image, yellow_mask, white_mask, roi_x, roi_y, roi_w, roi_h)

        bird_img_with_lanes = bird_img.copy()
        original_with_lanes = image.copy()

        self._fit_and_draw(bird_img_with_lanes, mask_y_bird, True, original_with_lanes, M_inv)
        self._fit_and_draw(bird_img_with_lanes, mask_w_bird, False, original_with_lanes, M_inv)

        yellow_mask_full = np.zeros_like(image[:, :, 0])
        white_mask_full = np.zeros_like(image[:, :, 0])
        yellow_mask_full[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w] = yellow_mask
        white_mask_full[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w] = white_mask

        yellow_mask_overlay = cv2.bitwise_and(image, image, mask=yellow_mask_full)
        white_mask_overlay = cv2.bitwise_and(image, image, mask=white_mask_full)

        cv2.imshow("1. Yellow Lane Mask on Original", yellow_mask_overlay)
        cv2.imshow("2. White Lane Mask on Original", white_mask_overlay)
        cv2.imshow("3. Original with Lanes", original_with_lanes)
        cv2.imshow("4. Bird Eye View (Original)", bird_img)
        cv2.imshow("5. Bird Eye View with Lanes", bird_img_with_lanes)
        cv2.waitKey(1)

    def _apply_color_filter(self, hsv_roi, color_range):
        mask = cv2.inRange(hsv_roi, color_range[0], color_range[1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
        return mask

    def _warp_perspective(self, frame, yellow_mask, white_mask, x, y, w_roi, h_roi):
        h, w = frame.shape[:2]
        src = np.float32([
            [x + w_roi * 0.15, y + h_roi],
            [x + w_roi * 0.85, y + h_roi],
            [x + w_roi * 0.65, y + h_roi * 0.5],
            [x + w_roi * 0.35, y + h_roi * 0.5]
        ])
        dst = np.float32([
            [w * 0.4, h],
            [w * 0.6, h],
            [w * 0.6, 0],
            [w * 0.4, 0]
        ])
        M = cv2.getPerspectiveTransform(src, dst)
        M_inv = cv2.getPerspectiveTransform(dst, src)

        full_y_mask = np.zeros_like(frame[:, :, 0])
        full_y_mask[y:y + h_roi, x:x + w_roi] = yellow_mask

        full_w_mask = np.zeros_like(frame[:, :, 0])
        full_w_mask[y:y + h_roi, x:x + w_roi] = white_mask

        return (
            cv2.warpPerspective(frame, M, (w, h)),
            cv2.warpPerspective(full_y_mask, M, (w, h)),
            cv2.warpPerspective(full_w_mask, M, (w, h)),
            M,
            M_inv
        )

    def _fit_and_draw(self, bev_img, mask, is_left, orig_img, M_inv):
        nonzeroy, nonzerox = mask.nonzero()
        xs = nonzerox
        ys = nonzeroy

        if len(xs) < self.pixel_thresh or len(ys) < self.pixel_thresh:
            return

        coeffs = self.adaptive_polyfit(xs, ys)
        if coeffs is None:
            rospy.logwarn("Fitting failed on %s lane", "left" if is_left else "right")
            return

        y_vals = np.linspace(0, bev_img.shape[0] - 1, bev_img.shape[0])
        x_vals = np.polyval(coeffs, y_vals)
        pts = np.array([np.transpose(np.vstack([x_vals, y_vals]))], dtype=np.int32)

        color = (0, 255, 0) if is_left else (0, 165, 255)

        cv2.polylines(bev_img, pts, isClosed=False, color=color, thickness=5)

        C0 = coeffs[-1]
        C1 = coeffs[-2] if len(coeffs) > 1 else 0
        C2 = coeffs[-3] if len(coeffs) > 2 else 0
        C3 = coeffs[-4] if len(coeffs) > 3 else 0

        offset = C0
        angle_rad = C1
        angle_deg = angle_rad * 180.0 / np.pi
        curvature = 2 * C2
        radius = 1.0 / curvature if curvature != 0 else float("inf")

        label = "Yellow Left" if is_left else "White Right"
        text_pos_y = 30 if is_left else 60
        text = "{}: {:.2f} deg | R={:.1f}m | Offset={:.1f}".format(label, angle_deg, radius, offset)
        cv2.putText(bev_img, text, (50, text_pos_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        msg = Float32MultiArray()
        msg.data = [C3, C2, C1, C0, curvature, radius, angle_deg, offset]
        if is_left:
            self.left_pub.publish(msg)
        else:
            self.right_pub.publish(msg)

        #  터미널 출력 추가
        rospy.loginfo("[%s Lane] C3=%.6f, C2=%.6f, C1=%.6f, C0=%.2f | Curvature=%.6f | Radius=%.2f m | Angle=%.2f deg | Offset=%.2f px",
                      "Left" if is_left else "Right", C3, C2, C1, C0, curvature, radius, angle_deg, offset)

    def adaptive_polyfit(self, xs, ys, max_degree=3):
        min_rmse = float('inf')
        best_fit = None

        for deg in range(1, max_degree + 1):
            try:
                coeffs = np.polyfit(ys, xs, deg)
                preds = np.polyval(coeffs, ys)
                rmse = np.sqrt(np.mean((xs - preds) ** 2))
                if rmse < min_rmse:
                    min_rmse = rmse
                    best_fit = coeffs
            except np.RankWarning:
                continue

        return best_fit

if __name__ == '__main__':
    try:
        LaneEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

