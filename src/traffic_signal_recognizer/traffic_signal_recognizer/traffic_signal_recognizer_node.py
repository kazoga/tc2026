#!/usr/bin/env python3
"""traffic_signal_recognizer ノードの実装モジュール."""

import copy
import threading
from typing import List, Optional, Sequence, Tuple

import cv2
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from vision_msgs.msg import Detection2D, Detection2DArray

from traffic_signal_recognizer.signal_recognition_core import (
    DetectionCandidate,
    SignalDecision,
    TrafficSignalRecognitionCore,
)


class TrafficSignalRecognizerNode(Node):
    """YOLO 検出結果から信号の GO/STOP を判定するノード."""

    def __init__(self) -> None:
        super().__init__('traffic_signal_recognizer')

        self._declare_parameters()
        self._load_parameters()

        self.core = TrafficSignalRecognitionCore(
            confidence_threshold=self.confidence_threshold,
            judge_count=self.judge_count,
            go_status=self.go_status,
            stop_status=self.stop_status,
            unknown_class_id=self.unknown_class_id,
            green_class_ids=self.green_class_ids,
            red_class_ids=self.red_class_ids,
            green_class_names=self.green_class_names,
            red_class_names=self.red_class_names,
            hold_go=self.hold_go,
        )

        self.bridge = CvBridge()
        self.enabled = False
        self.latest_image: Optional[np.ndarray] = None
        self.image_lock = threading.Lock()

        self.create_subscription(Int32, self.recog_flag_topic, self._recog_flag_callback, 10)
        self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self._detections_callback,
            10,
        )

        self.sig_recog_publisher = self.create_publisher(Int32, self.sig_recog_topic, 10)
        self.sig_image_publisher = self.create_publisher(Image, self.sig_det_image_topic, 10)

        self.get_logger().info(
            'traffic_signal_recognizer を起動しました。'
            f' recog_flag={self.recog_flag_topic}, detections={self.detections_topic}, '
            f'image={self.image_topic}, sig_recog={self.sig_recog_topic}, '
            f'sig_det_imgs={self.sig_det_image_topic}'
        )

    def _declare_parameters(self) -> None:
        """ノードが使用するパラメータを宣言する."""

        self.declare_parameter('recog_flag_topic', '/recog_flag')
        self.declare_parameter('detections_topic', '/perception/traffic_signal/detections')
        self.declare_parameter('image_topic', '/usb_cam/image_raw')
        self.declare_parameter('sig_recog_topic', '/sig_recog')
        self.declare_parameter('sig_det_image_topic', '/sig_det_imgs')
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('judge_count', 3)
        self.declare_parameter('go_status', 1)
        self.declare_parameter('stop_status', 2)
        self.declare_parameter('unknown_class_id', 99)
        self.declare_parameter('green_class_ids', [1])
        self.declare_parameter('red_class_ids', [0])
        self.declare_parameter('green_class_names', ['green'])
        self.declare_parameter('red_class_names', ['red'])
        self.declare_parameter('class_names', ['red', 'green'])
        self.declare_parameter('hold_go', False)
        self.declare_parameter('publish_stop_when_disabled', False)
        self.declare_parameter('publish_image_when_disabled', True)

    def _load_parameters(self) -> None:
        """宣言済みパラメータを読み込む."""

        self.recog_flag_topic = self._get_string_parameter('recog_flag_topic')
        self.detections_topic = self._get_string_parameter('detections_topic')
        self.image_topic = self._get_string_parameter('image_topic')
        self.sig_recog_topic = self._get_string_parameter('sig_recog_topic')
        self.sig_det_image_topic = self._get_string_parameter('sig_det_image_topic')
        self.confidence_threshold = self._get_double_parameter('confidence_threshold')
        self.judge_count = self._get_int_parameter('judge_count')
        self.go_status = self._get_int_parameter('go_status')
        self.stop_status = self._get_int_parameter('stop_status')
        self.unknown_class_id = self._get_int_parameter('unknown_class_id')
        self.green_class_ids = self._get_int_array_parameter('green_class_ids')
        self.red_class_ids = self._get_int_array_parameter('red_class_ids')
        self.green_class_names = self._get_string_array_parameter('green_class_names')
        self.red_class_names = self._get_string_array_parameter('red_class_names')
        self.class_names = self._get_string_array_parameter('class_names')
        self.hold_go = self._get_bool_parameter('hold_go')
        self.publish_stop_when_disabled = self._get_bool_parameter('publish_stop_when_disabled')
        self.publish_image_when_disabled = self._get_bool_parameter('publish_image_when_disabled')

    def _get_string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _get_int_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _get_bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _get_int_array_parameter(self, name: str) -> List[int]:
        return [int(value) for value in self.get_parameter(name).value]

    def _get_string_array_parameter(self, name: str) -> List[str]:
        return [str(value) for value in self.get_parameter(name).value]

    def _recog_flag_callback(self, msg: Int32) -> None:
        """recog_flag の最新値に応じて判定有効状態を更新する."""

        next_enabled = int(msg.data) == 1
        if self.enabled == next_enabled:
            return

        self.enabled = next_enabled
        self.core.reset()
        state_text = '有効' if self.enabled else '無効'
        self.get_logger().info(
            f'信号認識を{state_text}にしました。recog_flag={msg.data}'
        )

        if not self.enabled and self.publish_stop_when_disabled:
            self._publish_sig_recog(self.stop_status)

    def _image_callback(self, msg: Image) -> None:
        """画像を保持し、無効時は監視画像としてそのまま再配信する."""

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'画像変換に失敗しました: {exc}')
            return

        with self.image_lock:
            self.latest_image = cv_image.copy()

        if not self.enabled and self.publish_image_when_disabled:
            self.sig_image_publisher.publish(msg)

    def _detections_callback(self, msg: Detection2DArray) -> None:
        """Detection2DArray を受信し、信号判定を行う."""

        if not self.enabled:
            return

        candidates = self._extract_candidates(msg.detections)
        decision = self.core.update(candidates)
        self._publish_sig_recog(decision.status)
        self._publish_signal_image(msg.detections, decision)

        self.get_logger().debug(
            '信号認識: '
            f'status={decision.status}, class={decision.selected_class_name}, '
            f'score={decision.selected_score:.2f}, history={decision.history}'
        )

    def _extract_candidates(self, detections: Sequence[Detection2D]) -> List[DetectionCandidate]:
        """Detection2D から判定候補を抽出する."""

        candidates: List[DetectionCandidate] = []
        for detection in detections:
            best = self._extract_best_result(detection)
            if best is None:
                continue
            class_id, score = best
            candidates.append(
                DetectionCandidate(
                    class_id=class_id,
                    class_name=self._resolve_class_name(class_id),
                    score=score,
                )
            )
        return candidates

    def _extract_best_result(self, detection: Detection2D) -> Optional[Tuple[int, float]]:
        """Detection2D.results からスコア最大の (class_id, score) を返す."""

        best_pair: Optional[Tuple[int, float]] = None
        for result in detection.results:
            try:
                class_id = int(result.id)
                score = float(result.score)
            except (TypeError, ValueError):
                continue
            if best_pair is None or score > best_pair[1]:
                best_pair = (class_id, score)
        return best_pair

    def _resolve_class_name(self, class_id: int) -> str:
        """class id から class name を取得する."""

        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return f'class_{class_id}'

    def _publish_sig_recog(self, status: int) -> None:
        """sig_recog を publish する."""

        self.sig_recog_publisher.publish(Int32(data=int(status)))

    def _publish_signal_image(
        self, detections: Sequence[Detection2D], decision: SignalDecision
    ) -> None:
        """信号判定を重畳した画像を publish する."""

        with self.image_lock:
            if self.latest_image is None:
                return
            image = copy.deepcopy(self.latest_image)

        self._draw_decision_border(image, decision)
        self._draw_detections(image, detections)

        try:
            image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'信号認識画像の変換に失敗しました: {exc}')
            return
        self.sig_image_publisher.publish(image_msg)

    def _draw_decision_border(self, image: np.ndarray, decision: SignalDecision) -> None:
        """GO/STOP 判定を画像枠として描画する."""

        color = (0, 255, 0) if decision.status == self.go_status else (0, 0, 255)
        height, width = image.shape[:2]
        cv2.rectangle(image, (0, 0), (width - 1, height - 1), color, 5)
        label = f'sig_recog={decision.status}'
        cv2.putText(image, label, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

    def _draw_detections(self, image: np.ndarray, detections: Sequence[Detection2D]) -> None:
        """検出矩形を画像へ描画する."""

        for detection in detections:
            best = self._extract_best_result(detection)
            if best is None:
                continue
            class_id, score = best
            if score < self.confidence_threshold:
                continue

            class_name = self._resolve_class_name(class_id)
            color = (0, 255, 0) if class_name.lower() == 'green' else (0, 0, 255)
            x1, y1, x2, y2 = self._bbox_to_xyxy(detection)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            label = f'{class_name}:{score:.2f}'
            cv2.putText(
                image,
                label,
                (x1, max(y1 - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

    @staticmethod
    def _bbox_to_xyxy(detection: Detection2D) -> Tuple[int, int, int, int]:
        """Detection2D.bbox を OpenCV 描画用座標へ変換する."""

        center_x = detection.bbox.center.x
        center_y = detection.bbox.center.y
        half_w = detection.bbox.size_x / 2.0
        half_h = detection.bbox.size_y / 2.0
        return (
            int(center_x - half_w),
            int(center_y - half_h),
            int(center_x + half_w),
            int(center_y + half_h),
        )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)

    node = TrafficSignalRecognizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
