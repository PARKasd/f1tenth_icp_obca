#!/usr/bin/env python3
"""Pose 토픽(nav_msgs/Odometry)을 TUM 궤적 파일로 기록.

사용: record_pose_tum.py <topic> <output.tum>
종료: SIGTERM/SIGINT — finally에서 파일을 닫아 flush를 보장한다.
"""
import signal
import sys

import rclpy
from nav_msgs.msg import Odometry


def main():
    topic, out_path = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node("record_pose_tum")
    f = open(out_path, "w")

    def cb(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        f.write(f"{s:.6f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
                f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")

    node.create_subscription(Odometry, topic, cb, 10)
    # SIGTERM(하네스 정리)에서도 finally가 타도록 SystemExit으로 변환.
    # spin_once 타임아웃이 있어야 C waitset에 블록된 동안에도 핸들러가 실행된다.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        f.close()
        node.destroy_node()
        if rclpy.ok():  # SIGINT 경로는 rclpy가 이미 context를 내렸을 수 있다
            rclpy.shutdown()


if __name__ == "__main__":
    main()
