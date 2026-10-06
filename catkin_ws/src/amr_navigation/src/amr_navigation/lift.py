"""
Pallet Stacker Vertical Mast Elevation Library.
Provides smooth velocity-ramped mast elevation control.
"""
import math
import rospy
from std_msgs.msg import Float32


class LiftController:
    """
    Controller for commanding AMR pallet stacker lift height with smooth velocity ramping.
    """

    def __init__(self, topic="/lift_cmd", default_speed=0.06, initial_height=0.0):
        """
        :param topic: ROS topic name (default: /lift_cmd).
        :param default_speed: Vertical lift speed in meters per second (default: 0.06 m/s = 6 cm/s).
        :param initial_height: Assumed starting mast elevation in meters (default: 0.0m).
        """
        self.topic = topic
        self.default_speed = float(rospy.get_param("~lift_speed", default_speed))
        self.current_height = float(initial_height)
        self.pub = rospy.Publisher(topic, Float32, queue_size=10, latch=True)

    def set_height(self, target_height_m, speed=None, settling_time=0.3, wait_sec=None):
        """
        Smoothly ramps the mast elevation to target_height_m at the specified speed (m/s).

        :param target_height_m: Desired elevation in meters (e.g. 0.35m).
        :param speed: Speed in m/s (e.g. 0.05 to 0.08 m/s). If None, uses default_speed.
        :param settling_time: Duration to pause after reaching target for physical settling (seconds).
        :param wait_sec: Alias for settling_time.
        :return: True if target was reached, False if aborted.
        """
        if wait_sec is not None:
            settling_time = float(wait_sec)

        target = float(target_height_m)
        v = float(speed) if speed is not None else self.default_speed

        if v <= 0:
            # Fallback to direct step if non-positive speed is given
            self.current_height = target
            self.pub.publish(Float32(data=target))
            return True

        rate_hz = 30  # 30 Hz control loop for fluid motion
        rate = rospy.Rate(rate_hz)
        dt = 1.0 / rate_hz

        total_distance = abs(target - self.current_height)
        estimated_duration = total_distance / v if v > 0 else 0.0
        direction = 1.0 if target > self.current_height else -1.0

        rospy.loginfo(
            "[LiftController] Moving lift: %.3fm -> %.3fm (Dist: %.3fm | Speed: %.2fm/s | Duration: ~%.1fs)",
            self.current_height, target, total_distance, v, estimated_duration
        )

        last_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            # Handle simulation time properly
            actual_dt = now - last_time if (last_time > 0 and (now - last_time) > 0) else dt
            last_time = now

            step = v * actual_dt
            diff = abs(target - self.current_height)

            if diff <= step:
                self.current_height = target
                self.pub.publish(Float32(data=self.current_height))
                break

            self.current_height += direction * step
            self.pub.publish(Float32(data=self.current_height))
            rate.sleep()

        rospy.loginfo("[LiftController] Target elevation %.3fm reached.", self.current_height)

        if settling_time > 0 and not rospy.is_shutdown():
            rospy.sleep(settling_time)

        return True

    def raise_lift(self, height_m=0.35, speed=None, settling_time=0.3):
        """Convenience method to elevate pallet."""
        return self.set_height(height_m, speed=speed, settling_time=settling_time)

    def lower_lift(self, height_m=0.0, speed=None, settling_time=0.3):
        """Convenience method to lower pallet."""
        return self.set_height(height_m, speed=speed, settling_time=settling_time)
