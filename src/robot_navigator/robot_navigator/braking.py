"""Stopping envelope in metres, seconds, and m/s² (independent of ROS)."""
import math


def stopping_distance(speed: float, deceleration: float, delay: float) -> float:
    """Constant deceleration plus worst-case travel during the reaction delay."""
    return abs(speed) * delay + speed * speed / (2.0 * deceleration)


def speed_for_distance(distance: float, deceleration: float, delay: float) -> float:
    """Nonnegative inverse of stopping_distance, numerically stable near zero."""
    if distance <= 0.0:
        return 0.0
    term = deceleration * delay
    return 2.0 * deceleration * distance / (math.sqrt(term * term + 2.0 * deceleration * distance) + term)


def limit_for_stop(proposed: float, measured: float, previous: float,
                   distance: float, deceleration: float, delay: float) -> float:
    """Never underestimate braking using a reduced command during turning."""
    if not all(math.isfinite(x) for x in (proposed, measured, previous, distance)):
        return 0.0
    speed = max(abs(measured), abs(previous))
    if stopping_distance(speed, deceleration, delay) >= distance - 1e-9:
        return 0.0
    return min(proposed, speed_for_distance(distance, deceleration, delay))


def validate_limits(velocity, angular_velocity, acceleration, angular_acceleration,
                    deceleration, delay, control_rate):
    for value in (velocity, angular_velocity, acceleration, angular_acceleration,
                  deceleration, control_rate):
        if not math.isfinite(value) or value <= 0:
            raise ValueError('Motion limits and control rate must be finite and positive')
    if not math.isfinite(delay) or delay < 0:
        raise ValueError('braking_delay_sec must be finite and nonnegative')


def driver_compatible(message, velocity, angular_velocity, acceleration,
                      angular_acceleration, deceleration):
    pairs = ((velocity, message.max_linear_velocity),
             (angular_velocity, message.max_angular_velocity),
             (acceleration, message.linear_acceleration),
             (angular_acceleration, message.angular_acceleration),
             (deceleration, message.linear_deceleration))
    return all(math.isfinite(actual) and actual > 0 and required <= actual + 1e-9
               for required, actual in pairs)
