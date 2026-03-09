from pymavlink import mavutil

mav = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
mav.wait_heartbeat()

# spin motor 1 at 10% for 3 seconds
mav.mav.command_long_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
    0,
    1,      # motor number (1-indexed)
    0,      # MOTOR_TEST_THROTTLE_PERCENT
    10,     # throttle percent
    3,      # timeout seconds
    0, 0, 0
)