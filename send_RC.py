# Import mavutil
from pymavlink import mavutil
# Create the connection
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if id < 1:
        print("Channel does not exist.")
        return

    # We only have 8 channels
    #http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE
    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm
        master.mav.rc_channels_override_send(
            master.target_system,                # target_system
            master.target_component,             # target_component
            *rc_channel_values)                  # RC channel list, in microseconds.
while True:
    # Set some pitch
    set_rc_channel_pwm(1, 1500)
    # Set some roll
    set_rc_channel_pwm(2, 1500)
    # Set some throttle
    set_rc_channel_pwm(3, 1500)
    # Set some yaw
    set_rc_channel_pwm(4, 1500)
    # Set some forward
    set_rc_channel_pwm(5, 1550)
    # Set some Lateral
    set_rc_channel_pwm(6, 1500)

# The camera pwm value is the servo speed
# and not the servo position
set_rc_channel_pwm(8, 1500)