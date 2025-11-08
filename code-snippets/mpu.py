import smbus
import time
import math

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

# Setup I2C
bus = smbus.SMBus(1)
Device_Address = 0x68

# Wake up MPU6050
bus.write_byte_data(Device_Address, PWR_MGMT_1, 0)

# Thresholds
ACCEL_THRESHOLD   = 0.1   # Difference in acc, to count something as movement
TILT_THRESHOLD    = 5     # Min angle to count as titlting movement
DRINK_ANGLE_MIN   = 40    # Min angle to count a drinking event
DRINK_ANGLE_MAX   = 80    # Max angle to count a drinking event
DRINK_HOLD_TIME   = 1.0   # seconds
STILL_TIME        = 2.0   # seconds of no movement
PLACED_ANGLE_MAX  = 15    # degrees (bottle nearly flat)
PLACED_CONFIRM    = 1.0   # seconds to confirm bottle placed

def read_raw_accel(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low  = bus.read_byte_data(Device_Address, addr+1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value


def read_accel():
    ax = read_raw_accel(ACCEL_XOUT_H) / 16384.0
    ay = read_raw_accel(ACCEL_YOUT_H) / 16384.0
    az = read_raw_accel(ACCEL_ZOUT_H) / 16384.0
    return ax, ay, az


def tilt_angles(ax, ay, az):
    pitch = math.atan2(ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    return pitch


# State tracking
prev_ax, prev_ay, prev_az = read_accel() # Previous accel values for all axes
prev_pitch = tilt_angles(prev_ax, prev_ay, prev_az) # Previous pitch (tilt angle)
bottle_picked = False
drinking = False
drink_start_time = None # Timestamp when bottle was within drink angle range
last_motion_time = time.time() # Previous time that the bottle moved
placed_candidate_time = None # Timestamp of when bottle met conditions of being still and flat

while True:
    ax, ay, az = read_accel()
    pitch = tilt_angles(ax, ay, az)
    now = time.time()

    accel_change = (
        abs(ax - prev_ax) > ACCEL_THRESHOLD or
        abs(ay - prev_ay) > ACCEL_THRESHOLD or
        abs(az - prev_az) > ACCEL_THRESHOLD
    )
    tilt_change = abs(pitch - prev_pitch) > TILT_THRESHOLD

    # Movement detection
    if accel_change or tilt_change:
        last_motion_time = now
        placed_candidate_time = None  # reset placement candidate
        if not bottle_picked:
            bottle_picked = True
            print("Bottle is picked up")

    # Check for possible "placed down" condition
    if bottle_picked and now - last_motion_time > STILL_TIME and abs(pitch) < PLACED_ANGLE_MAX:
        # first time meeting placement condition
        if placed_candidate_time is None:
            placed_candidate_time = now
        # confirm if it stays down long enough
        elif now - placed_candidate_time >= PLACED_CONFIRM:
            bottle_picked = False
            drinking = False
            drink_start_time = None
            placed_candidate_time = None
            print("Bottle is placed down")
    else:
        placed_candidate_time = None  # reset if bottle moves again

    # Drinking detection
    if bottle_picked:
        if DRINK_ANGLE_MIN <= abs(pitch) <= DRINK_ANGLE_MAX:
            if not drinking:
                if drink_start_time is None:
                    drink_start_time = now
                elif now - drink_start_time >= DRINK_HOLD_TIME:
                    drinking = True
                    print("Drinking detected!")
        else:
            drink_start_time = None
            if drinking:
                drinking = False
                print("Stopped drinking")

    # Update previous readings
    prev_ax, prev_ay, prev_az = ax, ay, az
    prev_pitch = pitch
    time.sleep(0.1)

