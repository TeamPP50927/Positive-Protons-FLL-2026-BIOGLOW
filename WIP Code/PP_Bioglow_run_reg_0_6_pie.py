"""Positive Protons BIOGLOW - SPIKE Prime Pybricks single-file program.
Last Modified: Oct 29, 2026

LEFT: previous run. RIGHT: next run.
CENTER: execute selected run, then advance.
BLUETOOTH: stop and exit, including during motor movements.

Keep hub stationary while gyro initializes and before starting each run.
"""

from math import pi
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Button, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch


hub = PrimeHub()
hub.system.set_stop_button(Button.BLUETOOTH)

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.E, Direction.CLOCKWISE)

LAM = Motor(Port.D, Direction.CLOCKWISE)
RAM = Motor(Port.C, Direction.CLOCKWISE)

drive_base = DriveBase(left_motor, right_motor, 56, 97)


def initialize_robot():
    """Run once at startup while the robot is stationary."""
    drive_base.stop()
    LAM.stop()
    RAM.stop()

    wait(500)
    drive_base.use_gyro(True)

    while not hub.imu.ready():
        wait(100)

    drive_base.settings(
        straight_speed=300,
        straight_acceleration=500,
        turn_rate=150,
        turn_acceleration=300
    )

    drive_base.reset(0, 0)
    LAM.reset_angle(0)
    RAM.reset_angle(0)

    print('Robot ready - LEFT previous, RIGHT next, CENTER run, BLUETOOTH exit')


def prepare_run():
    """Zero distance/heading before each mission."""
    drive_base.stop()
    LAM.stop()
    RAM.stop()
    drive_base.reset(0, 0)


def gyro_straight(distance, speed=200):
    """Drive signed distance in mm at positive speed in mm/s."""
    if speed <= 0:
        raise ValueError('Speed must be positive')

    drive_base.settings(straight_speed=speed)
    drive_base.straight(distance)


def gyro_turn(angle, speed=100):
    """Turn signed angle in degrees at positive rate in degrees/s."""
    if speed <= 0:
        raise ValueError('Turn speed must be positive')

    drive_base.settings(turn_rate=speed)
    drive_base.turn(angle)


def gyro_curve(speed, radius, angle, kp=3.0):
    """
    Follow a circular arc using gyro heading feedback.

    speed: Forward speed in mm/s (positive)
    radius: Curve radius in mm (positive)
    angle: Signed turn angle in degrees
           Positive = right, negative = left
    kp: Proportional heading correction gain

    Uses DriveBase angle(), which is gyro-based
    because use_gyro(True) is enabled.

    The function preserves the existing distance
    and heading references.
    """

    if speed <= 0:
        raise ValueError('Curve speed must be positive')

    if radius <= 0:
        raise ValueError('Curve radius must be positive')

    if angle == 0:
        return

    start_distance = drive_base.distance()
    start_angle = drive_base.angle()

    direction = 1 if angle > 0 else -1

    # Total distance along the arc
    arc_length = radius * abs(angle) * pi / 180

    # Nominal turning rate in degrees per second
    base_turn_rate = direction * speed / radius * 180 / pi

    # Limit steering to a reasonable range
    max_turn_rate = 180

    # Timeout protection
    timer = StopWatch()
    estimated_time = arc_length / speed * 1000
    timeout = max(2000, int(estimated_time * 3 + 1000))

    try:
        while True:
            distance = drive_base.distance() - start_distance

            if distance >= arc_length:
                break

            if timer.time() >= timeout:
                print('Curve movement timeout')
                break

            # Expected heading along the arc
            target_angle = (
                start_angle
                + direction * (distance / radius) * 180 / pi
            )

            # Actual gyro heading
            actual_angle = drive_base.angle()

            # Heading error
            error = target_angle - actual_angle

            # Proportional gyro correction
            correction = kp * error

            turn_rate = base_turn_rate + correction

            # Limit maximum turning rate
            turn_rate = max(
                -max_turn_rate,
                min(max_turn_rate, turn_rate)
            )

            drive_base.drive(speed, turn_rate)

            wait(10)

    finally:
        drive_base.stop()

def gyro_pivot(angle, speed=150, pivot="left"):
    """
    Gyro-controlled pivot turn.

    Positive angle = Right turn
    Negative angle = Left turn

    pivot = "left"  -> Left wheel stationary
    pivot = "right" -> Right wheel stationary
    """

    if speed <= 0:
        raise ValueError("Speed must be positive")

    if pivot not in ("left", "right"):
        raise ValueError("Invalid pivot wheel")

    if angle == 0:
        return

    drive_base.stop()

    start_angle = drive_base.angle()
    target_angle = start_angle + angle

    direction = 1 if angle > 0 else -1

    timer = StopWatch()
    timeout = 5000

    try:
        while True:

            current_angle = drive_base.angle()
            error = target_angle - current_angle

            # Stop when target reached or crossed
            if direction * error <= 1.5:
                break

            if timer.time() > timeout:
                print("Pivot timeout")
                break

            # Slow down approaching target
            motor_speed = min(
                speed,
                max(30, abs(error) * 3)
            )

            if pivot == "left":
                left_motor.brake()

                # Right wheel moves backward for right turn
                right_motor.run(-direction * motor_speed)

            else:
                right_motor.brake()

                # Left wheel moves forward for right turn
                left_motor.run(direction * motor_speed)

            wait(10)

    finally:
        left_motor.brake()
        right_motor.brake()

    print("Target:", target_angle)
    print("Actual:", drive_base.angle())

def gyro_straight_stall(distance, speed=50, timeout=3000):
    """Drive until completed, stalled, or timed out."""

    drive_base.settings(straight_speed=abs(speed))
    drive_base.straight(distance, wait=False)

    timer = StopWatch()

    while not drive_base.done():

        if drive_base.stalled():
            print("Stall detected")
            drive_base.stop()
            return True

        if timer.time() >= timeout:
            print("Movement timeout")
            drive_base.stop()
            return False

        wait(10)

    print("Movement completed")
    return False


def attachment_move(motor, angle, speed=300):
    """Move attachment relative to its current angle."""
    motor.run_angle(speed, angle, then=Stop.HOLD, wait=True)


# --------------------------------------------------
# MISSION RUNS
# --------------------------------------------------

def run_0():
    #Money Run
    RAM.reset_angle(0)
    LAM.reset_angle(0)
    gyro_straight(640,700)
    gyro_curve(50,60,45)
    gyro_straight_stall(120,50)
    LAM.run_angle(250,600)
    LAM.run_angle(250,-600)
    #gyro_turn(46,47)
    #gyro_straight(105,700)
    #LAM.run_target(500,600)
    #gyro_straight(-70,700)
    #LAM.run_angle(500,-300)
    #completed Keystone
    gyro_straight(-290,700)
    RAM.run_target(400,-300)
    gyro_turn(-40,51)
    #Completed Into the Past
    gyro_straight(130,50)
    RAM.run_target(600,200)
    gyro_straight(-20,100)
    RAM.run_target(250,-100)
    #Completed Fungus
    gyro_turn(75,50)
    #gyro_turn(70,50)
    gyro_straight(230,200)
    RAM.run_angle(200,180)
    RAM.run_angle(200,-180)
    gyro_turn(10,50)
    #Completed Forest Elder
    gyro_straight(-450,500)
    RAM.run_angle(200,200)
    gyro_turn(-45,50)
    gyro_straight(200,50)    
    gyro_straight(-200,500)
    #Leaf Cutter Frenzy complete  
    gyro_turn(45,600) 
    gyro_straight(430,700) 
    gyro_turn(-90,600)
    gyro_straight(-800,700) 

def run_1():
    gyro_straight(350,800)
    RAM.run_angle(-200,500)
    gyro_straight(50,50)
    pass
def run_2():
    #Biocentric Arcitecture
    LAM.reset_angle(0)
    gyro_straight(195, 700)
    LAM.run_target(700, 3200, wait=False)
    gyro_straight(90, 20)
    wait(1000)
    gyro_straight(-100, 20)
    gyro_straight(30, 20)
    LAM.run_target(900, 1500)
    LAM.run_target(900, 0, wait=False)
    gyro_straight(-300, 700)

def run_3():
    pass    
   
def run_4():
    #Drone Survey
    RAM.reset_angle(0)
    gyro_straight(730, 150)
    gyro_straight(-70,50)
    gyro_turn(-55, 45)
    #gyro_straight(-15,50)
    #gyro_turn(15,50)
    gyro_turn(45, 45)
    gyro_straight(-620, 850)

    #gyro_straight(60, 500)
    #RAM.run_target(650, 1980)
    #gyro_straight(-100, 500)
    #gyro_straight(40, 300)
    #RAM.run_target(550, -1980)
    #gyro_straight(-320, 500)

def run_5():
    #Exploding Seeds
    LAM.reset_angle(0)
    gyro_straight(150, 350)
    gyro_turn(48, 45)
    LAM.run_target(350, 300)
    gyro_straight(270, 850)
    LAM.run_target(250, 50)
    LAM.run_target(150, -100, wait=False)
    gyro_straight(-200, 100)
    gyro_straight(-200, 300)

def run_6():
    #Flip the Rock and Reaching Roots
    RAM.reset_angle(0)
    LAM.reset_angle(0)
    gyro_straight(400, 250)
    gyro_straight(-250, 200)
    #Completed Flip the Rock
    RAM.run_target(150, -250,wait=False)
    gyro_turn(45, 37)
    LAM.run_target(100, 160,wait=False)
    gyro_straight(250, 900)
    gyro_turn(-43, 37)
    gyro_straight(420,900)
    #Arrived at Reaching Roots
    RAM.run_angle(150, 100)
    LAM.run_target(100,0,wait=False)
    #gyro_straight(-20,50)
    #Got 10 points
    gyro_straight(-30,150)
    RAM.run_angle(1850,200)
    #gyro_straight(50, 50)
    RAM.run_target(850,0)
    RAM.run_target(850,-250)
    gyro_straight(30,50)
    RAM.run_target(850,0)
    #Got 20 points
    gyro_straight(-900, 900)   

def run_7():
    #The Big Tree   
    LAM.reset_angle(0)
    gyro_straight(300, 300)
    gyro_turn(40, 45)
    gyro_straight(490, 450)
    gyro_turn(-40, 45)
    RAM.run_target(100, -165,wait=False)
    gyro_straight(-100, 100)
    gyro_straight(96, 100)
    gyro_straight(20, 10)
    LAM.run_angle(2000, 7000)
    gyro_straight(30, 100)
    #Finished research platform
    RAM.run_angle(50, 150)
    gyro_straight(-100, 70)
    #Finished tangled
def run_7(): 
    RAM.run_target(850,-250)
    gyro_straight(50,50)
    RAM.run_target(850,0)

def run_8():
    print("Starting Run 8")

    drive_base.stop()

    # Test right motor backward
    right_motor.run_angle(100, -180)

    wait(500)

    # Test left motor backward
    left_motor.run_angle(100, -180)

    print("Motor test complete")
    
RUNS = (run_0, run_1, run_2, run_3, run_4, run_5, run_6, run_7, run_8)


# --------------------------------------------------
# BUTTON CONTROL
# --------------------------------------------------

def wait_for_buttons_release():
    """Wait until all hub buttons have been released."""
    while hub.buttons.pressed():
        wait(20)


def main():
    initialize_robot()

    selected = 0
    hub.display.number(selected)

    wait_for_buttons_release()

    while True:
        buttons = hub.buttons.pressed()

        if Button.LEFT in buttons:
            wait_for_buttons_release()
            selected = (selected - 1) % len(RUNS)
            hub.display.number(selected)
            print('Selected run:', selected)

        elif Button.RIGHT in buttons:
            wait_for_buttons_release()
            selected = (selected + 1) % len(RUNS)
            hub.display.number(selected)
            print('Selected run:', selected)

        elif Button.CENTER in buttons:
            wait_for_buttons_release()
            print('Starting run:', selected)

            try:
                prepare_run()
                RUNS[selected]()

            except Exception as exc:
                drive_base.stop()
                LAM.stop()
                RAM.stop()

                hub.display.char('E')
                print('Run failed:', exc)
                wait(1500)

            else:
                drive_base.stop()
                print('Completed run:', selected)

                selected = (selected + 1) % len(RUNS)

            hub.display.number(selected)

        wait(20)


main()
