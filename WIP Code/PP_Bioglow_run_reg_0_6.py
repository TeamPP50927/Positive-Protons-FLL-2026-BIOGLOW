"""Positive Protons BIOGLOW - SPIKE Prime Pybricks single-file program.
Last Modifed : Oct 25 2026

LEFT: previous run. RIGHT: next run.
CENTER: execute selected run, then advance.
BLUETOOTH: stop and exit, including during motor movements.
Pybricks editor Stop also remains available.
Keep hub stationary while gyro initializes and before starting each run.
"""
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Button, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
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
    drive_base.settings(straight_speed=300,
                        straight_acceleration=500,
                        turn_rate=150,
                        turn_acceleration=300)
    drive_base.reset(0, 0)
    LAM.reset_angle(0)
    RAM.reset_angle(0)
    print('Robot ready - LEFT previous, RIGHT next, CENTER run, BLUETOOTH exit')


def prepare_run():
    """Zero distance/heading before each mission; do not reinitialize gyro."""
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

def gyro_straight_stall(distance, speed=50, timeout=3000):
    # Set driving speed
    drive_base.settings(straight_speed=abs(speed))

    # Start movement without waiting
    drive_base.straight(distance, wait=False)

    timer = StopWatch()

    while not drive_base.done():
        # Stop if robot is stalled
        if drive_base.stalled():
            print("Stall detected")
            drive_base.stop()
            return True

        # Safety timeout
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


# Example missions. Replace with your team's real movements.
def run_0():
    gyro_straight(760, 200)
    gyro_straight(-45, 40)
    gyro_turn(-60,45)
    gyro_straight(-10, 40)
    gyro_turn(60,45)
    gyro_straight(-540, 500)
    gyro_turn(-90,45)
    gyro_straight_stall(-100, 50)
    #gyro_straight(-100, 50)
    #attachment_move(LAM, 5560, 960)
    #gyro_straight(30, 550)
    #gyro_straight(-80, 150)
    #attachment_move(LAM, -5560, 960)


def run_1():
    LAM.reset_angle(0)
    gyro_straight(150, 350)
    gyro_turn(48, 45)
    LAM.run_target(350, 300)
    gyro_straight(270, 850)
    LAM.run_target(250, 50)
    LAM.run_target(150, -100, wait=False)
    gyro_straight(-200, 100)
    gyro_straight(-200,300)


def run_2():
    gyro_straight(400, 250)
    gyro_straight(-200, 200)
    gyro_turn(45, 45)
    gyro_straight(500, 500)


def run_3():
    RAM.reset_angle(0)
    gyro_straight(60,500)
    RAM.run_target(650,1980)
    gyro_straight(-100,500)
    gyro_straight(40,300)
    RAM.run_target(550,-1980)
    gyro_straight(-320,500)

def run_4():
    RAM.reset_angle(0)
    gyro_straight(190,700)
    RAM.run_target(450,2700,wait=False)
    gyro_straight(90,20)
    wait(1000)
    gyro_straight(-100,20)
    gyro_straight(30,20)
    RAM.run_target(450,2200)
    RAM.run_target(900,0,wait=False)
    gyro_straight(-300,700)


def run_5():
    pass


RUNS = (run_0, run_1, run_2, run_3, run_4, run_5)


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
