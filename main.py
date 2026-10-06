"""Positive Protons FLL BIOGLOW main program."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

import Run_0
import Run_1
import Run_2
import Run_3
import Run_4
import Run_5

hub = PrimeHub()

hub.system.set_stop_button((Button.CENTER, Button.BLUETOOTH))

leftmotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
rightmotor = Motor(Port.E, Direction.CLOCKWISE)

drive_base = DriveBase(leftmotor, rightmotor, 56, 97)


def wait_for_center_button():

    while Button.CENTER in hub.buttons.pressed():
        wait(20)

    while Button.CENTER not in hub.buttons.pressed():
        wait(20)

    while Button.CENTER in hub.buttons.pressed():
        wait(20)


current_run = 0

while current_run <= 6:

    hub.display.number(current_run)

    wait_for_center_button()

    if current_run == 0:
        Run_0.main(drive_base)

    elif current_run == 1:
        Run_1.main(drive_base)

    elif current_run == 2:
        Run_2.main(drive_base)

    elif current_run == 3:
        Run_3.main(drive_base)

    elif current_run == 4:
        Run_4.main(drive_base)
    
    elif current_run == 5:
        Run_5.main(drive_base)
        
    elif current_run == 6:
        Run_6.main(drive_base)

    current_run += 1

hub.display.text("DONE")
