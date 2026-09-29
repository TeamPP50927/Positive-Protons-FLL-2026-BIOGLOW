"""Positive Protons FLL BIOGLOW main program."""

from pybricks.hubs import PrimeHub

hub = PrimeHub()


def run_selected(number):
    """Import and execute the selected run."""
    if number == 1:
        from runs.Run_01 import main
    elif number == 2:
        from runs.Run_02 import main
    elif number == 3:
        from runs.Run_03 import main
    elif number == 4:
        from runs.Run_04 import main
    elif number == 5:
        from runs.Run_05 import main
    elif number == 6:
        from runs.Run_06 import main
    else:
        return
    main()


# TODO: Add the team's button/menu logic here after the run programs are tested.
if __name__ == "__main__":
    run_selected(1)
