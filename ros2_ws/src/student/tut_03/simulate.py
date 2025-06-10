from typing import NoReturn
import numpy as np

import sys
sys.path.append('/workspaces/mavlab/')

from ros2_ws.src.student.tut_03.read_input import read_input
from ros2_ws.src.student.tut_03.class_vessel import Vessel
from ros2_ws.src.student.tut_03.plot_results import plot_simulation_results

def simulate() -> NoReturn:
    """Run vessel simulation using parameters from input files.
    
    Reads vessel and hydrodynamic parameters, creates a Vessel instance,
    and runs the simulation.
    """
    vessel_params, hydrodynamic_data = read_input()
    vessel = Vessel(vessel_params, hydrodynamic_data)
    vessel.simulate()
    plot_simulation_results(vessel)

if __name__ == "__main__":
    simulate()

