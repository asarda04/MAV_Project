from typing import Dict, Optional, List
import numpy as np
from scipy.integrate import solve_ivp

import sys
sys.path.append('/workspaces/mavlab/')
from ros2_ws.src.student.tut_03.module_kinematics import Smat, eul_to_rotm, eul_rate_matrix
import ros2_ws.src.student.tut_03.module_control as con
from ros2_ws.src.student.tut_03.module_dynamics import coriolis_matrix

class Vessel:
    """A class representing a marine vessel with its dynamics.
    
    Attributes:
        g (float): Gravitational acceleration
        rho (float): Water density
        L (float): Vessel length
        U (float): Forward speed
        mass (float): Vessel mass
        cog (np.ndarray): Center of gravity coordinates
        ...
    
    Example:
        >>> vessel_params = {...}  # vessel parameters
        >>> hydro_data = {...}     # hydrodynamic coefficients
        >>> vessel = Vessel(vessel_params, hydro_data)
        >>> vessel.simulate()
    """
    
    def __init__(self, vessel_params: Dict, hydrodynamic_data: Dict, ros_flag: bool = False):
        """Initialize vessel with parameters and hydrodynamic data.
        
        Args:
            vessel_params: Dictionary containing vessel parameters
            hydrodynamic_data: Dictionary containing hydrodynamic coefficients
        """

        # ROS flag
        self.ros_flag = ros_flag
 
        # Base vessel parameters
        self.g = vessel_params['g']
        self.rho = vessel_params['rho']     
        self.L = vessel_params['L']
        self.U = vessel_params['U']
        
        # Mass parameters
        self.mass = vessel_params['mass']
        self.cog = vessel_params['cog']
        self.gyration = vessel_params['gyration']

        # Generate mass matrix
        self._generate_mass_matrix()

        # Dimensionalization flag
        self.dim_flag = hydrodynamic_data.get('dim_flag', False)

        # Extract hydrodynamic coefficients
        self.X_u = hydrodynamic_data.get('X_u', 0.0)
        self.X_ud = hydrodynamic_data.get('X_ud', 0.0)
        self.Y_v = hydrodynamic_data.get('Y_v', 0.0)
        self.Y_vd = hydrodynamic_data.get('Y_vd', 0.0)
        self.Y_r = hydrodynamic_data.get('Y_r', 0.0)
        self.Y_rd = hydrodynamic_data.get('Y_rd', 0.0)
        self.N_v = hydrodynamic_data.get('N_v', 0.0)
        self.N_vd = hydrodynamic_data.get('N_vd', 0.0)
        self.N_r = hydrodynamic_data.get('N_r', 0.0)
        self.N_rd = hydrodynamic_data.get('N_rd', 0.0)
        self.Y_delta = hydrodynamic_data.get('Y_delta', 0.0)
        self.N_delta = hydrodynamic_data.get('N_delta', 0.0)

        # Dimensionalize if needed
        if not self.dim_flag:
            self._dimensionalize_coefficients(self.rho, self.L, self.U)
        
        # Control and simulation parameters
        self.T_delta = vessel_params['T_delta']
        self.current_state = np.concatenate([
            vessel_params['initial_velocity'],
            vessel_params['initial_position'],
            np.array([0.0])
        ])
        self.initial_state = self.current_state
                
        self.Tmax = vessel_params['sim_time']
        self.dt = vessel_params['time_step']
        self.t = 0.0
        self.control_type = vessel_params['control_type']
        self.delta_c = 0.0

        # Initialize history with pre-allocated array based on simulation time
        num_timesteps = int(self.Tmax / self.dt) + 2
        self.history = np.zeros((num_timesteps, 13))  # 13 state variables
        self.history[0, :] = self.current_state  # Store initial state
        self.time_index = 1  # Index to track position in history array

    def _dimensionalize_coefficients(self, rho, L, U):
        #====================================================================================
        # TODO: Code this function to dimenionalize non-dimensional hydrodynamic derivatives
        #====================================================================================
        
        # Write your code here
        # This part is redundant as this task is done below already!
        #pass
        #====================================================================================

        """Convert non-dimensional coefficients to dimensional form"""
        # Surge coefficients
        self.X_u *= 0.5 * rho * L**2 * U
        self.X_ud *= 0.5 * rho * L**3
        
        # Sway coefficients 
        self.Y_v *= 0.5 * rho * L**2 * U
        self.Y_vd *= 0.5 * rho * L**3
        self.Y_r *= 0.5 * rho * L**3 * U
        self.Y_rd *= 0.5 * rho * L**4

        # Yaw coefficients
        self.N_v *= 0.5 * rho * L**3 * U
        self.N_vd *= 0.5 * rho * L**4
        self.N_r *= 0.5 * rho * L**4 * U
        self.N_rd *= 0.5 * rho * L**5

        # Rudder coefficients
        self.Y_delta *= 0.5 * rho * L**2 * U**2
        self.N_delta *= 0.5 * rho * L**3 * U**2

        #====================================================================================
    
    def _generate_mass_matrix(self):
        #=======================================================================================
        # TODO: Code this function to generate the mass matrix and store it in self.mass_matrix
        #=======================================================================================
        self.mass_matrix = np.zeros((6,6))

        # Write your code here
        M11 = self.mass*(np.eye(3))
        M12 = -self.mass*Smat(self.cog)
        M21 = -M12
        M22 = np.zeros((3,3))

        M22[0,0] = self.mass*(self.gyration[0]**2)
        M22[1,1] = self.mass*(self.gyration[1]**2)
        M22[2,2] = self.mass*(self.gyration[2]**2)

        M22 = M22 - self.mass*np.dot((Smat(self.cog)),Smat(self.cog))

        M_top = np.concatenate((M11, M12), axis=1)

        # Concatenate the bottom row (M21 and M22)
        M_bottom = np.concatenate((M21, M22), axis=1)

        # Concatenate the top and bottom rows vertically
        self.mass_matrix = np.concatenate((M_top, M_bottom), axis=0)


        #pass
        #=======================================================================================

    def vessel_ode(self, t, state):
        # Extract velocities [u, v, w, p, q, r]
        vel = state[0:6]
        u, v, w, p, q, r = vel
        
        # Extract positions [x, y, z]
        pos = state[6:9] 
        x, y, z = pos
        
        # Extract euler angles [phi, theta, psi]
        angles = state[9:12]
        phi, theta, psi = angles

        # Extract rudder angle
        delta = state[12]

        # Commanded rudder angle
        #if not self.ros_flag:
        #    if self.control_type == 'fixed_rudder':
        #        self.delta_c = con.fixed_rudder(t, state)
        #    elif self.control_type == 'switching_rudder':
        #        self.delta_c = con.switching_rudder(t, state)
        #    else:
        #        raise ValueError(f"Invalid control type: {self.control_type}")
        
        # Initialize state_dot
        state_dot = np.zeros(13)

        # Calculate the hydrodynamic forces and moments
        F_hyd = self.hydrodynamic_forces(vel)

        # Calculate the control forces and moments
        F_control = self.control_forces(delta)

        # Calculate the mass matrices
        M_RB = self.mass_matrix
        M_A = self.added_mass_matrix()

        # Calculate the total mass matrix
        M = M_RB + M_A

        # Calculate the Coriolis force
        #F_coriolis = self.coriolis_forces(M, state)
        
        # Calculate the total force vector
        F = F_hyd + F_control # + F_coriolis

        #========================================================================
        # TODO: Calculate the state derivatives and store them to state_dot[0:12]
        #========================================================================

        # Write your code here
        state_dot[0:6] = np.dot(np.linalg.inv(M),F)
        state_dot[6:9] = np.dot(eul_to_rotm(angles),vel[0:3])
        state_dot[9:12] = np.dot(eul_rate_matrix(angles),vel[3:6])
        


        #========================================================================
        
        # Calculate the rudder dynamics
        state_dot[12] = (self.delta_c - delta) / self.T_delta

        # Set the derivatives of vertical modes to zero
        #state_dot[[2, 3, 4, 8, 9, 10]] = 0.0
            
        return state_dot
    
    def added_mass_matrix(self):
        """Calculate the added mass matrix"""
        # Initialize added mass matrix
        M_A = np.zeros((6, 6))

        # Calculate added mass matrix
        M_A[0, 0] = -self.X_ud
        M_A[1, 1] = -self.Y_vd
        M_A[1, 5] = -self.Y_rd
        M_A[5, 1] = -self.N_vd
        M_A[5, 5] = -self.N_rd

        return M_A
    
    def coriolis_forces(self, M, state):
        """Calculate the Coriolis force
        
        Args:
            M (array): Mass matrix (6x6)
            state (array): State vector (12x1)
            
        Returns:
            array: Coriolis force vector (6x1)
        """
        vel = state[0:6]
        C = coriolis_matrix(M, vel)
        F_coriolis = -(C @ vel)
        return F_coriolis
    
    def hydrodynamic_forces(self, vel):
        """Calculate hydrodynamic forces and moments
        
        Args:
            vel (array): Velocity vector [u, v, w, p, q, r]
            
        Returns:
            array: Forces and moments vector [X, Y, Z, K, M, N]
        """
        # Extract velocities
        u, v, w, p, q, r = vel
        
        # Initialize forces vector
        F = np.zeros(6)
        
        # Surge force (X)
        F[0] = self.X_u * (u - self.U)
        
        # Sway force (Y)
        F[1] = (self.Y_v * v + 
                self.Y_r * r)
        
        # Yaw moment (N) 
        F[5] = (self.N_v * v +
                self.N_r * r)
                
        return F

    def control_forces(self, delta):
        """Calculate control forces and moments
        
        Args:
            delta (float): Rudder angle in radians
            
        Returns:
            array: Forces and moments vector [X, Y, Z, K, M, N]
        """
        # Initialize control forces vector
        tau = np.zeros(6)
        
        # Calculate control forces
        tau[1] = self.Y_delta * delta
        tau[5] = self.N_delta * delta
        
        return tau
    
    def step(self):
        """Step the vessel forward in time"""        
        sol = solve_ivp(self.vessel_ode, [self.t, self.t + self.dt], self.current_state, method='RK45')        
        self.current_state = sol.y[:, -1]
        self.t = sol.t[-1]

        # Store current state in history
        if not self.ros_flag:
            self.history[self.time_index, :] = self.current_state
            self.time_index += 1

    def reset(self):
        """Reset the vessel to the initial state"""
        self.current_state = self.initial_state
        self.t = 0.0
    
    def simulate(self):
        """Simulate the vessel"""
        
        self.reset()
        while self.t < self.Tmax:
            self.step()
        
        # Trim history array to actual size
        self.history = self.history[:self.time_index, :]
