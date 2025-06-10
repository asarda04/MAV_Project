import numpy as np

import sys
sys.path.append('/workspaces/mavlab/')
from ros2_ws.src.student.tut_03.module_kinematics import Smat

def coriolis_matrix(M, vel):
    """
    Calculate the Coriolis matrix parametrized in terms of the mass 
    matrix M and the state vector.
    
    Args:
        M (array): Mass matrix (6x6)
        vel (array): Velocity vector (6x1)
        
    Returns:
        array: Coriolis matrix (6x6)
    """
    C = np.zeros((6, 6))

    #===========================================================================
    # TODO: Implement the code to calculate the Coriolis matrix
    #===========================================================================

    # Write your code here

    M11 = M[0:3,0:3]
    M12 = M[0:3,3:6]
    M21 = M[3:6,0:3]
    M22 = M[3:6,3:6]

    V1 = vel[0:3]
    V2 = vel[3:6]
    
    C12 = -Smat(M11.dot(V1) + M12.dot(V2))
    C21 = -Smat(M11.dot(V1) + M12.dot(V2))
    C22 = -Smat(M21.dot(V1) + M22.dot(V2))
    C11 = np.zeros((3,3))

    # Concatenate the top row (C11 and C12)
    C_top = np.concatenate((C11, C12), axis=1)

    # Concatenate the bottom row (C21 and C22)
    C_bottom = np.concatenate((C21, C22), axis=1)

    # Concatenate the top and bottom rows vertically
    C = np.concatenate((C_top, C_bottom), axis=0)


    #pass
    #===========================================================================

    return C