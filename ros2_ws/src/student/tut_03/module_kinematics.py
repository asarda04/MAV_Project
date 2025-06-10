import numpy as np
import warnings

# Coordinate frames:
#
# 1. Earth Centered Inertial (ECI) frame - {i}
# 2. Earth Centered Earth Fixed (ECEF) frame - {e}
# 3. North-East-Down (NED) frame - {n}
# 4. BODY frame - {b}
#
# You may further assume the following:
# 
# eul = [phi, theta, psi] 
# 
# with the order of rotation being ZYX and
#   phi being angle about x-axis
#   theta being angle about y-axis
#   psi being angle about z-axis
#
# We assume ZYX rotation order in this file. For example,
# this means that the NED frame is rotated about 
# its z-axis by angle psi followed by a rotation about 
# the resultant y-axis by angle theta followed by a 
# rotation about the resultant x-axis by angle phi
# to reach the BODY frame.
#
# quat = [qw, qx, qy, qz] is considered to be an unit quaternion
#
# rotm = 3 x 3 matrix
# 
# The rotation matrix rotm when pre-multiplied by a vector
# in BODY frame will yield a vector in NED frame

r_e = 6378137
e = 0.0818
r_p =6356752.314245


def Smat(vec):
    """
    Calculate the skew-symmetric matrix for a given vector
    
    Args:
        vec (array): Vector to be converted to skew-symmetric matrix
        
    Returns:
        array: Skew-symmetric matrix
    """
    S = np.zeros((3,3));
    #===========================================================================
    # TODO: Implement the skew-symmetric matrix
    #===========================================================================
    # Write your code here

    S = np.array([[0, -1*vec[2], vec[1]],
                  [vec[2], 0, -1*vec[0]],
                  [-1*vec[1], vec[0], 0]])
    
    #pass
    #===========================================================================               
    return S

def eul_to_rotm(angles):
    """
    Convert Euler angles to rotation matrix
    
    Args:
        angles (array): Euler angles [phi, theta, psi]
        
    Returns:
        array: Rotation matrix
    """
    R = np.zeros((3,3))

    phi = angles[0]
    theta = angles[1]
    psi = angles[2]

    #===========================================================================
    # TODO: Implement the rotation matrix
    #===========================================================================
    # Write your code here

    cphi, sphi = np.cos(phi), np.sin(phi)
    ctheta, stheta = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)

    R = np.array([[ctheta * cpsi, sphi * stheta * cpsi - cphi * spsi, cphi * stheta * cpsi + sphi * spsi],
                  [ctheta * spsi, sphi * stheta * spsi + cphi * cpsi, cphi * stheta * spsi - sphi * cpsi],
                  [-stheta, sphi * ctheta, cphi * ctheta]])

    #pass # return R
    #===========================================================================
    
    return R

def rotm_to_eul(rotm, order='ZYX', deg=False):
    """
    Convert rotation matrix to Euler angles
    
    Args:
        rotm (array): Rotation matrix
        order (str): Order of Euler angles (default: 'ZYX')
        deg (bool): Return angles in degrees (default: False)
        
    Returns:
        array: Euler angles
    """

    # Initialize the Euler angles
    eul = np.zeros(3, dtype=float)

    # Check if the order is ZYX
    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    if order == 'ZYX':

        #===========================================================================
        # TODO: Implement the code to convert rotation matrix to Euler angles
        #===========================================================================

        # Write your code here

        theta = -1*np.arcsin(rotm[2,0])
        if np.abs(rotm[2, 0]) < 1:
            phi = np.arctan2(rotm[2, 1], rotm[2, 2])
            psi = np.arctan2(rotm[1, 0], rotm[0, 0])
        else:
            phi = np.arctan2(-1*rotm[1, 2], rotm[1, 1])
            psi = 0
    
        eul = [phi, theta, psi]

        #pass
        #===========================================================================

        if deg:
            eul = eul * 180 / np.pi       

    return eul

def eul_rate_matrix(angles, order='ZYX'):
    """
    Calculate the Euler rate matrix J2
    
    Args:
        angles (array): Euler angles [phi, theta, psi]
        
    Returns:
        array: Euler rate matrix
    """
    J2 = np.zeros((3,3))
    
    phi = angles[0]
    theta = angles[1]
    psi = angles[2]

    #===========================================================================
    # TODO: Implement the Euler rate matrix J2
    #===========================================================================
    # Write your code here

    J2 = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                   [0, np.cos(phi), -np.sin(phi)],
                   [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]])

    #J2 is same as T(THETA_nb)
    #===========================================================================
    
    return J2

def eul_rate(eul, w_nb_b, order='ZYX'):
    """
    Calculate the Euler rate
    
    Args:
        eul (array): Euler angles [phi, theta, psi]
        w_nb_b (array): Angular velocity in body frame [wx, wy, wz]
        order (str): Order of Euler angles (default: 'ZYX')
        
    Returns:
        array: Euler rate
    """
    deul = np.zeros(3, dtype=float)

    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    if order == 'ZYX':

        #===========================================================================
        # TODO: Implement the code to calculate the Euler rate
        #===========================================================================

        # Write your code here

        J2 = eul_rate_matrix(eul)
        deul = np.dot(J2, w_nb_b)
        # deul = d(THETA_nb)/dt
        #===========================================================================

    return deul

def eul_to_quat(eul, order='ZYX', deg=False):
    """
    Convert Euler angles to quaternion
    
    Args:
        eul (array): Euler angles [phi, theta, psi]
        
    Returns:
        array: Quaternion
    """
    quat = np.zeros(4, dtype=float)
    quat[0] = 1.0

    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':
        
        if deg:
            phi = eul[0] * np.pi / 180
            theta = eul[1] * np.pi / 180
            psi = eul[2] * np.pi / 180
        else:
            phi = eul[0]
            theta = eul[1]
            psi = eul[2]

        #===========================================================================
        # TODO: Implement the code to convert Euler angles to quaternion
        #===========================================================================

        # Write your code here

        cphi, sphi = np.cos(phi/2), np.sin(phi/2)
        ctheta, stheta = np.cos(theta/2), np.sin(theta/2)
        cpsi, spsi = np.cos(psi/2), np.sin(psi/2)
        
        w = cphi * ctheta * cpsi + sphi * stheta * spsi
        x = sphi * ctheta * cpsi - cphi * stheta * spsi
        y = cphi * stheta * cpsi + sphi * ctheta * spsi
        z = cphi * ctheta * spsi - sphi * stheta * cpsi

        quat = np.array([w,x,y,z])

        #pass
        #===========================================================================

        quat = quat / np.linalg.norm(quat)

    return quat

def quat_to_eul(quat, order='ZYX', deg=False):
    """
    Convert quaternion to Euler angles
    
    Args:
        quat (array): Quaternion
        order (str): Order of Euler angles (default: 'ZYX')
        deg (bool): Return angles in degrees (default: False)
        
    Returns:
        array: Euler angles
    """
    eul = np.zeros(3, dtype=float)
    
    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':
        qw = quat[0]
        qx = quat[1]
        qy = quat[2]
        qz = quat[3]

        #===========================================================================
        # TODO: Implement the code to convert quaternion to Euler angles
        #===========================================================================

        # Write your code here

        w, x, y, z = quat
    
        phi = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
        theta = np.arcsin(2 * (qw * qy - qz * qx))
        psi = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
        
        eul = np.array([phi, theta, psi])

        #pass
        #===========================================================================

    return eul

def quat_to_rotm(quat):    
    """
    Convert quaternion to rotation matrix
    
    Args:
        quat (array): Quaternion [qw, qx, qy, qz]
        
    Returns:
        array: Rotation matrix
    """
    rotm = np.eye(3, dtype=float)

    #===========================================================================
    # TODO: Implement the code to convert quaternion to rotation matrix
    #===========================================================================

    # Write your code here

    qw, qx, qy, qz = quat
    
    rotm = np.array([
        [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)]
    ])

    #pass
    #===========================================================================

    return rotm

def quat_multiply(q1, q2):
    """
    Multiply two quaternions
    
    Args:
        q1 (array): Quaternion 1
        q2 (array): Quaternion 2
        
    Returns:
        array: Resultant quaternion
    """
    w1 = q1[0]; x1 = q1[1]; y1 = q1[2]; z1 = q1[3]
    w2 = q2[0]; x2 = q2[1]; y2 = q2[2]; z2 = q2[3]

    q_prod = np.zeros(4, dtype=float)
    q_prod[0] = 1.0

    #===========================================================================
    # TODO: Implement the code to multiply two quaternions
    #===========================================================================

    # Write your code here

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    q_prod = np.array([w, x, y, z])


    #pass
    #===========================================================================

    return q_prod

def quat_conjugate(quat):
    """
    Calculate the conjugate of a quaternion
    
    Args:
        quat (array): Quaternion
        
    Returns:
        array: Conjugate of the quaternion
    """
    q_conj = np.zeros(4, dtype=float)
    q_conj[0] = 1.0
    
    #===========================================================================
    # TODO: Implement the code to calculate the conjugate of a quaternion
    #===========================================================================

    # Write your code here

    q_conj = np.array([quat[0], -1*quat[1], -1*quat[2], -1*quat[3]])

    #pass
    #===========================================================================

    return q_conj

def rotate_vec_by_quat(vec_a, q_a_b):
    """
    Rotate a vector by a quaternion
    
    Args:
        vec_a (array): Vector to be rotated
        q_a_b (array): Quaternion
        
    Returns:
        array: Rotated vector
    """
    vec_b = np.zeros(3, dtype=float)
    #===========================================================================
    # TODO: Implement the code to rotate a vector by a quaternion
    #===========================================================================

    # Write your code here

    rotm = quat_to_rotm(q_a_b)
    vec_b = np.dot(rotm, vec_a)

    #pass
    #===========================================================================

    return vec_b

def quat_rate_matrix(quat):
    """
    Calculate the quaternion rate matrix
    
    Args:
        quat (array): Quaternion
        
    Returns:
        array: Quaternion rate matrix
    """
    Tmat = np.zeros((4, 3))

    #===========================================================================
    # TODO: Implement the code to calculate the quaternion rate matrix
    #===========================================================================

    # Write your code here
    w,x,y,z = quat
    Tmat = 0.5 *  np.array([[-1*x, -1*y, -1*z],
                            [  w , -1*z,  y  ],
                            [  z ,   w , -1*x],
                            [-1*y,   x ,   w ]])

    #pass
    #===========================================================================

    return Tmat

def quat_rate(quat, w_nb_b):
    """
    Calculate the quaternion rate
    
    Args:
        quat (array): Quaternion
        w_nb_b (array): Angular velocity in body frame [wx, wy, wz]
        
    Returns:
        array: Quaternion rate
    """
    dquat = np.zeros(4, dtype=float)

    #===========================================================================
    # TODO: Implement the code to calculate the quaternion rate
    #===========================================================================

    # Write your code here

    Tmat = quat_rate_matrix(quat)
    dquat = np.dot(Tmat, w_nb_b)

    #dquat = d(q_b_n)
    #===========================================================================

    return dquat

def ssa(ang, deg=False):
    """
    Smallest Signed Angle (SSA) function to wrap angle to [-180, 180] degrees or 
    [-pi, pi] radians
    
    Args:
        ang (float): Angle to be wrapped
        deg (bool): Return angle in degrees (default: False)
        
    Returns:
        float: Wrapped angle
    """

    #===========================================================================
    # TODO: Implement the code to wrap angle to [-180, 180] degrees or 
    # [-pi, pi] radians
    #===========================================================================

    # Write your code here

    if deg:
        ang = ang % 360
        if ang > 180:
            ang = ang-360
    else:
        ang = ang % (2*np.pi)
        if ang > np.pi:
            ang = ang-2*np.pi

    #pass
    #===========================================================================

    return ang

def ned_to_llh(ned, llh0):
    """
    Convert NED coordinates to LLH coordinates
    
    Args:
        ned (array): NED coordinates [xn, yn, zn]
        llh0 (array): Initial LLH coordinates [mu0, l0, h0]
        
    Returns:
        array: LLH coordinates [mu, l, h]
    """
    xn = ned[0]
    yn = ned[1]
    zn = ned[2]

    mu0 = llh0[0] * np.pi / 180
    l0 = llh0[1] * np.pi / 180
    h0 = llh0[2]

    llh = np.zeros(3, dtype=float)

    #===========================================================================
    # TODO: Implement the code to convert NED coordinates to LLH coordinates
    #===========================================================================

    # Write your code here

    R_N = r_e / np.sqrt(1- (e*np.sin(mu0))**2)
    R_M = (r_e*(1-e**2)) / (1- (e*np.sin(mu0))**2)

    delta_mu = xn*np.arctan2(1, R_N)
    delta_l = yn*np.arctan2(1,R_M*np.cos(mu0))

    mu = ssa((mu0 + delta_mu))*180/np.pi
    l = ssa((l0 + delta_l))*180/np.pi
    h = h0 - zn

    llh = [mu, l, h]

    #pass
    #===========================================================================

    return llh

def llh_to_ned(llh, llh0):
    """
    Convert LLH coordinates to NED coordinates
    
    Args:
        llh (array): LLH coordinates [mu, l, h]
        llh0 (array): Initial LLH coordinates [mu0, l0, h0]
        
    Returns:
        array: NED coordinates [xn, yn, zn]
    """
    
    ned = np.zeros(3, dtype=float)
    #===========================================================================
    # TODO: Implement the code to convert LLH coordinates to NED coordinates
    #===========================================================================

    # Write your code here

    mu0 = llh0[0] * np.pi / 180
    l0 = llh0[1] * np.pi / 180
    h0 = llh0[2]
    
    mu = llh[0] * np.pi / 180
    l = llh[1] * np.pi / 180
    h = llh[2]

    R_N = r_e / np.sqrt(1- (e*np.sin(mu0))**2)
    R_M = (r_e*(1-e**2)) / (1- (e*np.sin(mu0))**2)

    delta_mu = mu - mu0
    delta_l = l - l0

    xn = delta_mu / np.arctan2(1, R_N)
    yn = delta_l / np.arctan2(1,R_M*np.cos(mu0))
    zn = h0 - h
    ned = [xn, yn, zn]

    #pass
    #===========================================================================

    return ned

def rotm_ned_to_ecef(llh):
    """
    Calculate the rotation matrix from NED to ECEF frame
    
    Args:
        llh (array): LLH coordinates [mu, l, h]
        
    Returns:
        array: Rotation matrix
    """
    mu = llh[0] * np.pi / 180
    l = llh[1] * np.pi / 180
    h = llh[2]

    rotm = np.zeros((3,3), dtype=float)

    #===========================================================================
    # TODO: Implement the code to calculate the rotation matrix from NED to ECEF frame
    #===========================================================================

    # Write your code here
    rotm = np.array([[-np.cos(l)*np.sin(mu),-np.sin(l),-np.cos(l)*np.cos(mu)],
                    [-np.sin(l)*np.sin(mu),np.cos(l),-np.sin(l)*np.cos(mu)],
                    [np.cos(mu),0,-np.sin(mu)]])
    #pass
    #===========================================================================

    return rotm

def ecef_to_llh(ecef): #llh0
    """
    Convert ECEF coordinates to LLH coordinates
    
    Args:
        ecef (array): ECEF coordinates [xe, ye, ze]
        llh0 (array): Initial LLH coordinates [mu0, l0, h0]
        
    Returns:
        array: LLH coordinates [mu, l, h]
    """
    llh = np.zeros(3, dtype=float)

    #===========================================================================
    # TODO: Implement the code to convert ECEF coordinates to LLH coordinates
    #===========================================================================

    # Write your code here
    xe = ecef[0]
    ye = ecef[1]
    ze = ecef[2]
    '''mu0 = llh0[0]
    l0 = llh0[1]
    h0 = llh0[2]''' 
    p = np.sqrt(xe**2 + ye**2)
    e = np.sqrt(1-(r_p/r_e)**2)
    mu0 = np.arctan2(ze,(p*(1-e**2)))

    l = np.arctan2(ye,xe)*180/np.pi

    # Find mu and h iterativley
    h,mu,iter = 0,0,0
    tol = 10**(-10)
    max_iter = 1000
    while True:
        N = r_e**2 /(np.sqrt(r_e**2 * np.cos(mu0)**2 + r_p**2 * np.sin(mu0)**2))
        h = (p/np.cos(mu0)) - N
        mu = np.arctan(ze/(p*(1- e**2 * (N/(N+h)))))

        if abs(mu-mu0) < tol:
            break
        else:
            mu0 = mu

        iter +=1
        if iter>=max_iter:
            print("h and mu not converged !!")
            break
    mu = mu*180/np.pi
    llh = np.array([mu,l,h])
    #pass
    #===========================================================================

    return llh

def llh_to_ecef(llh):
    """
    Convert LLH coordinates to ECEF coordinates
    
    Args:
        llh (array): LLH coordinates [mu, l, h]
        
    Returns:
        array: ECEF coordinates [xe, ye, ze]
    """
    ecef = np.zeros(3, dtype=float)

    #===========================================================================
    # TODO: Implement the code to convert LLH coordinates to ECEF coordinates
    #===========================================================================

    # Write your code here
    mu = llh[0]*np.pi/180
    l = llh[1]*np.pi/180
    h = llh[2]

    N = r_e**2 /(np.sqrt((r_e * np.cos(mu))**2 + (r_p * np.sin(mu))**2))

    xe = (N+h)*np.cos(mu)*np.cos(l)
    ye = (N+h)*np.cos(mu)*np.sin(l)
    ze = ((r_p**2 / r_e**2 )*N + h)*np.sin(mu)
    ecef = np.array([xe,ye,ze])


    #pass
    #===========================================================================

    return ecef
