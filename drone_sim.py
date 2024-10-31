import numpy as np
import math as mt
import simulation


def main(x,t):
    sim = simulation.Simulation()
    x = np.ones((12,1)) # placeholder state vector at time t
    
    dx = np.array((12,1))
    
    u_b_mps = x[0] # axial velocity (forward) of CM wrt inertial CS
    v_b_mps = x[1] # lateral velocity of CM wrt inertial CS
    w_b_mps = x[2] # vertical velocity of CM wrt inertial CS
    p_b_rps = x[3] # roll angular velocity of body
    q_b_rps = x[4] # pitch angular velocity of body
    r_b_rps = x[5] # yaw angular velocity of body

    vel_b_mps = [u_b_mps,v_b_mps,w_b_mps]
    w_b_rps = [p_b_rps,q_b_rps,r_b_rps]


    phi_rad = x[6] # roll angle
    theta_rad = x[7] # pitch angle
    psi_rad =  x[8] # yaw angle
    p1_n_m = x[9] # x position of aircraft in NED CS
    p2_n_m = x[10] # y position of aircraft in NED CS
    p3_n_m = x[11] # z position of aircraft in NED CS
    J_b_kgm2 = [[0,0,0],[0,0,0],[0,0,0]]
   
    g_n = [0 ,0 , 9.8] # gravity
    F_b = np.array([0,0,0]) #F_xb, F_yb, F_zb
    m_kg = 5 #mass 
    
    # mass and moments of inertia
    Jxz_b_kgm2 = []
    Jxx_b_kgm2 = []
    Jyy_b_kgm2 = []
    Jzz_b_kgm2 = []

    dx[0] = 1/m_kg * F_b[0]
    g_body_mps2 = np.dot(sim.computeRotationMatrix(phi_rad,theta_rad,psi_rad),g_n) #gravity in the body coordinate system

    #compute acceleration
    dx[0:3] = sim.returnAcceleration(F_b,m_kg,g_n,w_b_rps, vel_b_mps,phi_rad,theta_rad,psi_rad)
    #compute angular acceleration
    dx[3:6] = sim.returnAngAcceleration(J_b_kgm2,m_kg,w_b_rps,vel_b_mps) 
    return dx

