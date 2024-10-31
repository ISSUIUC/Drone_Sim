
import numpy as np
import math as mt
class Simulation:
    
    
    def __init__(self):
        pass

    #matrix to convert from ned coordinates to body
    def computeRotationMatrix(self,roll,pitch, yaw):
        mat = [[mt.cos(pitch)*mt.cos(yaw), mt.cos(pitch) * mt.cos(yaw), -mt.sin(pitch)], 
               [(-mt.cos(roll) * mt.sin(yaw)) + mt.sin(roll) * mt.sin(pitch) * mt.cos(yaw),( mt.cos(roll) * mt.sin(yaw)) + mt.sin(roll) * mt.sin(pitch) * mt.sin(yaw), mt.sin(roll) *mt.cos(pitch) ],
               [mt.sin(roll) * mt.sin(yaw) + mt.cos(roll)*mt.sin(pitch)*mt.cos(yaw), -mt.sin(roll)*mt.cos(yaw) + mt.cos(roll)*mt.sin(pitch)*mt.sin(yaw) , mt.cos(roll)*mt.cos(pitch)]]
        return mat
    


    
    #acceleration of the body 
    def returnAcceleration(self, F, m, g_n, w_b, v_cm, roll,pitch,yaw ):
        #C_bn transfers gravity from NED coordinates to body frame
        return 1/m*F + np.dot(self.computeRotationMatrix(roll,pitch,yaw),g_n)- np.cross(w_b,v_cm)
    
    
    #ang acceleration of the body
    #M_b = Ang acceleration due to moments from forces [l,M,N] (roll,pitch,yaw axis)
    #J_b is the inertia matrix, resistance due to change in speed of rotation due to aircraft mass distribution 
    def returnAngAcceleration(self,J_b,M_b, w_b, v_b ):
        #second part of equation is the angular acceleration due to the rotation of the angular momentum vector
        return np.dot(np.linalg.inv(J_b),M_b - np.dot(np.cross(w_b,v_b),J_b,w_b))
    

def main():
    sim = Simulation()
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
    F_b = [0,0,0] #F_xb, F_yb, F_zb
    m_kg = 0 #mass 
    
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
    
    
    x0 = x0.transpose(); nx0 =x0.size
    t0_s = 0.0
    tf_s  = 10.0
    h_s = 0.01

    t_s = np.arange(t0_s, tf_s+ h_s, h_s); nt_s = t_s.size
    x= np.empty((nx0,nt_s),dtype=float)

    x[:,0] = x0

