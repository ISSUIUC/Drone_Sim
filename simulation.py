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
        return (1/m)*F + np.dot(self.computeRotationMatrix(roll,pitch,yaw),g_n)- np.cross(w_b,v_cm)
    
    
    #ang acceleration of the body
    #M_b = Ang acceleration due to moments from forces [l,M,N] (roll,pitch,yaw axis)
    #J_b is the inertia matrix, resistance due to change in speed of rotation due to aircraft mass distribution 
    def returnAngAcceleration(self,J_b,M_b, w_b, v_b ):
        #second part of equation is the angular acceleration due to the rotation of the angular momentum vector
        return np.dot(np.linalg.inv(J_b),M_b - np.dot(np.cross(w_b,v_b),J_b,w_b))
    
