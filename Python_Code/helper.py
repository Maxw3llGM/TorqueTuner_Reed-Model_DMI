import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from functools import partial



def reedFlow_Equation(pressure, exponent_1, exponent_2, tip_equilibrium, spring_constant):
    flow = np.where(pressure<=tip_equilibrium*spring_constant,(tip_equilibrium*np.power(np.abs(pressure),exponent_1)-np.power(np.abs(pressure),exponent_2)/spring_constant)*np.sign(pressure),0)
    return flow

def linearized_reedFlow_Equation(pressure, tip_equilibrium, spring_constant):
    coefficient_1 = tip_equilibrium*spring_constant
    coefficient_2 = coefficient_1/3

    a_1 = reedFlow_Equation(coefficient_2,1/2,3/2,tip_equilibrium,spring_constant)/coefficient_2
    a_2 = reedFlow_Equation(coefficient_2,1/2,3/2,tip_equilibrium,spring_constant) * -1 /(coefficient_1-coefficient_2)
    b_2 = (a_1-a_2)*coefficient_2

    return np.where(pressure<coefficient_2, np.where(pressure > 0, a_1*pressure, 0),np.where(pressure<coefficient_1, a_2*pressure+b_2,0))

def pressure_difference_cal(c, b, a):
    return b-c*a

def bisection(x0,x1,f,error = 0.001):
    f2 = 100
    f0 = f(x0)
    f1 = f(x1)
    while abs(f2) > error:
        x2 = (x0+x1)/2
        f2 = f(x2)
        if f0*f2 < 0:
            x1 = x2
        else:
            x0 = x2
            f0 = f2
    return x2

def partial_diff(func1,func2,x):
    return func1(x)-func2(x)

def reedFlow_Equation_partiable(tip_equilibrium, spring_constant, exponent_1, exponent_2,pressure):
    flow = np.where(pressure<=tip_equilibrium*spring_constant,(tip_equilibrium*np.power(np.abs(pressure),exponent_1)-np.power(np.abs(pressure),exponent_2)/spring_constant)*np.sign(pressure),0)
    return flow

def smallest_value(x, const1, fun1, fun2, prev_index):
    diff = partial(partial_diff,fun1,fun2)
    
    difference = diff(x)
    index_min = np.argmin(abs(difference))
    if (prev_index > index_min) and prev_index-index_min >= -10:
        index_min = prev_index + 1
    prev_index_min = index_min

    if difference[index_min] == 0.0:
        return x[index_min], prev_index_min
    if(index_min < len(difference) and index_min>0):
        if(np.sign(difference[index_min-1]*difference[index_min]) < 0):
            second_min = index_min - 1
        else :
            second_min = index_min + 1
    else:
        if(index_min == 0):
            second_min = index_min + 1
        if(index_min == len(difference)):
            second_min = index_min - 1
    return bisection(x[index_min],x[second_min],diff,const1), prev_index_min

def pressure_difference_solution(mouth_and_incoming_pressure_difference,pressure_difference,H,k):
    pressure_difference_solution = np.zeros_like(pressure_difference)
    flow = partial(reedFlow_Equation_partiable,H, k, 1/2, 3/2)

    prev_index = 0
    for i in range(len(mouth_and_incoming_pressure_difference)):
        p_minus_delta = partial(pressure_difference_cal,1,mouth_and_incoming_pressure_difference[i])
        pressure_difference_solution[i],prev_index = smallest_value(pressure_difference,mouth_and_incoming_pressure_difference[i],flow,p_minus_delta,prev_index)
    return pressure_difference_solution

##### SCRAP HERE ########
# # ************************* Rudimentary Delay Line **********************************

# n = 2049
# global inpoint
# inpoint = 0
# global outpoint
# outpoint = 0
# delay = 200
# global delay_line
# delay_line = np.zeros([1,n])

# if(inpoint >= delay):
#     outpoint = inpoint - delay
# else: 
#     outpoint = len(delay_line) + inpoint - delay

# def tick(input):
#     delay_line[inpoint] = input
#     inpoint += 1
#     if inpoint == len(delay_line):
#         inpoint = 0
    
#     lastframe = delay_line[outpoint]
#     outpoint += 1

#     if outpoint == len(delay_line):
#         outpoint = 0

#     return lastframe

# # *******************************************************************************



'''
Idea:

p_delta  = p_m_d - u_d'(p_delta)
p_m_d = p_m - 2 p_d (d0n3)
'''
# ****************** method 1 WRONG ********************
# mouth_pressure = np.linspace(0,1.0, 5000)
# incoming_pressure = np.append(np.zeros([1,2000]),0.6*mouth_pressure[0:3000])
# pressure_difference = np.zeros(5000)
# print(mouth_pressure.shape, incoming_pressure.shape, pressure_difference.shape)
# p_m_d = mouth_pressure - 2*incoming_pressure
# stored_pressure_difference = 0

# for i in range(0, len(mouth_pressure),1):
#     print(p_m_d[i], incoming_pressure[i])
#     u_d_prime = 0.015*reedFlow_Equation(stored_pressure_difference, init_H, init_k)
#     pressure_difference[i] = p_m_d[i] - u_d_prime
#     stored_pressure_difference = pressure_difference[i]

# ****************** method 2 TBD ********************