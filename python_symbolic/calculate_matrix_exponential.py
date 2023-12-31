import sympy as sp
from sympy import *
init_printing(use_unicode=True)

A = sp.symbols('A')
B = sp.symbols('B')
C = sp.symbols('C')
dt = sp.symbols('dt')

R = sp.symbols('R')
ax = sp.symbols('ax')


# a = exp(Matrix([
#     [0, 1, 0, 0, 0],
#     [0, 0, A*B, C, 0], 
#     [0, 0, 1, 0, 0],
#     [0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0]]))
F0 = Matrix([
    [0, 1, 0, 0, 0, 0],
    [0, 0, -R*ax, 0, -R, 0], 
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0]])

I66 =Matrix([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])


expF0dt_1 = I66 + F0*dt
expF0dt_2 = I66 + F0*dt + 1/2*F0*F0*dt**2
expF0dt_3 = I66 + F0*dt + 1/2*F0*F0*dt**2 + 1/6 * F0*F0*F0*dt**3
expF0dt_4 = I66 + F0*dt + 1/2*F0*F0*dt**2 + 1/6 * F0*F0*F0*dt**3 + 1/24 * F0*F0*F0*F0*dt**4
expF0dt_5 = I66 + F0*dt + 1/2*F0*F0*dt**2 + 1/6 * F0*F0*F0*dt**3 + 1/24 * F0*F0*F0*F0*dt**4 + 1/120 * F0*F0*F0*F0*F0*dt**5
print("expF0dt_1:" ,expF0dt_1)
print("expF0dt_2:" ,expF0dt_2)
print("expF0dt_3:" ,expF0dt_3)
print("expF0dt_4:" ,expF0dt_4)
print("expF0dt_5:" ,expF0dt_5)
print(F0*F0*F0*F0)
# expF0dt_1: 
# Matrix(
# [
# [1, dt, 0, 0, 0, 0], 
# [0, 1, -R*ax*dt, 0, -R*dt, 0], 
# [0, 0, 1, 0, 0, dt], 
# [0, 0, 0, 1, 0, 0], 
# [0, 0, 0, 0, 1, 0], 
# [0, 0, 0, 0, 0, 1]])
# expF0dt_2: 
# Matrix(
# [
# [1, dt, -1/2*R*ax*dt**2, 0, -1/2*R*dt**2, 0], 
# [0, 1, -R*ax*dt, 0, -R*dt, -1/2*R*ax*dt**2], 
# [0, 0, 1, 0, 0, dt], 
# [0, 0, 0, 1, 0, 0], 
# [0, 0, 0, 0, 1, 0], 
# [0, 0, 0, 0, 0, 1]])
# expF0dt_3: 
# Matrix(
# [
# [1, dt, -1/2*R*ax*dt**2, 0, -1/2*R*dt**2, -1/6*R*ax*dt**3], 
# [0, 1, -R*ax*dt, 0, -R*dt, -1/2*R*ax*dt**2],
# [0, 0, 1, 0, 0, dt], 
# [0, 0, 0, 1, 0, 0], 
# [0, 0, 0, 0, 1, 0], 
# [0, 0, 0, 0, 0, 1]])
# expF0dt_4: 
# Matrix(
# [
# [1, dt, -1/2*R*ax*dt**2, 0, -1/2*R*dt**2, -1/6*R*ax*dt**3], 
# [0, 1, -R*ax*dt, 0, -R*dt, -1/2*R*ax*dt**2], 
# [0, 0, 1, 0, 0, dt], 
# [0, 0, 0, 1, 0, 0], 
# [0, 0, 0, 0, 1, 0],
# [0, 0, 0, 0, 0, 1]])
# expF0dt_5: 
# Matrix(
# [
# [1, dt, -1/2*R*ax*dt**2, 0, -1/2*R*dt**2, -1/6*R*ax*dt**3], 
# [0, 1, -R*ax*dt, 0, -R*dt, -1/2*R*ax*dt**2], 
# [0, 0, 1, 0, 0, dt], 
# [0, 0, 0, 1, 0, 0], 
# [0, 0, 0, 0, 1, 0], 
# [0, 0, 0, 0, 0, 1]])
