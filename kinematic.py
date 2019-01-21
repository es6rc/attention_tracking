from sympy import *
import math
from IPython.display import Latex
from kinematic import dhtf, rttf, latexmtx, invdhtf 

PI = math.pi

theta2 = +0
alpha2 = 0
d2 = 0
r2 = -.13691

theta3 = -PI/2
alpha3 = -PI/2
d3 = .0535
r3 = 0

theta4 = 0
alpha4 = 0
d4 = .04435
r4 = 0

theta5 = theta2
alpha5 = 0
d5 = 0
r5 = -r2

theta6 = -PI/2
alpha6 = -PI/2
d6 = d3
r6 = 0

theta7 = 0
alpha7 = 0
d7 = -d4
r7 = 0
x4L = -.0218
theta4L = -PI/2

T12 = dhtf('symbolic', 2, alpha2, d2, r2, theta2)
print('T12')
Latex(latexmtx(T12))
T21 = invdhtf('symbolic', 2, alpha2, d2, r2, theta2)
print('T21')
Latex(latexmtx(T21))

T23 = dhtf('symbolic', 3, alpha3, d3, r3, theta3)
print("T23")
Latex(latexmtx(T23))
T32 = invdhtf('symbolic', 3, alpha3, d3, r3, theta3)
print("T32")
Latex(latexmtx(T32))

T34 = dhtf('symbolic', 4, alpha4, d4, r4, theta4)
print('T34')
Latex(latexmtx(T34))
T43 = invdhtf('symbolic', 4, alpha4, d4, r4, theta4)
print('T43')
Latex(latexmtx(T43))

T4L = rttf(x4L, 0, 0, 0, theta4L, 0)
print('T4L')
Latex(latexmtx(T4L))

TL4 = rttf(0, 0, x4L, 0, -theta4L, 0)
print('TL4')
Latex( latexmtx(TL4) )

T1L = T12 * T23 * T34 * T4L
print("T1L")
Latex( latexmtx(T1L) )

TL1 = TL4 * T43 * T32 * T21
print("TL1")
Latex( latexmtx(TL1) )