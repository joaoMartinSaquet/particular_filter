import matplotlib.pyplot as plt
import numpy as np
from math import *

pi = 3.14

def gauss(x, y,mean,var):
    return 1/(2*pi*var*var)*np.exp(-1/2*((x/var)**2 +(y/var)**2) )




N = 1000
up = 1
bot = -up
var = 0.04 #var = sigma²
x = np.linspace(bot,up,N)

z = (1/sqrt(2*np.pi)*sqrt(var))*np.exp(-x**2/(2*var))



plt.plot(x,z)
plt.title("pdf p(z|x)")
plt.xlabel("distance en m")
plt.ylabel("pdf(dist)")
plt.grid()
plt.show()

