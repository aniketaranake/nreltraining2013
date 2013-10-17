import numpy as np, pylab as py

af1 = np.loadtxt('du25.dat')

py.figure()
py.subplot(2,1,1)
py.plot(af1[:,0], af1[:,1])
py.title('Cl')

py.subplot(2,1,2)
py.plot(af1[:,0], af1[:,2])
py.title('Cd')

py.show()
