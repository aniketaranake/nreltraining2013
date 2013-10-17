import numpy as np, pylab as py

import os.path

alpha = np.array(range(-15,16))

npoints = len(alpha)
cl = np.zeros([npoints])
cd = np.zeros([npoints])

for j,a in enumerate(alpha):

  dirname  = 'restart_files/sol%03d'%a
  filename = dirname+'/restart_flow.dat'
  if os.path.isdir(dirname):
    if os.path.isfile(filename):
      print "Found file ", a
      data = np.loadtxt('%s/history.plt'%dirname, skiprows=3, delimiter=",")
      cl[j] = data[-1,1]
      cd[j] = data[-1,2]

# Plot RAE polar
py.figure()
py.subplot(2,1,1)
py.plot(alpha, cl, 'rs-')

py.subplot(2,1,2)
py.plot(alpha,cd, 'rs-')


# Plot airfoil from example
af1 = np.loadtxt('du25.dat')
py.subplot(2,1,1)
py.plot(af1[:,0], af1[:,1])
py.title('Cl')

py.subplot(2,1,2)
py.plot(af1[:,0], af1[:,2])
py.title('Cd')

py.show()
