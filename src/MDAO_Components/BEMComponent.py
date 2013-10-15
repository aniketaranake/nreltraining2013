#__all__ = ['BEMComponent']

from math import pi, cos, sin, tan
import numpy as np

from openmdao.main.api import Component, Assembly, VariableTree
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree

import sys
sys.path.append('../CCblade/src/')
from ccblade import CCBlade, CCAirfoil

from os import path

class BEMComponent(Component):
    '''Component to evaluate wind turbine performance for a given twist and taper'''

    # Geometry to match that of Andrew's test case
    r = np.array([2.8667, 5.6000, 8.3333, 11.7500, 15.8500, 19.9500, 24.0500,
                  28.1500, 32.2500, 36.3500, 40.4500, 44.5500, 48.6500, 52.7500,
                  56.1667, 58.9000, 61.6333])
    Rhub = 1.5
    Rtip = 63.

    # set conditions
    Uinf = 10.0
    tsr = 7.55
    pitch = 0.0
    Omega = Uinf*tsr/Rtip * 30.0/pi  # convert to RPM

    def __init__(self, n_elements=17):

        super(BEMComponent, self).__init__()

        # Inputs
        self.add('theta',  Array(np.zeros([n_elements]), size=[n_elements], iotype="in"))
        self.add('chord',  Array(np.zeros([n_elements]), size=[n_elements], iotype="in"))

        # Outputs
        self.add('power', Float(iotype="out"))

        # TODO: Replace this with another way of determining af
        self.load_test_airfoils()

    def load_test_airfoils(self):
        '''Loads the airfoils from Andre Ning's directory of test airfoils'''
        # Path for output files
        afinit = CCAirfoil.initFromAerodynFile  # just for shorthand
        basepath = path.join(path.abspath('.'),'../CCBlade/test/5MW_AFFiles/')

        # load all airfoils
        airfoil_types = [0]*8
        airfoil_types[0] = afinit(basepath + 'Cylinder1.dat')
        airfoil_types[1] = afinit(basepath + 'Cylinder2.dat')
        airfoil_types[2] = afinit(basepath + 'DU40_A17.dat')
        airfoil_types[3] = afinit(basepath + 'DU35_A17.dat')
        airfoil_types[4] = afinit(basepath + 'DU30_A17.dat')
        airfoil_types[5] = afinit(basepath + 'DU25_A17.dat')
        airfoil_types[6] = afinit(basepath + 'DU21_A17.dat')
        airfoil_types[7] = afinit(basepath + 'NACA64_A17.dat')
    
        # place at appropriate radial stations
        af_idx = [0, 0, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 7, 7, 7, 7]
    
        self.af = [0]*len(self.r)
        for i in range(len(self.r)):
            self.af[i] = airfoil_types[af_idx[i]]


    def execute(self):

        # Constructor of CCBlade -------------------------------------------------------------------------------------------
        #    def __init__(self, r, chord, theta, af, Rhub, Rtip, B=3, rho=1.225, mu=1.81206e-5,
        #                 precone=0.0, tilt=0.0, yaw=0.0, shearExp=0.2, hubHt=80.0,
        #                 nSector=8, precurve=None, precurveTip=0.0, presweep=None, presweepTip=0.0,
        #                 tiploss=True, hubloss=True, wakerotation=True, usecd=True, iterRe=1, derivatives=False):
        #-------------------------------------------------------------------------------------------------------------------

        # Just chosen to match Andrew's... no intention of changing this for now
        B   = 3
        rho = 1.225
        mu  = 1.81206e-5
        tilt = -5.0
        precone = 2.5
        yaw = 0.0
        shearExp = 0.2
        hubHt = 80.0
        nSector = 8

        blade = CCBlade(self.r, self.chord, self.theta, self.af, self.Rhub, self.Rtip,
                        B, rho, mu, precone, tilt, yaw, shearExp, hubHt, nSector)

        power, thrust, torque = blade.evaluate([self.Uinf], [self.Omega], [self.pitch]) 
        self.power = power[0]


if __name__=="__main__":


    # Andrew's test case
    n_elems = 17
    chord = np.array([3.542, 3.854, 4.167, 4.557, 4.652, 4.458, 4.249, 4.007, 3.748,
                      3.502, 3.256, 3.010, 2.764, 2.518, 2.313, 2.086, 1.419])
    theta = np.array([13.308, 13.308, 13.308, 13.308, 11.480, 10.162, 9.011, 7.795,
                      6.544, 5.361, 4.188, 3.125, 2.319, 1.526, 0.863, 0.370, 0.106])

    top = Assembly()
    top.add('b', BEMComponent())

    print top
    print top.b

    for j in range(n_elems):
      top.b.chord[j] = chord[j]
      top.b.theta[j] = theta[j]

    top.driver.workflow.add('b')
    top.run()
    print
    print "power: ", top.b.power
