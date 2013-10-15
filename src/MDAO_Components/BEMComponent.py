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

    def __init__(self, n_elements=17, nSweep=8):

        super(BEMComponent, self).__init__()

        self.n_elements = n_elements
        self.nSweep     = nSweep

        # Inputs
        self.add('theta',  Array(np.zeros([n_elements]), size=[n_elements], iotype="in"))
        self.add('chord',  Array(np.zeros([n_elements]), size=[n_elements], iotype="in"))
        self.add('alphas', Array(np.zeros([nSweep]),     size=[nSweep],     iotype="in"))
        self.add('cls',    Array(np.zeros([nSweep]),     size=[nSweep],     iotype="in"))
        self.add('cds',    Array(np.zeros([nSweep]),     size=[nSweep],     iotype="in"))

        # Outputs
        self.add('power', Float(iotype="out"))
        self.nEvalsExecute = 0
        self.totalEvals = 0

    def load_test_airfoils(self):
        '''Loads the airfoils from Andrew Ning's directory of test airfoils'''
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

        power, thrust, torque = self.CallCCBlade()

        self.nEvalsExecute += 1
        print "Calling execute ", "nExecute", self.nEvalsExecute, "nEvals",self.totalEvals
        print "theta", self.theta
        print "power", power[0]
        self.power = power[0]

    def CallCCBlade(self):
        #print "alpha",self.alphas
        #print "cls",self.cls
        #print "cds",self.cds
        self.totalEvals += 1
        # Create a CCAirfoil object using the input alpha sweep
        airfoil = CCAirfoil(self.alphas, [], self.cls, self.cds)
        
            
        # Show cl vs. alpha
        #newAlphas = np.linspace(self.alphas[0],self.alphas[len(self.alphas)-1], 1000)
        #newRes = np.linspace(1e7,1e8, 1000)
        #for i in range(len(newAlphas)):
        #    cl,cd = airfoil.evaluate(newAlphas[i]/360 * pi,newRes[i])
        #    print "alpha", newAlphas[i], "\tcl", cl, "\tcd", cd
        #exit()

        self.af = [0]*self.n_elements

        for j in range(self.n_elements):
            self.af[j] = airfoil

        blade = CCBlade(self.r, self.chord, self.theta, self.af, self.Rhub, self.Rtip,
                        self.B, self.rho, self.mu, self.precone, self.tilt, self.yaw, self.shearExp, self.hubHt, self.nSector)

        return blade.evaluate([self.Uinf], [self.Omega], [self.pitch]) 

    def linearize(self):
        '''Compute Jacobian d(outputs)/d(inputs)
                   theta  chord  (TODO: alphas  cls  cds)
           ------|----------------------------------- 
           power |
        
        '''

        print "Calling linearize"
        
        # Create a CCAirfoil object using the input alpha sweep
        airfoil = CCAirfoil(self.alphas, [], self.cls, self.cds)
        self.af = [0]*self.n_elements

        for j in range(self.n_elements):
            self.af[j] = airfoil

        blade = CCBlade(self.r, self.chord, self.theta, self.af, self.Rhub, self.Rtip,
                        self.B, self.rho, self.mu, self.precone, self.tilt, self.yaw, self.shearExp, self.hubHt, self.nSector,
                        derivatives=True)

        # Run BEM code and extract derivatives
        self.P, self.T, self.Q, self.dP_ds, self.dT_ds, self.dQ_ds, self.dP_dv, self.dT_dv, \
        self.dQ_dv = blade.evaluate([self.Uinf], [self.Omega], [self.pitch], coefficient=False)

        # Store relevant parts of Jacobian in self.J
        self.J = np.zeros([1,2*self.n_elements + 3*self.nSweep])
        for j in range(self.n_elements):
            self.J[0,j] = self.dP_dv[0,2,j]
        for j in range(self.n_elements):
            self.J[0,self.n_elements+j] = self.dP_dv[0, 1, j]

        # Say the derivative wrt. alpha is zero
        for j in range(self.nSweep):
            self.J[0,self.n_elements*2 + j] = 0

        clStepSize = 1e-8
        cdStepSize = 1e-8

        offset = self.n_elements*2 + self.nSweep
        #compute finite difference for derivatives wrt cl
        for j in range(self.nSweep):
            self.cls[j] += clStepSize
            power1, thrust1, torque1 = self.CallCCBlade()
            self.cls[j] -= 2*clStepSize
            power2, thrust2, torque2 = self.CallCCBlade()
            self.cls[j] += clStepSize
            self.J[0, offset + j] = (power1 -power2) / (2 * clStepSize)

        offset = self.n_elements*2 + 2*self.nSweep
        #compute finite difference for derivatives wrt cl
        for j in range(self.nSweep):
            power0, thrust0, torque0 = self.CallCCBlade()
            self.cds[j] += cdStepSize
            power1, thrust1, torque1 = self.CallCCBlade()
            self.cds[j] += cdStepSize
            power2, thrust2, torque2 = self.CallCCBlade()
            self.cds[j] -= 2* clStepSize
            self.J[0, offset + j] = (-3*power0 + 4*power1 - power2) / (2* cdStepSize)

        print self.J

    def provideJ(self):

        input_keys = []
        for j in range(self.n_elements):
            input_keys.append('theta[%d]'%j)
        for j in range(self.n_elements):
            input_keys.append('chord[%d]'%j)
        for j in range(self.nSweep):
            input_keys.append('alphas[%d]'%j)
        for j in range(self.nSweep):
            input_keys.append('cls[%d]'%j)
        for j in range(self.nSweep):
            input_keys.append('cds[%d]'%j)

        output_keys = ('power',)



        return input_keys, output_keys, self.J


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
