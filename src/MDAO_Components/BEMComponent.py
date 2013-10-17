#__all__ = ['BEMComponent']

from math import pi, cos, sin, tan
import numpy as np
import pylab as py

from openmdao.main.api import Component, Assembly, VariableTree
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from openmdao.lib.drivers.api import SLSQPdriver
from openmdao.lib.casehandlers.api import DumpCaseRecorder

import sys
sys.path.append('../CCblade/src/')
from ccblade import CCBlade, CCAirfoil

from os import path

class BEMComponent(Component):
    '''Component to evaluate wind turbine performance for a given twist and taper'''
    Rhub = 1.5
    Rtip = 63.

    # set conditions
    Uinf  = 10.0
    tsr   = 7.55
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

    def __init__(self, alpha_sweep, r, optChord=False):

        super(BEMComponent, self).__init__()
        self.r           = r
        self.n_elements  = len(r)
        self.alpha_sweep = alpha_sweep
        self.nSweep      = len(alpha_sweep)

        # Inputs
        self.add('theta',  Array(np.zeros([self.n_elements]), size=[self.n_elements], iotype="in"))
        self.add('cls',    Array(np.zeros([self.nSweep]),     size=[self.nSweep],     iotype="in"))
        self.add('cds',    Array(np.zeros([self.nSweep]),     size=[self.nSweep],     iotype="in"))

        if optChord:
            self.add('chord',  Array(np.zeros([self.n_elements]), size=[self.n_elements], iotype="in"))
        else:
            if self.n_elements != 17:
                error("Must have 17 elements")
            self.chord = np.array([3.542, 3.854, 4.167, 4.557, 4.652, 4.458, 4.249, 4.007, 3.748,
                      3.502, 3.256, 3.010, 2.764, 2.518, 2.313, 2.086, 1.419])

        # Outputs
        self.add('power', Float(iotype="out"))
        self.nEvalsExecute = 0
        self.totalEvals = 0

        # Set up keys for j
        self.input_keys = []
        for j in range(self.n_elements):
            self.input_keys.append('theta[%d]'%j)
        for j in range(self.n_elements):
            self.input_keys.append('chord[%d]'%j)
        for j in range(self.nSweep):
            self.input_keys.append('cls[%d]'%j)
        for j in range(self.nSweep):
            self.input_keys.append('cds[%d]'%j)
        self.J = np.zeros([1,2*self.n_elements + 2*self.nSweep])

        self.output_keys = ('power',)

        # Load the du25_A17 airfoil data, which will be used for alphas where SU^2 is invalid
        self.du25data   = np.loadtxt('du25.dat')
        alpha_list      = self.du25data[:,0]
        cl_list         = self.du25data[:,1]
        cd_list         = self.du25data[:,2]

        # Store the data outside of the alpha range SU^2 will be providing
        j_pre  = alpha_list < self.alpha_sweep[0]
        self.alpha_pre = alpha_list[j_pre]
        self.cl_pre    = cl_list[j_pre]
        self.cd_pre    = cd_list[j_pre]
        j_post = alpha_list > self.alpha_sweep[-1]
        self.alpha_post = alpha_list[j_post]
        self.cl_post    = cl_list[j_post]
        self.cd_post    = cd_list[j_post]

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
    
        af = [0]*len(self.r)
        for i in range(len(self.r)):
            af[i] = airfoil_types[af_idx[i]]

        return af


    def execute(self):

        # Constructor of CCBlade -------------------------------------------------------------------------------------------
        #    def __init__(self, r, chord, theta, af, Rhub, Rtip, B=3, rho=1.225, mu=1.81206e-5,
        #                 precone=0.0, tilt=0.0, yaw=0.0, shearExp=0.2, hubHt=80.0,
        #                 nSector=8, precurve=None, precurveTip=0.0, presweep=None, presweepTip=0.0,
        #                 tiploss=True, hubloss=True, wakerotation=True, usecd=True, iterRe=1, derivatives=False):
        #-------------------------------------------------------------------------------------------------------------------

        power, thrust, torque, blade = self.CallCCBlade()

        self.nEvalsExecute += 1
        print "Calling execute ", "nExecute", self.nEvalsExecute, "nEvals",self.totalEvals
        #print "theta", self.theta
        #print blade.alpha
        #print "power", power[0]
        self.power = power[0]

    def CallCCBlade(self):

        self.totalEvals += 1

        # Show cl vs. alpha
        #newAlphas = np.linspace(self.alphas[0],self.alphas[len(self.alphas)-1], 1000)
        #newRes = np.linspace(1e7,1e8, 1000)
        #for i in range(len(newAlphas)):
        #    cl,cd = airfoil.evaluate(newAlphas[i]/360 * pi,newRes[i])
        #    print "alpha", newAlphas[i], "\tcl", cl, "\tcd", cd
        #exit()

        af    = self.generate_af()
        blade = CCBlade(self.r, self.chord, self.theta, af, self.Rhub, self.Rtip,
                        self.B, self.rho, self.mu, self.precone, self.tilt, self.yaw, self.shearExp, self.hubHt, self.nSector)

        power, thrust, torque =  blade.evaluate([self.Uinf], [self.Omega], [self.pitch])

        return power, thrust, torque, blade

    def generate_af(self):

        alpha_list = []
        cl_list    = []
        cd_list    = []
        alpha_list.extend(self.alpha_pre)
        alpha_list.extend(self.alpha_sweep)
        alpha_list.extend(self.alpha_post)

        cl_list.extend(self.cl_pre)
        cl_list.extend(self.cls)
        cl_list.extend(self.cl_post)

        cd_list.extend(self.cd_pre)
        cd_list.extend(self.cds)
        cd_list.extend(self.cd_post)
                    
        # Create a CCAirfoil object using the input alpha sweep
        su2_airfoil = CCAirfoil(alpha_list, [], cl_list, cd_list)

        # load cylinder files
        afinit = CCAirfoil.initFromAerodynFile  # just for shorthand
        basepath = path.join(path.abspath('.'),'../CCBlade/test/5MW_AFFiles/')
        cylinder1 = afinit(basepath + 'Cylinder1.dat')
        cylinder2 = afinit(basepath + 'Cylinder2.dat')

        # Collect airfoils into list 
        af = [0]*self.n_elements
        af[0] = cylinder1
        af[1] = cylinder1
        af[2] = cylinder2
        for j in range(0,self.n_elements):
            af[j] = su2_airfoil

        return af

    def linearize(self):
        '''Compute Jacobian d(outputs)/d(inputs)
                   theta  chord  cls  cds
           ------|----------------------------------- 
           power |
        
        '''

        print "Calling linearize"
        
        af    = self.generate_af()
        blade = CCBlade(self.r, self.chord, self.theta, af, self.Rhub, self.Rtip,
                        self.B, self.rho, self.mu, self.precone, self.tilt, self.yaw, self.shearExp, self.hubHt, self.nSector,
                        derivatives=True)

        # Run BEM code and extract derivatives
        self.P, self.T, self.Q, self.dP_ds, self.dT_ds, self.dQ_ds, self.dP_dv, self.dT_dv, \
        self.dQ_dv = blade.evaluate([self.Uinf], [self.Omega], [self.pitch], coefficient=False)

        # Store relevant parts of Jacobian in self.J
        for j in xrange(self.n_elements):
            self.J[0,j] = self.dP_dv[0,2,j]
        for j in xrange(self.n_elements):
            self.J[0,self.n_elements+j] = self.dP_dv[0, 1, j]

        clStepSize = 1e-8
        cdStepSize = 1e-8

        offset = self.n_elements*2
        #compute finite difference for derivatives wrt cl
        for j in xrange(self.nSweep):
            
            self.cls[j] += clStepSize
            power1, thrust1, torque1, blade1 = self.CallCCBlade()
            self.cls[j] -= 2*clStepSize
            power2, thrust2, torque2, blade2 = self.CallCCBlade()
            self.cls[j] += clStepSize
            self.J[0, offset + j] = (power1 -power2) / (2 * clStepSize)
            
            #self.J[0, offset + j] = 0


        offset = self.n_elements*2 + self.nSweep
        #compute finite difference for derivatives wrt cl
        for j in xrange(self.nSweep):
            
            power0, thrust0, torque0, blade0 = self.CallCCBlade()
            self.cds[j] += cdStepSize
            power1, thrust1, torque1, blade1 = self.CallCCBlade()
            self.cds[j] += cdStepSize
            power2, thrust2, torque2, blade2 = self.CallCCBlade()
            self.cds[j] -= 2* clStepSize
            self.J[0, offset + j] = (-3*power0 + 4*power1 - power2) / (2* cdStepSize)
            
            #self.J[0, offset + j] = 0.1

        print "J:"
        for j,key in enumerate(self.input_keys):
            print self.input_keys[j], ":", self.J[0,j]

    def provideJ(self):
        return self.input_keys, self.output_keys, self.J


class BEMAssembly(Assembly):
    '''A class with just one BEMComponent, to optimize the BEM portion independently of SU^2'''

    def __init__(self, alpha_sweep, r, optChord=False):
        self.alpha_sweep = alpha_sweep
        self.r           = r
        super(BEMAssembly, self).__init__()

    def configure(self):

        self.nSweep     = len(self.alpha_sweep)
        self.n_elements = len(self.r)

        # Inputs
        self.add('cls',    Array(np.zeros([self.nSweep]),     size=[self.nSweep],     iotype="in"))
        self.add('cds',    Array(np.zeros([self.nSweep]),     size=[self.nSweep],     iotype="in"))

        # Outputs
        self.add('power', Float(iotype="out"))

        # Add the one component
        self.add('bem_component', BEMComponent(self.alpha_sweep, self.r))

        # Set up the SLSQP driver
        self.driver.workflow.add('bem_component')
        self.add('driver', SLSQPdriver())
        self.driver.iprint = 1
        for j in range(self.n_elements):
            self.driver.add_parameter('bem_component.theta[%d]'%j, high=10, low=0.0, start=4)

        # Connect ins and outs
        for j in range(self.nSweep):
            self.connect('cls[%d]'%j, 'bem_component.cls[%d]'%j)
            self.connect('cds[%d]'%j, 'bem_component.cds[%d]'%j)
        self.connect('bem_component.power','power')

        # Objective for optimization
        self.driver.add_objective('-power')


if __name__=="__main__":

    # Andrew's test case
    n_elems = 17

    # Geometry to match that of Andrew's test case
    r = np.array([2.8667, 5.6000, 8.3333, 11.7500, 15.8500, 19.9500, 24.0500,
                  28.1500, 32.2500, 36.3500, 40.4500, 44.5500, 48.6500, 52.7500,
                  56.1667, 58.9000, 61.6333])
    chord = np.array([3.542, 3.854, 4.167, 4.557, 4.652, 4.458, 4.249, 4.007, 3.748,
                      3.502, 3.256, 3.010, 2.764, 2.518, 2.313, 2.086, 1.419])
    theta = np.array([13.308, 13.308, 13.308, 13.308, 11.480, 10.162, 9.011, 7.795,
                      6.544, 5.361, 4.188, 3.125, 2.319, 1.526, 0.863, 0.370, 0.106])

    #alpha_sweep = np.linspace(-10,80,50)
    alpha_sweep = np.array([-90,-30, -20, -15, -13, 0., 13., 15, 20, 30,90])
    cl_data    = np.array([0,-1.1,-.7, -.8, -1.3,0, 1.3, .8, .7, 1.1,0])
    cd_data    = np.array([5,1.,0.6,0.3, 1e-2, 0., 1e-2, 0.3, 0.6, 1.,5]) + 1e-5

    top = BEMAssembly(alpha_sweep, r)

    for j in range(len(alpha_sweep)):
        top.cls[j] = cl_data[j]
        top.cds[j] = cd_data[j]

    top.run()

    print top
    print top.bem_component
    print top.driver

    if top.driver.error_code != 0:
        print "error code: ",top.driver.error_code, "error message: ", top.driver.error_messages[top.driver.error_code]
    print
    print "power: ", top.power
    print "theta: ", top.bem_component.theta

