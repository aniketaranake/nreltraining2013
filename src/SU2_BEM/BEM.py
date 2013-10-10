__all__ = ['BEM', 'BladeElement', 'BEMPerf', 'BEMPerfData']

from math import pi, cos, sin, tan

import numpy as np
from scipy.optimize import fsolve
from scipy.interpolate import interp1d

from openmdao.main.api import Component, Assembly, VariableTree
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from openmdao.lib.components.api import LinearDistribution


class FlowConditions(VariableTree):     
    rho = Float(1.225, desc="air density", units="kg/m**3")
    V = Float(7., desc="free stream air velocity", units="m/s")


#Note, not being used right now
class BEMPerfData(VariableTree):
    """Container that holds all rotor performance data"""

    net_thrust = Float(desc="net axial thrust", units="N")
    net_power = Float(desc="net power produced", units="W")
    Ct = Float(desc="thrust coefficient")
    Cp = Float(desc="power coefficient")
    J = Float(desc="advance ratio")
    tip_speed_ratio = Float(desc="tip speed ratio")
    #eta = Float(desc="turbine efficiency")

class BEMPerf(Component):
    """collects data from set of BladeElements and calculates aggregate values"""

    r = Float(.8, iotype="in", desc="tip radius of the rotor", units="m")
    rpm = Float(2100, iotype="in", desc="rotations per minute", low=0, units="min**-1")

    free_stream = VarTree(FlowConditions(), iotype="in") 

    #data = VarTree(BEMPerfData(), iotype="out")
    #NOTE: Var Trees in the objective don't seem to be working
    data = Array(iotype="out") #[F,Pwr,Ct,Cp,J,lambda]

    #this lets the size of the arrays vary for different numbers of elements
    def __init__(self, n=10):
        super(BEMPerf, self).__init__()

        #needed initialization for VTs
        #self.add('data', BEMPerfData())  
        #self.add('free_stream', FlowConditions())

        #array size based on number of elements
        self.add('delta_Ct', Array(iotype='in', desc='thrusts from %d different blade elements'%n,
                               default_value=np.ones((n,)), shape=(n,), dtype=Float, units="N"))
        self.add('delta_Cp', Array(iotype='in', desc='Cp integrant points from %d different blade elements'%n,
                               default_value=np.ones((n,)), shape=(n,), dtype=Float))
        self.add('lambda_r', Array(iotype='in', desc='lambda_r from %d different blade elements'%n,
                               default_value=np.ones((n,)), shape=(n,), dtype=Float))      
                                 
    def execute(self):

        #self.data = BEMPerfData()  #emtpy the variable tree
        self.data = np.zeros((6,))
        #[F,Pwr,Ct,Cp,J,lambda]

        V_inf = self.free_stream.V
        rho = self.free_stream.rho

        norm = (.5*rho*(V_inf**2)*(pi*self.r**2))
        #self.data.Ct= np.trapz(self.delta_Ct, x=self.lambda_r)
        Ct = self.data[2]= np.trapz(self.delta_Ct, x=self.lambda_r)
        #self.data.net_thrust = self.data.Ct*norm
        self.data[0] = Ct*norm

        #self.data.Cp = np.trapz(self.delta_Cp, x=self.lambda_r) * 8. / self.lambda_r.max()**2
        #self.data.net_power = self.data.Cp*norm*V_inf
        Cp = self.data[3] = np.trapz(self.delta_Cp, x=self.lambda_r) * 8. / self.lambda_r.max()**2
        self.data[1] = Cp*norm*V_inf

        #self.data.J = V_inf/(self.rpm/60.0*2*self.r)
        self.data[4] = V_inf/(self.rpm/60.0*2*self.r)

        omega = self.rpm*2*pi/60
        #self.data.tip_speed_ratio = omega*self.r/self.free_stream.V
        self.data[5] = omega*self.r/self.free_stream.V


class BladeElement(Component):
    """Calculations for a single radial slice of a rotor blade"""

    #inputs
    a_init = Float(0.2, iotype="in", desc="initial guess for axial inflow factor")
    b_init = Float(0.01, iotype="in", desc="initial guess for angular inflow factor")
    rpm = Float(106.952, iotype="in", desc="rotations per minute", low=0, units="min**-1")
    r = Float(5., iotype="in", desc="mean radius of the blade element", units="m")
    dr = Float(1., iotype="in", desc="width of the blade element", units="m")
    twist = Float(1.616, iotype="in", desc="local twist angle", units="rad")
    chord = Float(.1872796, iotype="in", desc="local chord length", units="m", low=0)
    B = Int(3, iotype="in", desc="Number of blade elements")

    rho = Float(1.225, iotype="in", desc="air density", units="kg/m**3")
    V_inf = Float(7, iotype="in", desc="free stream air velocity", units="m/s")


    C_L = Array([0, 1.3, .8, .7, 1.1], size=(5,), iotype="in", desc="Section lift coefficients at alphas: [0., 13., 15, 20, 30]")
    C_D = Array([0., 0., 0.3, 0.6, 1.], size=(5,), iotype="in", desc="Section drag coefficients at alphas: [0., 13., 15, 20, 30]")

    #outputs
    V_0 = Float(iotype="out", desc="axial flow at propeller disk", units="m/s")
    V_1 = Float(iotype="out", desc="local flow velocity", units="m/s")
    V_2 = Float(iotype="out", desc="angular flow at propeller disk", units="m/s")
    omega = Float(iotype="out", desc="average angular velocity for element", units="rad/s")
    sigma = Float(iotype="out", desc="Local solidity")
    alpha = Float(iotype="out", desc="local angle of attack", units="rad")
    delta_Ct = Float(iotype="out", desc="section thrust coefficient", units="N")
    delta_Cp = Float(iotype="out", desc="section power coefficent")
    a = Float(iotype="out", desc="converged value for axial inflow factor")
    b = Float(iotype="out", desc="converged value for radial inflow factor")
    lambda_r = Float(8, iotype="out", desc="local tip speed ratio")
    phi = Float(1.487, iotype="out", desc="relative flow angle onto blades", units="rad")

 
    def _coeff_lookup(self, i):
        C_L = self.cl_interp(i)
        C_D = self.cd_interp(i)    
        return C_D, C_L
        
    def execute(self):  

        self.sigma = self.B*self.chord / (2* np.pi * self.r)
        self.omega = self.rpm*2*pi/60.0
        omega_r = self.omega*self.r
        self.lambda_r = self.omega*self.r/self.V_inf # need lambda_r for iterates

        #linear interpolation from airfoil data
        rad = np.array([0., 13., 15, 20, 30])*pi/180
        self.cl_interp = interp1d(rad, self.C_L, fill_value=0.001, bounds_error=False)

        rad = np.array([0., 10, 20, 30, 40])*pi/180
        self.cd_interp = interp1d(rad, self.C_D, fill_value=0.001, bounds_error=False)

        result = fsolve(self._iteration, [self.a_init, self.b_init])
        self.a = result[0]
        self.b = result[1]

        self.V_0 = self.V_inf - self.a*self.V_inf
        self.V_2 = omega_r-self.b*omega_r
        self.V_1 = (self.V_0**2+self.V_2**2)**.5

        q_c = self.B*.5*(self.rho*self.V_1**2)*self.chord*self.dr
        cos_phi = cos(self.phi)
        sin_phi = sin(self.phi)
        C_D, C_L = self._coeff_lookup(self.alpha)
        self.delta_Ct = q_c*(C_L*cos_phi-C_D*sin_phi)/(.5*self.rho*(self.V_inf**2)*(pi*self.r**2))
        self.delta_Cp = self.b*(1-self.a)*self.lambda_r**3*(1-C_D/C_L*tan(self.phi))

        print "%15s%15s%15s%15s%15s%15s%15s"%("r","alpha","a","b","cl","cd","twist")
        print "%15.7f%15.7f%15.7f%15.7f%15.7f%15.7f%15.7f"%(self.r,self.alpha*180./pi,self.a,self.b,C_L,C_D,self.twist*180./pi)

    def _iteration(self, X):
        self.phi = np.arctan(self.lambda_r*(1+X[1])/(1-X[0]))
        self.alpha = pi/2-self.twist-self.phi
        C_D, C_L = self._coeff_lookup(self.alpha)
        self.a = 1./(1 + 4.*(np.cos(self.phi)**2)/(self.sigma*C_L*np.sin(self.phi)))
        self.b = (self.sigma*C_L) / (4* self.lambda_r * np.cos(self.phi)) * (1 - self.a)

        return (X[0]-self.a), (X[1]-self.b)


class BEM(Assembly):
    """Blade Rotor with user specified number BladeElements"""

    #physical properties inputs
    r_hub = Float(0.2, iotype="in", desc="blade hub radius", units="m", low=0)
    twist_hub = Float(29, iotype="in", desc="twist angle at the hub radius", units="deg")
    chord_hub = Float(.7, iotype="in", desc="chord length at the rotor hub", units="m", low=.05)
    r_tip = Float(5, iotype="in", desc="blade tip radius", units="m")
    twist_tip = Float(-3.58, iotype="in", desc="twist angle at the tip radius", units="deg")
    chord_tip = Float(.187, iotype="in", desc="chord length at the rotor hub", units="m", low=.05)
    pitch = Float(0, iotype="in", desc="overall blade pitch", units="deg")
    rpm = Float(107, iotype="in", desc="rotations per minute", low=0, units="min**-1")
    B = Int(3, iotype="in", desc="number of blades", low=1)

    #wind condition inputs
    free_stream = VarTree(FlowConditions(), iotype="in") 

    def __init__(self, n_elements=6):
        self._n_elements = n_elements
        super(BEM, self).__init__()

    def configure(self):

        self.add('free_stream', FlowConditions()) #initialize

        n_elements = self._n_elements

        self.add('radius_dist', LinearDistribution(n=n_elements, units="m"))
        self.connect('r_hub', 'radius_dist.start')
        self.connect('r_tip', 'radius_dist.end')

        self.add('chord_dist', LinearDistribution(n=n_elements, units="m"))
        self.connect('chord_hub', 'chord_dist.start')
        self.connect('chord_tip', 'chord_dist.end')

        self.add('twist_dist', LinearDistribution(n=n_elements, units="deg"))
        self.connect('twist_hub', 'twist_dist.start')
        self.connect('twist_tip', 'twist_dist.end')
        self.connect('pitch', 'twist_dist.offset')

        self.driver.workflow.add('chord_dist')
        self.driver.workflow.add('radius_dist')
        self.driver.workflow.add('twist_dist')

        self.add('perf', BEMPerf(n=n_elements))
        self.create_passthrough('perf.data')
        self.connect('r_tip', 'perf.r')
        self.connect('rpm', 'perf.rpm')
        self.connect('free_stream', 'perf.free_stream')

        self._elements = []
        for i in range(n_elements):
            name = 'BE%d'%i
            self._elements.append(name)
            self.add(name, BladeElement())
            self.driver.workflow.add(name)
            self.connect('radius_dist.output[%d]'%i, name+'.r')
            self.connect('radius_dist.delta', name+'.dr')
            self.connect('twist_dist.output[%d]'%i, name+'.twist')
            self.connect('chord_dist.output[%d]'%i, name+".chord")

            self.connect('B', name+'.B')
            self.connect('rpm', name+'.rpm')

            self.connect('free_stream.rho', name+'.rho')
            self.connect('free_stream.V', name+'.V_inf')
            self.connect(name+'.delta_Ct', 'perf.delta_Ct[%d]'%i)

            self.connect(name+'.delta_Cp', 'perf.delta_Cp[%d]'%i)
            self.connect(name+'.lambda_r', 'perf.lambda_r[%d]'%i)

        self.driver.workflow.add('perf')




if __name__ == "__main__":
    
    top = Assembly()
    top.add('b', BEM())
    top.driver.workflow.add('b')

    top.run()
    print 
    print 'Cp: ', top.b.data[3]
    print 'rpm: ',top.b.rpm
    print 'top.b.chord_hub: ', top.b.chord_hub
    print 'top.b.chord_tip: ', top.b.chord_tip
    print 'top.b.twist_hub: ', top.b.twist_hub
    print 'top.b.twist_tip: ', top.b.twist_tip

    
    
    #from openmdao.lib.drivers.api import SLSQPdriver
    #from openmdao.lib.casehandlers.api import DumpCaseRecorder
    #top.add('driver', SLSQPdriver())
    #top.driver.add_parameter('b.chord_hub', low=.5, high=2)
    #top.driver.add_parameter('b.chord_tip', low=.5, high=2)
    #top.driver.add_parameter('b.twist_hub', low=-5, high=50)
    #top.driver.add_parameter('b.twist_tip', low=-5, high=50)
    #top.driver.add_parameter('b.rpm', low=10, high=200)
    #top.driver.add_parameter('b.r_tip', low=1, high=10)
    ##top.driver.recorders =[DumpCaseRecorder()]

    #top.driver.add_objective('-b.data[3]')

    #top.run()
    #print 
    #print 'Cp: ', top.b.data[3]
    #print 'rpm: ',top.b.rpm
    #print 'top.b.chord_hub: ', top.b.chord_hub
    #print 'top.b.chord_tip: ', top.b.chord_tip
    #print 'top.b.twist_hub: ', top.b.twist_hub
    #print 'top.b.twist_tip: ', top.b.twist_tip
    




