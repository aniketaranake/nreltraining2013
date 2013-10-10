# Python imports
import sys
import numpy as np

# Imports for our model
from BEM import BEM
sys.path.append('../SU2_CLCD')
from su2_clcd import SU2_CLCD_Sections

# OpenMDAO imports
from openmdao.main.api import Component, Assembly, VariableTree
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from openmdao.lib.drivers.api import SLSQPdriver, COBYLAdriver

from custom_opt import SLSQPdriver

from openmdao.lib.casehandlers.api import DumpCaseRecorder

# SU^2 imports
from SU2.io import Config

class blade_opt(Assembly):

    def configure(self):
        
        # Add the bem component to the assembly
        self.add('bem', BEM())
        self.add('su2', SU2_CLCD_Sections())

        #self.su2.set_su2_config('inv_NACA0012.cfg')

        # Choose SLSQP as the driver and add components to the workflow
        self.add('driver', SLSQPdriver())
        self.driver.workflow.add(['bem','su2'])

        # Optimization parameters
        #self.driver.add_parameter('bem.chord_hub', low=.5, high=2)
        #self.driver.add_parameter('bem.chord_tip', low=.5, high=2)
        #self.driver.add_parameter('bem.twist_hub', low=-5, high=50)
        #self.driver.add_parameter('bem.twist_tip', low=-5, high=50)

        # Constraints and connections
        for i in range(len(self.bem.a_in_array)):
            self.driver.add_parameter('bem.a_in_array[%d]'%i,low=0, high=1)
            self.driver.add_parameter('bem.b_in_array[%d]'%i,low=0, high=1)
            # Internal to bem
            self.driver.add_constraint('(bem.a_in_array[%d]-bem.a_out_array[%d])**2 < .001'%(i,i))
            self.driver.add_constraint('(bem.b_in_array[%d]-bem.b_out_array[%d])**2 < .001'%(i,i))

            # Between bem and su2
            self.connect('su2.cls[%d]'%i,'bem.cl_array[%d]'%i)
            self.connect('su2.cds[%d]'%i,'bem.cd_array[%d]'%i)

            self.driver.add_parameter('su2.alphas[%d]'%i, low=0.1,high=20)
            self.su2.alphas[i] = 0.01
            self.driver.add_constraint('(bem.alphas[%d]-su2.alphas[%d])**2 < .001'%(i,i))
    
        self.driver.add_objective('-bem.data[3]')

if __name__=="__main__":

    bo = blade_opt()
    bo.run()
    print bo.driver.error_code
    print 'bo.bem.chord_hub: ', bo.bem.chord_hub
    print 'bo.bem.chord_tip: ', bo.bem.chord_tip
    print 'bo.bem.twist_hub: ', bo.bem.twist_hub
    print 'bo.bem.twist_tip: ', bo.bem.twist_tip
    print 'bo.bem.data[3] (Cp):   ', bo.bem.data[3]
    print 'bo.bem.alphas:   ', bo.bem.alphas
    print 'bo.su2.alphas:   ', bo.su2.alphas