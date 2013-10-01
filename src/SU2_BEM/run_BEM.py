from BEM import BEM

from openmdao.main.api import Component, Assembly, VariableTree
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree

class blade_opt(Assembly):

    def configure(self):
        
        # Add the bem component to the assembly
        self.add('bem', BEM())

        # Choose SLSQP as the driver and add components to the workflow
        self.add('driver', SLSQPdriver())
        self.driver.workflow.add('bem')

        # Optimization parameters
        self.driver.add_parameter('bem.chord_hub', low=.5, high=2)
        self.driver.add_parameter('bem.chord_tip', low=.5, high=2)
        self.driver.add_parameter('bem.twist_hub', low=-5, high=50)
        self.driver.add_parameter('bem.twist_tip', low=-5, high=50)

        # Constraints
        for i in range(len(self.bem.a_in_array)):
            self.driver.add_constraint('bem.a_in_array[%d]==bem.a_out_array[%d]'%(i,i))
            self.driver.add_constraint('bem.b_in_array[%d]==bem.b_out_array[%d]'%(i,i))
    
        top.driver.add_objective('-bem.data[3]')
        print 'top.b.chord_hub: ', top.b.chord_hub
        print 'top.b.chord_tip: ', top.b.chord_tip
        print 'top.b.twist_hub: ', top.b.twist_hub
        print 'top.b.twist_tip: ', top.b.twist_tip
