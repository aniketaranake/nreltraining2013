# Python imports
from math import pi
import numpy as np
from scipy.interpolate import interp1d
from copy import deepcopy

# OpenMDAO imports
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree, File
from openmdao.main.api import Component, Assembly, set_as_top
from SU2_wrapper import Solve, Deform
from SU2_wrapper.SU2_wrapper import ConfigVar, Config


class SU2_CLCD(Component): 
  '''Fake SU2, for debugging purposes. Cl, Cd come from interpolation'''

  config_in = ConfigVar(Config(), iotype='in')
  mesh_file = File(iotype='in') #made up sizes

  alpha = Float(2., iotype="in",desc="angle of attack of the airfoil section", units="deg")

  coefficientOfLift = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")
  coefficientOfDrag = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")

  def execute(self): 

    # Data for interpolation
    alpha_data = np.array([0., 13., 15, 20, 30])
    cl_data    = np.array([0, 1.3, .8, .7, 1.1])
    cd_data    = np.array([0., 1e-2, 0.3, 0.6, 1.])

    # Perform interpolation
    f_cl = interp1d(alpha_data, cl_data, fill_value=0.001, bounds_error=False)
    f_cd = interp1d(alpha_data, cd_data, fill_value=0.001, bounds_error=False)
    self.coefficientOfLift = float(f_cl(self.alpha))
    self.coefficientOfDrag = float(f_cd(self.alpha)) + 1e-5

    print "Fake_SU2: alpha = ", self.alpha, ", cl = ", self.coefficientOfLift, ", cd = ", self.coefficientOfDrag


class _SU2_CLCD(Solve):
  """Calculate the coefficient of lift and the coefficient of drag from SU2"""

  alpha = Float(2., iotype="in",desc="angle of attack of the airfoil section", units="deg")

  coefficientOfLift = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")
  coefficientOfDrag = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")

  def execute(self):

    super(SU2_CLCD, self).execute()
    coefficientOfLift = self.LIFT
    coefficientOfDrag = self.DRAG      

class SU2_CLCD_Sections(Assembly):

  def __init__(self, nElems = 6):
    self.nElems = nElems
    super(SU2_CLCD_Sections, self).__init__()

  def configure(self):

    self.add("alphas",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="in"))
    self.add("cls",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="out"))
    self.add("cds",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="out"))

    for i in range(self.nElems):

      # Create deform and solve objects
      su2def   = "SU2_deform_%d"%i
      su2solve = "SU2_solve_%d"%i
      deform = self.add(su2def  , Deform())
      solve = self.add(su2solve, SU2_CLCD())

      # TODO: inv_NACA0012 used as default, can be changed later. Clean this up
      new_config = Config()
      new_config.read('inv_NACA0012.cfg') 

      deform.config_in = new_config


      # Connect deform and solve objects together
      self.connect('%s.config_out'%su2def, '%s.config_in'%su2solve)
      self.connect('%s.mesh_file' %su2def, '%s.mesh_file'%su2solve)

      # Connect this assembly to solve objects
      self.connect("alphas[%d]"%i, su2solve+".alpha")
      self.connect(su2solve+".coefficientOfLift","cls[%d]"%i)
      self.connect(su2solve+".coefficientOfDrag", "cds[%d]"%i)

      # Add both deform and solve to the workflow
      self.driver.workflow.add(su2def)
      self.driver.workflow.add(su2solve)


if __name__ == "__main__":
  

  a = SU2_CLCD_Sections()

  a.run()

