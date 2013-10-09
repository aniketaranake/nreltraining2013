# Python imports
from math import pi
import numpy as np
from scipy.interpolate import interp1d
from copy import deepcopy

# OpenMDAO imports
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from openmdao.main.api import Component, Assembly
from SU2_wrapper import Solve, Deform

# SU2 imports
from SU2.io import Config

class SU2_CLCD(Solve):
  
  """Calculate the coefficient of lift and the coefficient of drag from SU2"""

  kind = "Fake_SU2"
  alpha = Float(2., iotype="in",desc="angle of attack of the airfoil section", units="deg")

  coefficientOfLift = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")
  coefficientOfDrag = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")

  def __init__(self):
    super(Solve,self).__init__()

  def execute(self):
    if self.kind == "Fake_SU2":

      alpha_data = np.array([0., 13., 15, 20, 30])
      cl_data    = np.array([0, 1.3, .8, .7, 1.1])
      cd_data    = np.array([0., 1e-2, 0.3, 0.6, 1.])

      f_cl = interp1d(alpha_data, cl_data, fill_value=0.001, bounds_error=False)
      f_cd = interp1d(alpha_data, cd_data, fill_value=0.001, bounds_error=False)
      self.coefficientOfLift = float(f_cl(self.alpha))
      self.coefficientOfDrag = float(f_cd(self.alpha)) + 1e-5

      #self.coefficientOfLift = 2 * pi * self.alpha
      #self.coefficientOfDrag = .00002 * pi * pi * self.alpha


      print "Fake_SU2: alpha = ", self.alpha, ", cl = ", self.coefficientOfLift, ", cd = ", self.coefficientOfDrag
    if self.kind == "SU2":
      super(SU2_CLCD, self).execute()
      coefficientOfLift = self.LIFT
      coefficientOfDrag = self.DRAG      

class SU2_CLCD_Sections(Assembly):

  def __init__(self, nElems = 6):
    super(SU2_CLCD_Sections, self).__init__()
    self.nElems = nElems
    self.add("alphas",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="in"))
    self.add("cls",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="out"))
    self.add("cds",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="out"))

  def configure(self):
    for i in range(self.nElems):

      # Create deform and solve objects
      su2def   = "SU2_deform_%d"%i
      su2solve = "SU2_solve_%d"%i
      self.add(su2def  , Deform())
      self.add(su2solve, SU2_CLCD())

      # TODO: Clean this up
      new_config = Config()
      new_config.read('inv_NACA0012.cfg') 

      execcmd = 'self.%s.config_in=deepcopy(new_config)'%su2def
      print execcmd
      exec(compile(execcmd,'<string>','exec'))

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

  def set_su2_config(self, filename):
    '''Routine to specify an SU^2 config file'''
    for i in range(self.nElems):

      # Name of the i'th su2 deform object
      su2def = "SU2_deform_%d"%i

      # Create a new config option and read the file into it
      new_config = Config()
      new_config.read(filename)

      execcmd = 'self.%s.config_in=new_config'%(su2def)
      print execcmd
      exec(compile(execcmd,'<string>','exec'))


if __name__ == "__main__":
  print "Hello, world!"

