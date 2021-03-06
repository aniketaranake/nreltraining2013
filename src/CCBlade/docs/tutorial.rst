.. _tutorial-label:

.. currentmodule:: ccblade

Tutorial
--------

Three examples are shown below.  The first is a complete setup for the :ref:`NREL 5-MW model <5MW-example>`, the second shows how to model blade :ref:`precurvature <curvature-example>`, and the final shows how to get the provided :ref:`analytic gradients <gradient-example>`.

.. _5MW-example:

NREL 5-MW
^^^^^^^^^

One example of a CCBlade application is the simulation of the NREL 5-MW reference model's aerodynamic performance.  First, define the geometry and atmospheric properties.

.. literalinclude:: examples/example.py
    :start-after: # 1 ---
    :end-before: # 1 ---

Airfoil aerodynamic data is specified using the :class:`CCAirfoil` class.  Rather than use the default constructor, this example uses the special constructor designed to read AeroDyn files directly :meth:`CCAirfoil.initFromAerodynFile`.

.. literalinclude:: examples/example.py
    :start-after: # 2 ---
    :end-before: # 2 ---


Next, construct the CCBlade object.

.. literalinclude:: examples/example.py
    :start-after: # 3 ---
    :end-before: # 3 ---


Evaluate the distributed loads at a chosen set of operating conditions.

.. literalinclude:: examples/example.py
    :start-after: # 4 ---
    :end-before: # 4 ---


Plot the flapwise and lead-lag aerodynamic loading

.. literalinclude:: examples/example.py
    :start-after: # 5 ---
    :end-before: # 5 ---


as shown in :num:`Figure #distributed-fig`.

.. _distributed-fig:

.. figure:: /images/distributedAeroLoads.*
    :width: 5in
    :align: center

    Flapwise and lead-lag aerodynamic loads along blade.


To get the power, thrust, and torque at the same conditions (in both absolute and coefficient form), use the :meth:`evaluate <ccblade.CCBlade.evaluate>` method.  This is generally used for generating power curves so it expects ``array_like`` input.  For this example a list of size one is used.

.. literalinclude:: examples/example.py
    :start-after: # 6 ---
    :end-before: # 6 ---


The result is

>>> CP = [ 0.46488096]
>>> CT = [ 0.76926398]
>>> CQ = [ 0.0616323]

Note that the outputs are numpy arrays (of length 1 for this example).  To generate a nondimensional power curve (:math:`\lambda` vs :math:`c_p`):

.. literalinclude:: examples/example.py
    :start-after: # 7 ---
    :end-before: # 7 ---


:num:`Figure #cp-fig` shows the resulting plot.

.. _cp-fig:

.. figure:: /images/cp.*
    :width: 5in
    :align: center

    Power coefficient as a function of tip-speed ratio.


CCBlade provides a few additional options in its constructor.  The other options are shown in the following example with their default values.

.. code-block:: python

    # create CCBlade object
    rotor = CCBlade(r, chord, theta, af, Rhub, Rtip, B, rho, mu,
                    precone, tilt, yaw, shearExp, hubHt, nSector
                    tiploss=True, hubloss=True, wakerotation=True, usecd=True, iterRe=1)

The parameters :code:`tiploss` and :code:`hubloss` toggle Prandtl tip and hub losses repsectively. The parameter :code:`wakerotation` toggles wake swirl (i.e., :math:`a^\prime = 0`).  The parameter :code:`usecd` can be used to disable the inclusion of drag in the calculation of the induction factors (it is always used in calculations of the distributed loads).  However, doing so may cause potential failure in the solution methodology (see :cite:`Ning2013A-simple-soluti`).  In practice, it should work fine, but special care for that particular case has not yet been examined, and the default implementation allows for the possibility of convergence failure.  All four of these parameters are ``True`` by default.  The parameter :code:`iterRe` is for advanced usage.  Referring to :cite:`Ning2013A-simple-soluti`, this parameter controls the number of internal iterations on the Reynolds number.  One iteration is almost always sufficient, but for high accuracy in the Reynolds number :code:`iterRe` could be set at 2.  Anything larger than that is unnecessary.

.. _curvature-example:

Precurve
^^^^^^^^

CCBlade can also simulate blades with precurve.  This is done by using the ``precurve`` and ``precurveTip`` parameters.  These correspond precisely to the ``r`` and ``Rtip`` parameters.  Precurve is defined as the position of the blade reference axis in the x-direciton of the :ref:`blade-aligned coordinate system <azimuth_blade_coord>` (r is the position in the z-direction of the same coordinate system).  Presweep can be specified in the same manner, by using the ``presweep`` and ``presweepTip`` parameters (position in blade-aligned y-axis).  Generally, it is advsiable to set ``precone=0`` for blades with precurve.  There is no loss of generality in defining the blade shape, and including a nonzero precone would change the rotor diameter in a nonlinear way. As an example, a downwind machine with significant curvature could be simulated using:

.. literalinclude:: examples/precurve.py
    :start-after: # 1 ---
    :end-before: # 1 ---

The shape of the blade is seen in :num:`Figure #shape-fig`.  Note that the radius of the blade is preserved because we have set the precone angle to zero.

.. _shape-fig:

.. figure:: /images/rotorshape.*
    :width: 5in
    :align: center

    Profile of an example (highly) precurved blade.

.. _gradient-example:

Gradients
^^^^^^^^^

CCBlade optinally provides analytic gradients of every output with respect to all design variables.  This is accomplished using a direct method (adjoint is identical because there is only one state variable at each blade section).  Partial derivatives are provided by `Tapenade <http://www-tapenade.inria.fr:8080/tapenade/index.jsp>`_ and hand calculations.  Starting with the previous example for the NREL 5-MW reference model we add the keyword value ``derivatives=True`` in the constructor.

.. literalinclude:: examples/gradients.py
    :start-after: # 3 ---
    :end-before: # 3 ---

Now when we ask for the distributed loads, we also get the gradients.  The gradients are returned as a 2D array and unpacked as follows.

.. literalinclude:: examples/gradients.py
    :start-after: # 5 ---
    :end-before: # 5 ---

For each vector variable (with the exception of precurve) a change in the local quantity affects only the local normal load.  In other words: :math:`dNp_i/dr_j = 0 \text{ for all } i \neq j`.  Thus only the diagonal terms are returned in a 1D vector (``dNp_dr[3] =`` :math:`dNp_3 / dr_3`).  Precurve affects the sections adjacent to it, so the returned matrix is tridiagonal.

We can compare against finite differencing as follows (with a randomly chosen station along the blade):

.. literalinclude:: examples/gradients.py
    :start-after: # 7 ---
    :end-before: # 7 ---

The output is:

>>> (analytic) dNp_i/dr_i = 107.680395266
>>> (fin diff) dNp_i/dr_i = 107.680370762

Similarly, when we compute thrust, torque, and power we also get the gradients (for either the nondimensional or dimensional form).  The gradients are returned as a multidimensional arrays.  The first index corresponds to the different input cases.  In the following example, we use only one input case (Uinf is a 1D array) so the first index is 0.  The derivatives with respect to scalar quantities and with respect to vector quantities are returned separately.

.. literalinclude:: examples/gradients.py
    :start-after: # 6 ---
    :end-before: # 6 ---

Let us compare the derivative of power against finite differencing for one of the scalar quantities (precone):

.. literalinclude:: examples/gradients.py
    :start-after: # 8 ---
    :end-before: # 8 ---

>>> (analytic) dP/dprecone = -4585.70730211
>>> (fin diff) dP/dprecone = -4585.71072668


Finally, we compare the derivative of power against finite differencing for one of the vector quantities (r) at a random index:

.. literalinclude:: examples/gradients.py
    :start-after: # 9 ---
    :end-before: # 9 ---

>>> (analytic) dP/dr_i = 848.368082036
>>> (fin diff) dP/dr_i = 848.355994992
