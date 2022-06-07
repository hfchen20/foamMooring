## Dynamic mooring restraints for sixDoFRigidBodyMotion in OpenFOAM
CFD simulation of floating body motion with mooring dynamics: Coupling MoorDyn with OpenFOAM

![](tutorial/Animation_overset3d_h12t20.mp4)

## How to use
- Download the repo in $WM_PROJECT_USER_DIR, run Allwmake. MAP++ may require other dependent libraries.
- Add in controlDict:  libs    (sixDoFMooring); 
- Prepare mooring input file in folder "Mooring" 
- Define mooring restraints sixDoFRigidBodyMotionCoeffs in dynamicMeshDict


![Three mooring line codes](tutorial/comparison_3_mooring_codes.PNG)

## Reference
[Haifei Chen, Matthew Hall (2022) CFD simulation of floating body motion with mooring dynamics: Coupling MoorDyn with OpenFOAM,
Applied Ocean Research, 124, 103210, https://doi.org/10.1016/j.apor.2022.103210.](https://www.sciencedirect.com/science/article/pii/S0141118722001511)

## About OpenFOAM
OpenFOAM is a free, open source CFD software [released and developed by OpenCFD Ltd since 2004](http://www.openfoam.com/history/).
It has a large user base across most areas of engineering and science, from both commercial and academic organisations.
OpenFOAM has an extensive range of features to solve anything from complex fluid flows involving chemical reactions, turbulence and heat transfer, to acoustics, solid mechanics and electromagnetics.
[See documentation](http://www.openfoam.com/documentation)
