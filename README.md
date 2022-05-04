## Dynamic mooring restraints for sixDoFRigidBodyMotion in OpenFOAM
CFD simulation of floating body motion with mooring dynamics: Coupling MoorDyn with OpenFOAM

## How to use
Download the repo in $WM_PROJECT_USER_DIR, run Allwmake
Add in controlDict:  libs    (sixDoFMooring); 
Define mooring restraints sixDoFRigidBodyMotionCoeffs in dynamicmeshDict

## About OpenFOAM
OpenFOAM is a free, open source CFD software [released and developed by OpenCFD Ltd since 2004](http://www.openfoam.com/history/).
It has a large user base across most areas of engineering and science, from both commercial and academic organisations.
OpenFOAM has an extensive range of features to solve anything from complex fluid flows involving chemical reactions, turbulence and heat transfer, to acoustics, solid mechanics and electromagnetics.
[See documentation](http://www.openfoam.com/documentation)
