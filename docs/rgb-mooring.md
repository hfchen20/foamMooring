# rigidBodyMooring

## Overview

The mooring restraint `librigidBodyMooring.so` enhances the native rigid body motion library `rigidBodyMotion`, which solves multi-body dynamics using the articulated-body algorithm (Featherstone, 2014). The restraint library can be simply loaded at runtime into the built-in OpenFOAM solvers `interFoam` and `overInterDyMFoam`, or into other variants (such as `waveFoam` and `olaFlow`) developed by the community. 

While the built-in six-DoF motions solver calculates the body motion one by one, this multibody dynamics solver calculates the motion responses of all bodies together. This algorithm is inherently applicable to simulations of interconnected bodies and multiple hinged bodies.

- After the hydrodynamic forces/moments acting on all the bodies are calculated, the reacting forces/moments by all restraints are added up to the body each of the restraint is attached to. 
- The forward dynamics algorithm then calculates the joint acceleration from the joints state and forces. 
- The joints velocity and position are then integrated using the Newmark-β scheme. 
- The bodies state is finally updated to correspond to the current joints state. 

## Mooring restraints for interconnected bodies

For the `rigidBodyMotion` restraints, only the point coupling mode is valid as the mooring restraining moments requested by the multibody dynamics formulation should be in global coordinate system. Multiple bodies interconnected with shared moorings or hinges can be simulated.

- map3R
- moorDynR1, moorDynR2
- moodyR
- linearSpringGroup

## How to use rigidBodyMooring
- Prepare an OpenFOAM case as usual. The floating body motion can be accommodated by either deforming mesh `interFoam` or overset grid `overInterDyMFoam`.

- Add in `controlDict`
```
libs    (rigidBodyMooring); 
```
- Prepare a mooring input file in case subfolder "Mooring". 

- Define mooring restraints in `constant/dynamicMeshDict`
```
// Example mooring restraints as defined in librigidBodyMooring, add one of
//	moorDynR1 || moorDynR2 || map3R || moodyR 
```
