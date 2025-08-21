## A mooring restraints library for rigid body motions in OpenFOAM

![](https://img.shields.io/badge/OpenFOAM-v2012-brightgreen) 
![](https://img.shields.io/badge/v2212-brightgreen)
![](https://img.shields.io/badge/v2312-brightgreen)

![](https://img.shields.io/badge/MoorDyn-v1,v2-brightgreen) ![](https://img.shields.io/badge/Moody-v2-brightgreen)

Documentation: https://hfchen20.gitlab.io/foamMooring/

GitHub mirror: https://github.com/hfchen20/foamMooring

:star: Consider starring the repository if you find it useful. :star:

## Overview

- Works for two rigid body motion libraries `sixDoFRigidBodyMotion` and `rigidBodyMotion`.
- Mooring models of restraints include MAP++, MoorDyn, Moody, groups of linear springs.
- Most restraints support runtime generation of legacy VTK files (including vtk.series).
- You can compile only part of the library (i.e., certain restraints) if that suits your needs.
- No need to change and re-compile the built-in motion libraries and flow solvers.
- Tested on OpenFOAM v2012 ~ v2312, mostly with overset grid solver `overInterDyMFoam`.
- Should also work with `interFoam` (deforming mesh) and other variants `waveFoam` and `olaFlow`.
- Even `overPimpleDyMFoam` ...

![One floater](docs/video/Animation_overset3d_h12t20.mp4){width=400px height=320px}
![Two floaters](docs/video/twoBody_moored.mp4){width=400px height=320px}
![Two floaters with shared mooring lines](docs/img/twinFB_shared_mooring_tight.jpeg){width=600px height=320px}

<!-- ![Two floaters with shared moorings](docs/img/twinFB_shared_mooring.ogv) -->

## Compile foamMooring

- Install [prerequisites](https://hfchen20.gitlab.io/foamMooring/installation/#prerequisites).

- Clone the repo in `$WM_PROJECT_USER_DIR`.
```
mkdir -p $WM_PROJECT_USER_DIR 
cd $WM_PROJECT_USER_DIR
git clone https://gitlab.com/hfchen20/foamMooring.git
cd foamMooring 
```

The `dynamic-only` branch has been deleted. If you are previously using this branch, change branch and update the code.
```
git status
# if not On branch master
git checkout master
git pull
```

- Run `Allwmake`. Upon successful compilation, there should be at least two libraries in `$FOAM_USER_LIBBIN`: `libsixDoFMooring.so` and `librigidBodyMooring.so`, .
```
./Allwmake
```

## Code structure
![Code structure](docs/img/flowchart_foamMooring.svg)

## How to use

- Prepare an OpenFOAM case as usual. The floating body motion can be accommodated by either deforming mesh `interFoam` or overset grid `overInterDyMFoam`.

- Add in `controlDict`
```
libs    (sixDoFMooring); // or (rigidBodyMooring)
```

- Prepare a mooring input file in case subfolder "Mooring". Mooring models can be MoorDyn v1, v2, Moody, MAP++, and simple springs.

- Define mooring restraints in `constant/dynamicMeshDict`. [sixDoFMooring](https://hfchen20.gitlab.io/foamMooring/six-dof-mooring/) and [rigidBodyMooring](https://hfchen20.gitlab.io/foamMooring/rgb-mooring/) have slightly different restraints.

Refer to [documentation](https://hfchen20.gitlab.io/foamMooring/) for more examples.

## Main features of mooring models

![Three mooring line codes](docs/img/comparison_3_mooring_codes.PNG)

## Visualize mooring lines in Paraview

See [docs](https://hfchen20.gitlab.io/foamMooring/utilities/#visualize-mooring-lines).


## References

- Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with mooring dynamics: Coupling MoorDyn with OpenFOAM,
Applied Ocean Research, 124, 103210. [https://doi.org/10.1016/j.apor.2022.103210](https://www.sciencedirect.com/science/article/pii/S0141118722001511)
- Chen, H., Medina, T. A., & Cercos-Pita, J. L. (2024). CFD simulation of multiple moored floating structures using OpenFOAM: An open-access mooring 
restraints library. Ocean Engineering, 303, 117697. [https://doi.org/10.1016/j.oceaneng.2024.117697](https://doi.org/10.1016/j.oceaneng.2024.117697) 
([preprint](http://dx.doi.org/10.13140/RG.2.2.34206.10569))

## Disclaimer

This offering is not approved or endorsed by OpenCFD Limited, producer and distributor of the OpenFOAM software via www.openfoam.com, and owner of the OPENFOAM® and OpenCFD® trade marks. This offering is not approved or endorsed by any software packages mentioned above or their respective owners, and should not be considered as such.

OpenFOAM is a free, open source CFD software [released and developed by OpenCFD Ltd since 2004](http://www.openfoam.com/history/).
It has a large user base across most areas of engineering and science, from both commercial and academic organisations. [See documentation](http://www.openfoam.com/documentation)

