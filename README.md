## Dynamic mooring restraints for rigid body motion in OpenFOAM
CFD simulation of floating body motion with mooring dynamics: Coupling MoorDyn with OpenFOAM

![One floater](tutorial/misc/Animation_overset3d_h12t20.mp4){width=400 height=320px}
![Two floaters](tutorial/misc/twoBody_moored.mp4){width=400 height=320px}

## Compile foamMooring
Prerequisites: git, make, cmake. MAP++ may require other dependent libraries, such as `lapacke`.
- Clone the repo in `$WM_PROJECT_USER_DIR`.
```
mkdir -p $WM_PROJECT_USER_DIR 
cd $WM_PROJECT_USER_DIR 
git clone https://gitlab.com/hfchen20/foamMooring.git 
cd foamMooring 
```
- Run Allwmake. Upon successful compilation, there should be at least a `libsixDoFMooring.so` library in `$FOAM_USER_LIBBIN`.
```
./Allwmake
```

- If there is difficulty in compiling Map++ and map3R (quasi-static mooring code), you could remove or comment out the corresponding entries in the Make files. For [files](/src/sixDoFMooringRestraints/Make/files), remove
```
map3R/mapFoamInterface.C
map3R/map3R.C
```
For [options](/src/sixDoFMooringRestraints/Make/options), remove
```
-llapacke \
-lmap-1.30.00 \
```

## How to use (tested on v2012, v2212)
- Prepare an OpenFOAM case as usual. The floating body motion can be accommodated by either deforming mesh `interFoam` or overset grid `overInterDyMFoam`.
- Add in `controlDict`
```
libs    (sixDoFMooring); 
```
- Prepare a mooring input file in case subfolder "Mooring". For example,
   - MoorDyn v1: [lines.txt](tutorial/sixDoF_2D/overset/background/Mooring) (filename hard-coded)

   - MoorDyn v2: [lines_v2.txt](tutorial/sixDoF_2D/overset/background/Mooring)

   - Moody: [boxWu_exPoint.m](tutorial/sixDoF_2D/overset/background/Mooring)

   - MAP++: [esflOWC_4lines.map](tutorial/sixDoF_2D/overset/background/Mooring)

- Define mooring restraints in `constant/dynamicMeshDict`
```
// Example mooring restraints as defined in libsixDoFMooring, add one of
//	moorDynR1 || moorDynR2 || map3R || moodyR 

moorDynR1
{
	sixDoFRigidBodyMotionRestraint moorDynR1;
}

moorDynR2
{
	sixDoFRigidBodyMotionRestraint moorDynR2;
	inputFile                      "Mooring/lines_v2.txt";
	writeMooringVTK                true;

}

map3R
{
	sixDoFRigidBodyMotionRestraint map3R;
	inputFile                     "Mooring/esflOWC_4lines.map";
	summaryFile                   "Mooring/esflOWC_summary.map";
	waterDepth                    0.5;
	refAttachmentPt
	(
		(-0.1      0.1    -0.077)
		(-0.1     -0.1    -0.077)
		( 0.1      0.1    -0.077)
		( 0.1     -0.1    -0.077)
	);
	numberOfSegments       20;
	writeMooringVTK        true;
}

moodyR
{
	sixDoFRigidBodyMotionRestraint moodyR;
	inputFile              "Mooring/boxWu_exPoint.m";

	couplingMode           "externalPoint";  // "externalPoint" or "externalRigidBody"
	//If couplingMode is "externalPoint", nCouplingDof = 3*refAttachmentPt.size()
	nCouplingDof           6;
	refAttachmentPt
	(
		(-0.1      0.1    -0.077)
		(-0.1     -0.1    -0.077)
		( 0.1      0.1    -0.077)
		( 0.1     -0.1    -0.077)
	);
	waveKinematics         false;
	twoD                   true;
}
```

## Main features
![Three mooring line codes](tutorial/misc/comparison_3_mooring_codes.PNG)

## Visualize mooring lines in Paraview
- Write VTK files 'mooringN.vtk' for mooring lines where N denotes a time sequence number.
- Prepare a vtk.series file 'mooring.vtk.series' to be loaded into Paraview.
- A python script and example VTK files are provided in the tutorial to post-process MoorDyn output.
- The mooring tension could also be added to the VTK files.


## Reference
[Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with mooring dynamics: Coupling MoorDyn with OpenFOAM,
Applied Ocean Research, 124, 103210. https://doi.org/10.1016/j.apor.2022.103210](https://www.sciencedirect.com/science/article/pii/S0141118722001511)

## About OpenFOAM
OpenFOAM is a free, open source CFD software [released and developed by OpenCFD Ltd since 2004](http://www.openfoam.com/history/).
It has a large user base across most areas of engineering and science, from both commercial and academic organisations.
OpenFOAM has an extensive range of features to solve anything from complex fluid flows involving chemical reactions, turbulence and heat transfer, to acoustics, solid mechanics and electromagnetics.
[See documentation](http://www.openfoam.com/documentation)
