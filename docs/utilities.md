# Useful tips

## Visualize mooring lines
Two options are available to visualize the mooring lines simulated by MoorDyn: 

1. post-processing MoorDyn lines output after runtime, and 
2. generating VTK files (legacy format or XML format) during runtime. 

### Legacy VTK
Detailed steps for option 1 are:

- Write VTK files `mooringN.vtk` for mooring lines where `N` denotes a time sequence number.
- Prepare a `vtk.series` file `mooring.vtk.series` to be loaded into Paraview.
- A [Python script](https://gitlab.com/hfchen20/foamMooring/-/blob/master/tutorial/visualize_moorings_in_paraview/postMoorDyn_VTK.py) and example VTK files are provided in the tutorial to post-process MoorDyn output.
- The mooring tension could also be added to the VTK files, if tension output is enabled in MoorDyn input file.

Option 2 (legacy VTK) requires certain entries in each mooring restraint's definition in `dynamicMeshDict`.

- The most important entry is `outerCorrector`, which should be consistent with `PIMPLE` settings in `system/fvSolution`.
- A VTK file is written during the last iteration of the outer loop, per OpenFOAM's `writeInterval` defined in `system/controlDict`.
- At the end of the simulation, a `vtk.series` file is generated automatically for visualization in ParaView.

```
writeMooringVTK    true; // optional, default false
vtkPrefix         "mdv2_pt"; // optional, default depends on the underlying mooring model
vtkStartTime       0; // optional, default 0
outerCorrector     3; // optional, default 3
```

!!! note
    Runtime generation of legacy VTK files (including vtk.series) is now supported for all mooring restraints, except for moodyR.
    
    MoorDyn v2 has a built-in functionality to write XML-based [VTK files](https://gitlab.com/hfchen20/foamMooring/-/merge_requests/3).
    
### XML format VTK

Option 2 (XML format VTK) requires installation of [`vtk` library](https://gitlab.com/hfchen20/foamMooring/-/merge_requests/3) and `USE_VTK=ON` when compiling MoorDyn v2. The updated `.Allwmake` script first tries to compiling MoorDyn v2 with VTK support. If an error is detected, it reverts to a compiling without VTK support.


## Extract rigid body motions
- Add functionObjects in `controlDict` to record motion history at runtime.
- After simulation is complete, run bash script to extract all motion info from solvers' log files `log.interFoam` or `log.overInterDyMFoam`.
- Custom code

### functionObjects
```
sixDoF_History
{
    type           sixDoFRigidBodyState;
    libs           ("libsixDoFRigidBodyState.so");
    angleFormat    radians; //or degrees
    writeControl   timeStep;
    writeInterval  5;
}
```

### Bash script
```
#!/bin/bash
#
# How-to use:
# Assuming a fixed number of outer iterations per time step!
# $ ./extractMulti.sh log.interFoam

file=$1

# Count the outer corrector
NCOM=$(cat $file | grep "Centre of mass" | wc -l)
NTIME=$(cat $file | grep "^Time = " | wc -l)
NOUTER=$((NCOM / NTIME))
echo "nOuterCorrector = $NCOM / $NTIME = $NOUTER"

# Load the data on separate files
grep -e "^Time = " $file | cut -d " " -f 3 > logs/times
grep "Centre of mass" $file | cut -d ":" -f 2 | tr -d "()" > logs/cM
grep "Linear velocity" $file | cut -d ":" -f 2 | tr -d "()" > logs/lV
grep "Angular velocity" $file | cut -d ":" -f 2 | tr -d "()" > logs/aV
grep "Orientation" $file  | cut -d ":" -f 2 | tr -d "()" > logs/orientation

# Split the files with data repeated each outer corrector
for ((i = 1 ; i <= $NOUTER ; i++)); do
	cat logs/cM | awk "(NR-$i) % $NOUTER == 0" > logs/cM.$i
	cat logs/lV | awk "(NR-$i) % $NOUTER == 0" > logs/lV.$i
	cat logs/aV | awk "(NR-$i) % $NOUTER == 0" > logs/aV.$i
	cat logs/orientation | awk "(NR-$i) % $NOUTER == 0" > logs/orientation.$i
done

paste logs/times logs/cM.* > logs/t_vs_CoM
paste logs/times logs/lV.* > logs/t_vs_linearV
paste logs/times logs/aV.* > logs/t_vs_angularV
paste logs/times logs/orientation.* > logs/t_vs_orientation

rm logs/times logs/cM* logs/lV* logs/orientation* logs/aV*

```