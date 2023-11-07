# Useful tips

## Visualize mooring lines
Two options are available to visualize the mooring lines simulated by MoorDyn: 

1. post-processing MoorDyn lines output after runtime, and 
2. generating VTK files during runtime. 

Detailed steps for option 1 are:

- Write VTK files `mooringN.vtk` for mooring lines where `N` denotes a time sequence number.
- Prepare a `vtk.series` file `mooring.vtk.series` to be loaded into Paraview.
- A [Python script](https://gitlab.com/hfchen20/foamMooring/-/blob/master/tutorial/visualize_moorings_in_paraview/postMoorDyn_VTK.py) and example VTK files are provided in the tutorial to post-process MoorDyn output.
- The mooring tension could also be added to the VTK files, if tension output is enabled in MoorDyn input file.

Option 2 (legacy VTK) requires certain entries in each mooring restraint's definition in `dynamicMeshDict`.

- The most important entry is `outerCorrector`, which should be consistent with `PIMPLE` settings in `system/fvSolution`.
- A VTK file is written during the last iteration of the outer loop, per OpenFOAM's `writeInterval`.
- At the end of the simulation, a `vtk.series` file is generated for visualization in ParaView.

```
writeMooringVTK    true; // optional, default false
vtkPrefix         "mdv2_pt"; // optional, default depends on the undelying mooring model
vtkStartTime       0; // optional, default 0
outerCorrector     3; // optional, default 3
```

!!! note
    Runtime generation of legacy VTK files (including vtk.series) is now supported for all mooring restraints, except for moodyR.
    
    MoorDyn v2 has a built-in functionality to write XML-based [VTK files](https://gitlab.com/hfchen20/foamMooring/-/merge_requests/3).
    
Option 2 (XML format VTK) requires installation of [`vtk` library](https://gitlab.com/hfchen20/foamMooring/-/merge_requests/3) and `USE_VTK=ON` when compiling MoorDyn v2. This generates VTK files every time step, for the moment.


## Post-process rigid body motions
- functionObjects
- processing script