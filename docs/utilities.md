# Useful tips

## Visualize mooring lines
Two options are available to visualize the mooring lines simulated by MoorDyn: 1) post-processing MoorDyn lines output after runtime, and 2) generating VTK files during runtime. Detailed steps for option 1 are:

- Write VTK files 'mooringN.vtk' for mooring lines where N denotes a time sequence number.
- Prepare a vtk.series file 'mooring.vtk.series' to be loaded into Paraview.
- A Python script and example VTK files are provided in the tutorial to post-process MoorDyn output.
- The mooring tension could also be added to the VTK files, if tension output is enabled in MoorDyn input file.


!!! note
    Runtime generation of legacy VTK files (including vtk.series) is now supported for all mooring restraints, except for moodyR.
    
    MoorDyn v2 has a built-in functionality to write XML-based [VTK files](https://gitlab.com/hfchen20/foamMooring/-/merge_requests/3).

## Post-process rigid body motions
- functionObjects
- processing script