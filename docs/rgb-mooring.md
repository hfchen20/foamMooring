# rigidBodyMooring

## Overview

The mooring restraint `librigidBodyMooring.so` enhances the native rigid body motion library `rigidBodyMotion`, which solves multi-body dynamics using the articulated-body algorithm (Featherstone, 2014). The restraint library can be simply loaded at runtime into the built-in OpenFOAM solvers `interFoam` and `overInterDyMFoam`, or into other variants (such as `waveFoam` and `olaFlow`) developed by the community. 

While the built-in six-DoF motions solver calculates the body motion one by one, this multibody dynamics solver calculates the motion responses of all bodies together. This algorithm is inherently applicable to simulations of interconnected bodies and multiple hinged bodies.

- After the hydrodynamic forces/moments acting on all the bodies are calculated, the reacting forces/moments by all restraints are added up to the body each of the restraint is attached to. 
- The forward dynamics algorithm calculates the joint acceleration from the joints state and forces. 
- The joints velocity and position are then integrated using the Newmark-β scheme. 
- The bodies state is finally updated to correspond to the current joints state. 

## Coupling mode for interconnected bodies

For the `rigidBodyMotion` restraints, only the point coupling mode is valid at the time of writing as the mooring restraining moments requested by the multibody dynamics formulation should be in global coordinate system. Multiple bodies interconnected with shared moorings or hinges can be simulated. Available restraints include

- map3R
- moorDynR1, moorDynR2
- moodyR
- linearSpringGroup

!!! note
    Body coupling mode for rigidBodyMooring has been fixed by Jose Luis Cercos-Pita.

## How to use rigidBodyMooring
- Prepare an OpenFOAM case as usual. The floating body motion can be accommodated by either deforming mesh `interFoam` or overset grid `overInterDyMFoam`.

- Add in `controlDict`
```
libs    (rigidBodyMooring); 
```
- Prepare a mooring input file in case subfolder "Mooring". 

- Define mooring restraints in `constant/dynamicMeshDict`. Add one of `moorDynR1 || moorDynR2 || map3R || moodyR || linearSpringGroup`.

=== "MAP++"

    ```
    map3R
    {
        type                map3R;
        body                floatingObject;
        inputFile           "Mooring/esflOWC_4lines.map";
        summaryFile         "Mooring/esflOWC_summary.map";
        waterDepth          0.5;

        refAttachmentPt
        (
            (-0.1      0.1    -0.062)
            (-0.1     -0.1    -0.062)
            ( 0.1      0.1    -0.062)
            ( 0.1     -0.1    -0.062)
        );

        writeMooringForces     true;
        outputFile             "Mooring/mapForces.dat";
        writeMooringVTK        true;
        //nNodes                 10;
        //nodesPerLine           (10 10 10 10);
    }
    ```

=== "MoorDyn"

    ```
    moorDynR1
    {
        type               moorDynR1;
        body               floatingObject;
        refAttachmentPt
        (
            (-0.1      0.1    -0.062)
            (-0.1     -0.1    -0.062)
            ( 0.1      0.1    -0.062)
            ( 0.1     -0.1    -0.062)
        );
    }

    moorDynR2_pt
    {
        type               moorDynR2;
        body               floatingObject;
        couplingMode       "POINT";
        inputFile          "Mooring/lines_v2_point.txt";
        refAttachmentPt
        (
            (-0.25      0.3725    -0.0652)
            (-0.25     -0.3725    -0.0652)
            ( 0.25      0.3725    -0.0652)
            ( 0.25     -0.3725    -0.0652)  
        );
        writeMooringVTK    true;
        vtkPrefix          "mdv2_pt";
        vtkStartTime       0;
        outerCorrector     3;
    }

    moorDynR2_bd
    {
        type               moorDynR2;
        body               floatingObject;
        couplingMode       "BODY";
        inputFile          "Mooring/lines_v2.txt";
        bodies
        (
            floatingObject
        );
        writeMooringVTK    true;
        vtkPrefix          "mdv2_bd";
        vtkStartTime       0;
        outerCorrector     3;
    }

    ```

=== "Moody"

    ```
    moodyR
    {
        type               moodyR;
        body               floatingObject;

        inputFile         "Mooring/Liang_typeC.m";
        refAttachmentPt
        (
            (-0.25      0.3725    -0.0652)
            (-0.25     -0.3725    -0.0652)
            ( 0.25      0.3725    -0.0652)
            ( 0.25     -0.3725    -0.0652)    
        );
    }
    ```

=== "linearSpringGroup"

    ```
    linearSpringGroup
    {
        type                linearSpringGroup;
        body                box;

        anchor
        (
            (-5.8646  5.9 0.0779)
            (-5.8646 -5.9 0.0779)
            ( 5.9354  5.9 0.0779)
            ( 5.9354 -5.9 0.0779)
        );

        refAttachmentPt
        (
            (-0.435  0.435 0)
            (-0.435 -0.435 0)
            ( 0.435  0.435 0)
            ( 0.435 -0.435 0)
        );

        numberOfSprings        4;

        // if true, specify a scalar for stiffness, damping, restLength
        // otherwise, specify a scalarList for each spring (x, x, x, x)
        identicalSprings      true;
        stiffness             28;
        damping               0;
        restLength            7.014391404;

        writeForce            true;
        writeVTK              true;
        compression           false; // false = no spring force when compressed
        frelax                0.8;
    }
    ```

## Multiple moored bodies

- Use keyword `bodies` to specify all the bodies each point in `refAttachmentPt` is attached to. Order matters. 
- Support interconnected bodies (shared moorings).

=== "MAP++"
    ```
    map3R
    {
        type               map3R;
        body               box1;

        bodies             (box1 box1 box1 box1 box2 box2 box2 box2);

        inputFile          "Mooring/twinFB.map";
        summaryFile        "Mooring/twinFB_summary.map";
        outputFile         "Mooring/mapForces.map";
        waterDepth         0.514;
        writeMooringForces true;
        writeMooringVTK    true;
        vtkStartTime       0;
        outerCorrector     3;
        nNodes             6; //# nodes for all lines
        nodesPerLine       (12 12 12 12 12 12 12 12);

        refAttachmentPt
        (
            (-0.25      0.3725    -0.0729)
            (-0.25     -0.3725    -0.0729)
            ( 0.25      0.3725    -0.0729)
            ( 0.25     -0.3725    -0.0729)

            (-0.25      0.3725    -0.0729)
            (-0.25     -0.3725    -0.0729)
            ( 0.25      0.3725    -0.0729)
            ( 0.25     -0.3725    -0.0729)
        );
    }
    ```
=== "MoorDyn"
    ```
    moorDynR2_point
    {
        type               moorDynR2;
        body               box1;

        couplingMode       "POINT";

        inputFile          "Mooring/lines_v2_pointCoupling.txt";
        bodies             (box1 box1 box1 box1 box2 box2 box2 box2);

        refAttachmentPt
        (
            (-0.25      0.3725    -0.0729)
            (-0.25     -0.3725    -0.0729)
            ( 0.25      0.3725    -0.0729)
            ( 0.25     -0.3725    -0.0729)

            (-0.25      0.3725    -0.0729)
            (-0.25     -0.3725    -0.0729)
            ( 0.25      0.3725    -0.0729)
            ( 0.25     -0.3725    -0.0729)
        );

        writeMooringVTK    true;
        vtkStartTime       0;
        outerCorrector     3;
    }
    ```

=== "Moody"
    ```
    moodyR
    {
        type               moodyR;
        body               box1;

        bodies            (box1 box1 box1 box1 box2 box2 box2 box2);

        inputFile         "Mooring/Chen2022_twinFB.m";
        refAttachmentPt
        (
            (-0.25      0.3725    -0.0729)
            (-0.25     -0.3725    -0.0729)
            ( 0.25      0.3725    -0.0729)
            ( 0.25     -0.3725    -0.0729)

            (-0.25      0.3725    -0.0729)
            (-0.25     -0.3725    -0.0729)
            ( 0.25      0.3725    -0.0729)
            ( 0.25     -0.3725    -0.0729)
        );
    }
    ```