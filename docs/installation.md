## Prerequisites
- [git](https://git-scm.com/), to clone foamMooring and submodules (MoorDyn v1 & v2).
- Make, to compile MoorDyn and MAP++.
- [CMake](https://cmake.org/), to build MoorDyn v2.
- [VTK](https://vtk.org/) if [USE_VTK=ON](https://gitlab.com/hfchen20/foamMooring/-/merge_requests/3) when compiling MoorDyn v2. 
```
# Ubuntu 20.04
sudo apt install libvtk7-dev
# Ubuntu 22.04 or newer
sudo apt install libvtk9-dev
```

- [lapacke](https://www.netlib.org/lapack/lapacke.html), (optional) required by MAP++ only.
```
sudo apt install liblapack3
sudo apt install liblapack-dev liblapacke-dev
```

## Compile foamMooring
- Clone the repo in `$WM_PROJECT_USER_DIR`.
```
mkdir -p $WM_PROJECT_USER_DIR
cd $WM_PROJECT_USER_DIR
git clone https://gitlab.com/hfchen20/foamMooring.git
cd foamMooring 
```

- Run `Allwmake`. Upon successful compilation, there should be at least two libraries, `libsixDoFMooring.so` and `librigidBodyMooring.so`, in `$FOAM_USER_LIBBIN`.
```
./Allwmake
```

## Use of MAP++
The quasi-static mooring model *MAP++* is not compiled by default. To use the related mooring restraints `map3R`,

- Remove the comment sign `#` in `Allwmake` to compile MAP++.
```
(cd map-plus-plus/src; make && cp libmap-1.30.00.so $FOAM_USER_LIBBIN)
```

- Add the corresponding entries in the Make files.

For `Make/files`, add
```
map3R/mapFoamInterface.C
map3R/map3R.C
```
Replace `Make/options` with [`Make/options-map3`](https://gitlab.com/hfchen20/foamMooring/-/blob/master/src/sixDoFMooringRestraints/Make/options-map3?ref_type=heads) which includes the headers and libraries needed by `map3R`, such as
```
-llapacke \
-lmap-1.30.00 \
```
