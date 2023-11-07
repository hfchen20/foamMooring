## Prerequisites
- [git](https://git-scm.com/), to clone foamMooring and submodules (MoorDyn v1 & v2).
- Make, to compile MoorDyn and MAP++.
- [CMake](https://cmake.org/), to help compile MoorDyn v2.
- [VTK](https://gitlab.com/hfchen20/foamMooring/-/merge_requests/3) if USE_VTK=ON when compiling MoorDyn v2. 
- [lapacke](https://www.netlib.org/lapack/lapacke.html), required by MAP++.
```
$ sudo apt install liblapack3
$ sudo apt install liblapack-dev liblapacke-dev
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

- You can selectively compile part of the library. If there is difficulty in compiling MAP++ and map3R (quasi-static mooring code),  you could skip compiling MAP++ in `Allwmake` and remove/comment out the corresponding entries in the Make files.

For `Make/files``, remove
```
map3R/mapFoamInterface.C
map3R/map3R.C
```

For `Make/options``, remove
```
-llapacke \
-lmap-1.30.00 \
```
