/*-------------------------------------------------------------------------------------*\
Class
    mapFoamInterface

Description
    Interface class between quasi-static mooring code MAP++ and map3R (sixDoFRigidBodyMotionRestraints).

SourceFiles
    mapFoamInterface.C

Author:
    Haifei Chen
    
    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with mooring dynamics: 
        Coupling MoorDyn with OpenFOAM. Applied Ocean Research, 124, 103210.
        https://doi.org/10.1016/j.apor.2022.103210
        
\*-------------------------------------------------------------------------------------*/

#ifndef mapFoamInterface_H
#define mapFoamInterface_H


//#include <string>


//using namespace std;

extern "C" {

#include "../../map-plus-plus/src/map.h"
#include "../../map-plus-plus/src/mapinit.h"
#include "../../map-plus-plus/src/outputstream.h"
#include "../../map-plus-plus/src/mapapi.h"
#include "../../map-plus-plus/src/freedata.h"

}

//NOTE: including C functions into C++

/*
#ifdef __cplusplus
extern "C"
{
#endif

#include "map.h"
#include "mapinit.h"
#include "outputstream.h"
#include "mapapi.h"
#include "freedata.h"

#ifdef __cplusplus
}
#endif
*/

//namespace Foam
//{


// Class mapFoamInterface 
class mapFoamInterface
{

    // Private members
    MAP_InitInputType_t*     _init_type;
    MAP_InitOutputType_t*      _io_type;
    MAP_InputType_t*            _u_type;
    MAP_ParameterType_t*        _p_type;
    MAP_ConstraintStateType_t*  _z_type;
    MAP_ContinuousStateType_t*  _x_type;
    MAP_OutputType_t*           _y_type;
    MAP_OtherStateType_t*   _other_type;

    MAP_ERROR_CODE success;  
    MAP_ERROR_CODE ierr;

    char map_msg[MAP_ERROR_STRING_LENGTH];

    // Private member functions
    void read_input_file(std::string inputFile);


public:   
    //- Runtime type information
    //TypeName("mapFoamInterface");
    
    // Constructor
    mapFoamInterface();

    // Destructor
    ~ mapFoamInterface();

    void initMAP(std::string inputFile, std::string summaryFile, double depth, double g, double rho);

    // Update fairlead positions and return fairlead force
    // both arrays of size (nLines x 3)
    void updateMAP(double time, double fPosition[], double fforce[]);

    // Free memory
    void closeMAP();

    // return # of lines
    int getNLines();

    // return node coordinates [x,y,z] for line i
    void getNodeCoordinates(int li, int N, double coord[]);

    // return fairlead force for line i
    void getFairleadForce(int li, double f[], double attachPosition[]);
  
};

//} // End namespace Foam

#endif
