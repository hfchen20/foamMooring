#include <iostream>
#include <fstream>

#include "mapFoamInterface.H"

//using namespace std;

//namespace Foam
//{

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

//defineTypeNameAndDebug(mapFoamInterface, 0);


// * * * * * * * * * * * * * * * Constructor * * * * * * * * * * * * * * * * //
mapFoamInterface::mapFoamInterface()
{
    _init_type = (MAP_InitInputType_t*)(uintptr_t)map_create_init_type(map_msg, &ierr); // @todo: check ierr==MAP_SAFE error for all
    _io_type = (MAP_InitOutputType_t*)(uintptr_t)map_create_initout_type(map_msg, &ierr);
    _u_type = (MAP_InputType_t*)(uintptr_t)map_create_input_type(map_msg, &ierr);
    _p_type = (MAP_ParameterType_t*)(uintptr_t)map_create_parameter_type(map_msg, &ierr);
    _z_type = (MAP_ConstraintStateType_t*)(uintptr_t)map_create_constraint_type(map_msg, &ierr);
    _x_type = (MAP_ContinuousStateType_t*)(uintptr_t)map_create_continuous_type(map_msg, &ierr);
    _y_type = (MAP_OutputType_t*)(uintptr_t)map_create_output_type(map_msg, &ierr);            
    _other_type = (MAP_OtherStateType_t*)(uintptr_t)map_create_other_type(map_msg, &ierr); 
    
    success = MAP_SAFE;  
    ierr = MAP_SAFE;
    
    map_msg[MAP_ERROR_STRING_LENGTH] = '\0';
}

// destructor
mapFoamInterface::~mapFoamInterface()
{
}

// * * * * * * * * * * * * * * * Member functions  * * * * * * * * * * * * * //

void mapFoamInterface::read_input_file(std::string inputFile)
{ 
    // split file into lines, then process line by line

    std::ifstream infile(inputFile.c_str());

    if (infile.is_open()) {
        std::string line;

        bool dictFlag, nodeFlag, propFlag, solvFlag;
        dictFlag = nodeFlag = propFlag = solvFlag = false;

        while (std::getline(infile, line))
        {
          
          // printf("%s\n", line.c_str());
              
          if (line.find("LINE DICTIONARY") != std::string::npos){
              //cout << "found LINE DICTIONARY!" << endl;
              
              dictFlag = true;
              nodeFlag = propFlag = solvFlag = false;
              
              // Process header
              std::getline(infile, line);
              std::getline(infile, line);
              continue;
          } else if (line.find("NODE PROPERTIES") != std::string::npos) {
              
              nodeFlag = true;
              dictFlag = propFlag = solvFlag = false;
              
              // Process header
              std::getline(infile, line);
              std::getline(infile, line);
              continue;
          } else if (line.find("LINE PROPERTIES") != std::string::npos) {
              propFlag = true;
              dictFlag = nodeFlag = solvFlag = false;
              
              // Process header
              std::getline(infile, line);
              std::getline(infile, line);
              continue;
          } else if (line.find("SOLVER OPTIONS") != std::string::npos) {  
              solvFlag = true;
              dictFlag = propFlag = nodeFlag = false;
              
              // Process header
              std::getline(infile, line);
              std::getline(infile, line);
              continue;
          }
          
          if (dictFlag)
          {    //_init_type.contents.libraryInputLine =  six.b(line+'\n\0');
              strcpy(_init_type->library_input_str, line.c_str());
              map_add_cable_library_input_text(_init_type);
          }
          if (nodeFlag)
          {
              strcpy(_init_type->node_input_str, line.c_str());
              map_add_node_input_text(_init_type);
          }
          if (propFlag)
          {
              strcpy(_init_type->line_input_str, line.c_str());
              map_add_line_input_text(_init_type);
          }
          if (solvFlag)
          {
              strcpy(_init_type->option_input_str, line.c_str());
              map_add_options_input_text(_init_type);
          }

        }
        infile.close();
        std::cout << "Finished reading MAP++ input file: " << inputFile << std::endl;
    }

}

void mapFoamInterface::initMAP(std::string inputFile, std::string summaryFile, double depth, double g, double rho)
{
    
    // cout << "--------- Start of INITIALIZATION ----------" << endl;
    
    map_initialize_msqs_base(_u_type, _p_type, _x_type, _z_type, _other_type, _y_type, _io_type);
    
    // SET ENVIRONMENT PARAMETERS
    // double depth = 100.0;
    // double g = 9.81;
    // double rho = 1025.0;
    
    map_set_sea_depth(_p_type, depth);
    map_set_gravity(_p_type, g);
    map_set_sea_density(_p_type, rho);
    
    // read MAP input file
    read_input_file(inputFile.c_str());
    
    // string summaryFile = "baseline.sum.map";
    
    strcpy(_init_type->summary_file_name, summaryFile.c_str()); 
    map_set_summary_file_name(_init_type, map_msg, &ierr);
    
    std::cout << "MAP init summary to be written to file: " << summaryFile << std::endl;
  
    map_init(_init_type, _u_type, _p_type, _x_type, NULL, _z_type, _other_type, _y_type, _io_type, &ierr, map_msg);
    if (ierr!=MAP_SAFE) {
        std::cout << map_msg << std::endl;
    };
    
    MAPFREE(_init_type); 
    MAPFREE(_io_type);
    
    // cout << "--------- MAP++ INITIALIZATION COMPLETE ----------" << endl;
}

/*  1) update the input states in u_type.x, u_type.y, u_type.z (fairlead displacements) 
 *     Alternatively, call py_offset_vessel to displace fairlead positions. 
 *  2) call map_update_states()       
 *  3) call map_calc_output() (optional is you don't want outlist to be updated)
 *  4) get outputs from y_type.fx, y_type.fy, y_type.fz (fairlead node sum-force). You have to calculate
 *     the moments manually, i.e., cross(r,f).
 
MAP_EXTERNCALL void map_offset_fairlead(MAP_InputType_t* u_type, const int fairlead_id, 
                                        const double x, const double y, const double z, 
                                        char* map_msg, MAP_ERROR_CODE* ierr)
   
MAP_EXTERNCALL void map_offset_vessel(MAP_OtherStateType_t* other_type, MAP_InputType_t* u_type, 
                                      double x, double y, double z, double phi, double the, double psi, 
                                      char* map_msg, MAP_ERROR_CODE* ierr)
*/

void mapFoamInterface::updateMAP(double time, double fairlead[], double force[])
{
    // Domain* data = (Domain*)_other_type->object;
    // Vessel* vessel = &data->vessel;

    int N = _u_type->x_Len;

    for (int i=0 ; i<N ; i++) {        
        _u_type->x[i] = fairlead[i*3];
        _u_type->y[i] = fairlead[i*3+1];
        _u_type->z[i] = fairlead[i*3+2];       
    };

    // map_update_states(time, its, u_type, p_type, x_type, NULL, z_type, other_type, &ierr, map_msg); 
    map_update_states(time, 0, _u_type, _p_type, _x_type, NULL, _z_type, _other_type, &ierr, map_msg);

    if (ierr!=MAP_SAFE) {
        std::cout << map_msg << std::endl;
    };

    // map_calc_output(time, _u_type, _p_type, _x_type, NULL, _z_type,
    //                 _other_type, _y_type, &ierr, map_msg);
    
    // Reverse fairlead force to get mooring force on floating body
    for (int i=0; i<N; i++) {        
        force[i*3] = -_y_type->Fx[i];
        force[i*3+1] = -_y_type->Fy[i];
        force[i*3+2] = -_y_type->Fz[i];       
    };

    // cout << "Time step: " << time << endl;
    // cout << "Line 0 fairlead force: " << _y_type->Fx[0] << " " << _y_type->Fy[0] << " " << _y_type->Fz[0] << endl; 

}

int mapFoamInterface::getNLines()
{
    return map_size_lines(_other_type, &ierr, map_msg);
}

// return node coordinates [x,y,z] for line i, of size nNodes x 3
void mapFoamInterface::getNodeCoordinates(int li, int nNodes, double coord[])
{
    double* arrx;
    double* arry;
    double* arrz;

    arrx = map_plot_x_array(_other_type, li, nNodes, map_msg, &ierr);
    arry = map_plot_y_array(_other_type, li, nNodes, map_msg, &ierr);
    arrz = map_plot_z_array(_other_type, li, nNodes, map_msg, &ierr);

    for(int i=0; i<nNodes; i++)
    {
        coord[i*3] = arrx[i];
        coord[i*3+1] = arry[i];
        coord[i*3+2] = arrz[i];
    }
}


void mapFoamInterface::closeMAP()
{
    // cout << "--------- Start of DEALLOCATION ----------" << endl;   

    // MAP_ERROR_CODE success = MAP_SAFE;
    map_end(_u_type, _p_type, _x_type, NULL, _z_type, _other_type, _y_type, &ierr, map_msg);

    success = map_free_types(_u_type, _p_type, _x_type, _z_type, _other_type, _y_type); 

    MAPFREE(_other_type); 
    MAPFREE(_y_type); 
    MAPFREE(_u_type);
    MAPFREE(_p_type);
    MAPFREE(_z_type);
    MAPFREE(_x_type);

    std::cout << "--------- MAP++ DEALLOCATION COMPLETE ----------" << std::endl;
}

//} // End namespace Foam
