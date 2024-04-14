/*---------------------------------------------------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | www.openfoam.com
     \\/     M anipulation  |
-------------------------------------------------------------------------------
    Copyright (C) 2019-2020 OpenCFD Ltd.
-------------------------------------------------------------------------------
License
    This file is part of OpenFOAM.

    OpenFOAM is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenFOAM is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
    for more details.

    You should have received a copy of the GNU General Public License
    along with OpenFOAM.  If not, see <http://www.gnu.org/licenses/>.

\*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*\
References

    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with
    mooring dynamics: Coupling MoorDyn with OpenFOAM. Applied Ocean
    Research, 124, 103210. https://doi.org/10.1016/j.apor.2022.103210
    
    Chen, H., Medina, T. A., & Cercos-Pita, J. L. (2024). CFD simulation of multiple
    moored floating structures using OpenFOAM: An open-access mooring restraints 
    library. Ocean Engineering, 303, 117697.
    https://doi.org/10.1016/j.oceaneng.2024.117697

\*---------------------------------------------------------------------------*/
#include "map3R.H"
#include "addToRunTimeSelectionTable.H"
#include "sixDoFRigidBodyMotion.H"
#include "Time.H"
#include "fvMesh.H"
#include "OFstream.H"
#include "uniformDimensionedFields.H"
#include "foamVtkSeriesWriter.H"

#include "mapFoamInterface.H"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace sixDoFRigidBodyMotionRestraints
{
    defineTypeNameAndDebug(map3R, 0);

    addToRunTimeSelectionTable
    (
        sixDoFRigidBodyMotionRestraint,
        map3R,
        dictionary
    );
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::map3R::map3R
(
    const word& name,
    const dictionary& sDoFRBMRDict
)
:
    sixDoFRigidBodyMotionRestraint(name, sDoFRBMRDict),
    map_()
{
    read(sDoFRBMRDict);
    
    initialized_ = false;
    nLines_ = 0;
    nFairleads_ = 0;

    vtkCounter_ = 0;
    curTime_ = -1;
    iteration_ = 0;

    Info<< "Create map3R (quasi-static mooring code MAP++) ..." << endl;

    if (writeVTK_)
    {
        Info<< "To write mooring VTK starting from time " << vtkStartTime_
            << ", for the last outerCorrector only " <<  outerCorrector_
            << endl;
    }

}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::map3R::~map3R()
{
    if (initialized_)
    {
        // Close MAP++
        map_.closeMAP();
        
        vtk::seriesWriter writer;
        writer.scan("Mooring/VTK/map3.vtk");
        
        Info<< "Writing mooring vtk series file" << nl;

        fileName vtkSeries("Mooring/VTK/map3.vtk");
        writer.write(vtkSeries);
    }
}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

void Foam::sixDoFRigidBodyMotionRestraints::map3R::initializeMAP(const Time& time) const
{
    // Look up for environmental parameters: g, water density rho, 
    // water depth: read in from Restraints dict

    const fvMesh& mesh = time.lookupObject<fvMesh>("region0");

    // Since ~v1812 gravity changed to register with time only,
    // https://develop.openfoam.com/Development/openfoam/-/commit/1a4fceec20e75391bd5aa3b90aba042ef25080c0
    // #include "gravityMeshObject.H"
    // const uniformDimensionedVectorField& g = meshObjects::gravity::New(mesh().time());

    //const uniformDimensionedVectorField& g = mesh.lookupObject<uniformDimensionedVectorField>("g");
    const uniformDimensionedVectorField& g = time.lookupObject<uniformDimensionedVectorField>("g");

    const dictionary& transportProperties = mesh.lookupObject<IOdictionary>("transportProperties");
    // For newer OF versions, the fluid phase has a name "water", which has a "rho" entry
    //const scalar rho = readScalar(transportProperties.subDict("water").lookup("rho"));
    const scalar rho = dimensionedScalar(transportProperties.subDict("water").lookup("rho")).value();

    map_.initMAP(inputFile_, summaryFile_, depth_, mag(g.value()), rho);

    nLines_ = map_.getNLines();
    nFairleads_ = map_.getNFairleads();

    Info<< "MAP++ initialized! # of lines: " << nLines_ 
        << ", # nFairleads " << nFairleads_ 
        << endl << endl;

    if (nLines_ != refAttachmentPt_.size())
    {
        Info<< "Warning: Number of refAttachmentPt unequal to # of lines defined in MAP++! refAttachmentPt size: "
            << refAttachmentPt_.size() << ", MAP.size_lines() = " << nLines_ << endl;
        
        // If there are connecting nodes, nLines_ != refAttachmentPt_.size()
    }
    if (nFairleads_ != refAttachmentPt_.size())
    {
        // FatalIOErrorInFunction(*this)
        FatalErrorInFunction
            << "Number of refAttachmentPt unequal to # of fairleads defined in MAP++! refAttachmentPt size: "
            << refAttachmentPt_.size() << ", # nFairleads " << nFairleads_
            << exit(FatalError);
    }

    if (writeForces_)
    {
        mps_.reset( new OFstream(outputFile_) );
        // Writing header
        mps_() << "Time history of fairlead tension (compts) from MAP++. Total # fairleads: " << nFairleads_ << endl;
    }

    if (writeVTK_)
    {
        mkDir("Mooring/VTK");
        if (!sDoFRBMRCoeffs_.found("nodesPerLine"))
        {
            nodesPerLine_ = List<label>(nLines_, nNodes_);
        }
        else if (nodesPerLine_.size() != nLines_)
        {
            FatalErrorInFunction
                << "Entries of nodesPerLine unequal to # of lines defined in MAP++! nodesPerLine size: "
                << nodesPerLine_.size() << ", # nLines " << nLines_
                << exit(FatalError);
        }
    }
}

void Foam::sixDoFRigidBodyMotionRestraints::map3R::restrain
(
    const sixDoFRigidBodyMotion& motion,
    vector& restraintPosition,
    vector& restraintForce,
    vector& restraintMoment
) const
{
    const Time& time = motion.time();

    scalar t = time.value();

    point CoR = motion.centreOfRotation();
    
    if (t == curTime_)
    {
        iteration_++;
    }
    else if (t > curTime_)
    {
        iteration_ = 1;
        curTime_ = t;
    }

    if (!initialized_)
    {   
        curTime_ = t;
        iteration_ = 1;

        initializeMAP(time);
        
        initialized_ = true;
    }
    
    vectorField fairForce = vectorField(nLines_, vector::zero);
    
    tmp<pointField> tpoints = motion.transform(refAttachmentPt_);
    pointField fairPos = tpoints();
    
    // Call updateMAP to calculate new mooring configuration
    map_.updateMAP(t, &fairPos[0][0], &fairForce[0][0]);
    
    // Sum forces and calculate moments manualy since MAP does not return the moments
    for(int pt=0; pt<nLines_; pt++)
    {
        restraintForce += fairForce[pt];
        
        restraintMoment += (fairPos[pt] - CoR) ^ fairForce[pt];
    }
    
    // Since moment is already calculated as above, set to
    // centreOfRotation to be sure of no spurious moment
    restraintPosition = motion.centreOfRotation();

    if (motion.report())
    {
        Info<< t << ": force " << restraintForce
            << ", moment " << restraintMoment
            << endl;
    }
    
    if (iteration_ == outerCorrector_)
    {
        if (writeForces_)
        {
            // Write forces to file
            mps_() << t;
            for(int pt=0; pt<nLines_; pt++)
            {
                vector fi = fairForce[pt];
                mps_() << "\t" << fi[0] << "\t" << fi[1] << "\t" << fi[2];
            }       
            mps_() << endl;
        }
        
        if (writeVTK_)
        {
            if (time.outputTime() && t >= vtkStartTime_)
            {
                Info<< "Write mooring VTK ..." << endl;
                writeVTK(time);
            }
        }
    }
    
}


bool Foam::sixDoFRigidBodyMotionRestraints::map3R::read
(
    const dictionary& sDoFRBMRDict
)
{
    sixDoFRigidBodyMotionRestraint::read(sDoFRBMRDict);
    sDoFRBMRCoeffs_.readEntry("inputFile", inputFile_);
    sDoFRBMRCoeffs_.readEntry("summaryFile", summaryFile_);
    sDoFRBMRCoeffs_.readEntry("outputFile", outputFile_);
    sDoFRBMRCoeffs_.readEntry("waterDepth", depth_);
    sDoFRBMRCoeffs_.readEntry("refAttachmentPt", refAttachmentPt_);
    
    writeForces_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("writeMooringForces", true);
    writeVTK_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("writeMooringVTK", true);
    
    if (writeVTK_)
    {
        vtkStartTime_ = sDoFRBMRCoeffs_.getOrDefault<scalar>("vtkStartTime", 0);
        nNodes_ = sDoFRBMRCoeffs_.getOrDefault<label>("nNodes", 10);

        if (sDoFRBMRCoeffs_.found("nodesPerLine"))
        {
            sDoFRBMRCoeffs_.readEntry("nodesPerLine", nodesPerLine_);
        }

        outerCorrector_ = sDoFRBMRCoeffs_.getOrDefault<scalar>("outerCorrector", 3);
    }

    return true;
}


void Foam::sixDoFRigidBodyMotionRestraints::map3R::write
(
    Ostream& os
) const
{
    os.writeEntry("inputFile", inputFile_);
    os.writeEntry("summaryFile", summaryFile_);
    os.writeEntry("outputFile", outputFile_);
    os.writeEntry("waterDepth", depth_);
    os.writeEntry("refAttachmentPt", refAttachmentPt_);
    os.writeEntry("writeMooringForces", writeForces_);
    os.writeEntry("writeMooringVTK", writeVTK_);
    
    if (writeVTK_)
    {
        os.writeEntry("vtkStartTime", vtkStartTime_);
        if (sDoFRBMRCoeffs_.found("nNodes"))
        {
            os.writeEntry("nNodes", nNodes_);
        }
        if (sDoFRBMRCoeffs_.found("nodesPerLine"))
        {
            os.writeEntry("nodesPerLine", nodesPerLine_);
        }
    }
}


void Foam::sixDoFRigidBodyMotionRestraints::map3R::writeVTK(const Time& time) const
{
    double coord[max(nodesPerLine_)][3];
    
    fileName name("Mooring/VTK/map3_");
    // OFstream mps(name + time.timeName() + "_.vtk");
    OFstream mps(name + Foam::name(++vtkCounter_) + ".vtk");
    mps.precision(4);

    // Writing header
    mps << "# vtk DataFile Version 3.0" << nl << "MAP++ vtk output time=" << time.timeName() 
        << nl << "ASCII" << nl << "DATASET POLYDATA" << endl;
 
    // Writing points
    mps << "\nPOINTS " << sum(nodesPerLine_) << " float" << endl;

    for(int i=0; i<nLines_; i++)
    {   
        map_.getNodeCoordinates(i, nodesPerLine_[i], &coord[0][0]);

        for(int p=0; p<nodesPerLine_[i]; p++)
        {
            mps << coord[p][0] << " " << coord[p][1] << " " << coord[p][2] << endl;
        }
    }
    
    // Writing lines
    //mps << "\nLINES " << nLines_ << " " << nLines_*(nodesPerLine_+1) << endl;
    mps << "\nLINES " << nLines_ << " " << sum(nodesPerLine_+1) << endl;

    label start_node(0);
    for(int i=0; i<nLines_; i++)
    {       
        mps << nodesPerLine_[i];
        
        for(int j=0; j<nodesPerLine_[i]; j++)
        {
            mps << " " << start_node+j;
        }
        mps << endl;
        
        start_node += nodesPerLine_[i];
    }

}


// ************************************************************************* //
