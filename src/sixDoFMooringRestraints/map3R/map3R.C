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

    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with mooring dynamics: Coupling 
        MoorDyn with OpenFOAM. Applied Ocean Research, 124, 103210.
        https://doi.org/10.1016/j.apor.2022.103210

\*---------------------------------------------------------------------------*/
#include "map3R.H"
#include "addToRunTimeSelectionTable.H"
#include "sixDoFRigidBodyMotion.H"
#include "Time.H"
#include "fvMesh.H"
#include "OFstream.H"
#include "uniformDimensionedFields.H"
 
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
    
    // map_ = 
    
    initialized_ = false;

    Info << "Create map3R (quasi-static mooring code MAP++) ..." << endl;

}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::map3R::~map3R()
{
    if (initialized_)
    {
        // Close MAP++
        map_.closeMAP();
    }
}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //


void Foam::sixDoFRigidBodyMotionRestraints::map3R::restrain
(
    const sixDoFRigidBodyMotion& motion,
    vector& restraintPosition,
    vector& restraintForce,
    vector& restraintMoment
) const
{
    const Time& time = motion.time();

    // scalar deltaT = time.deltaTValue();
    scalar t = time.value();
    // scalar tprev = t - deltaT;

    point CoR = motion.centreOfRotation();
    
    if (!initialized_)
    {
        // Initialize MAP++
        
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
        const scalar rho = readScalar(transportProperties.subDict("water").lookup("rho"));
            
        map_.initMAP(inputFile_, summaryFile_, depth_, mag(g.value()), rho);
        
        nLines_ = map_.getNLines();
        
        Info<< "MAP++ initialized! # of lines: " << nLines_ << endl << endl;
        initialized_ = true;
        
        if (nLines_ != refAttachmentPt_.size())
        {
            Info<< "Warning: Number of refAttachmentPt unequal to lines # in MAP input! refAttachmentPt size: "
                << refAttachmentPt_.size() << ", MAP.size_lines() = " << nLines_ << endl;
            
            // If there are connecting lines, the following is NOT true.
            
            // FatalIOErrorInFunction(*this)
            // FatalErrorInFunction
            //    << "Number of refAttachmentPt unequal to # of lines in MAP input!  refAttachmentPt size: "
            //    << refAttachmentPt_.size() << ", MAP.size_lines() = " << nLines_
            //    << exit(FatalError);
        }
    }
    
    pointField fairPos1 = vectorField(nLines_, vector::zero);
    vectorField fairForce = vectorField(nLines_, vector::zero);
    
    // Calculate fairlead position
    for(int pt=0; pt<nLines_; pt++)
    {
        fairPos1[pt] = motion.transform(refAttachmentPt_[pt]);
    }
    
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
}


bool Foam::sixDoFRigidBodyMotionRestraints::map3R::read
(
    const dictionary& sDoFRBMRDict
)
{
    sixDoFRigidBodyMotionRestraint::read(sDoFRBMRDict);
    sDoFRBMRCoeffs_.readEntry("inputFile", inputFile_);
    sDoFRBMRCoeffs_.readEntry("summaryFile", summaryFile_);
    sDoFRBMRCoeffs_.readEntry("waterDepth", depth_);
    sDoFRBMRCoeffs_.readEntry("numberOfSegments", nSegments_);
    
    sDoFRBMRCoeffs_.readEntry("refAttachmentPt", refAttachmentPt_);
    
    writeVTK_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("writeMooringVTK", true);

    return true;
}


void Foam::sixDoFRigidBodyMotionRestraints::map3R::write
(
    Ostream& os
) const
{
    os.writeEntry("inputFile", inputFile_);
    os.writeEntry("summaryFile", summaryFile_);
    os.writeEntry("waterDepth", depth_);
    os.writeEntry("numberOfLines", nLines_);
    os.writeEntry("numberOfSegments", nSegments_);
    os.writeEntry("refAttachmentPt", refAttachmentPt_);
    os.writeEntry("writeMooringVTK", writeVTK_);
}

/*
void Foam::sixDoFRigidBodyMotionRestraints::map3R::writeVTK(fvMesh& mesh) const
{
    if (!writeVTK_)
    {
        return;
    }

    fileName name(mesh.time().timePath()/"mooringPoints");
    OFstream mps(name + ".vtk");

    // Writing header
    mps << "# vtk DataFile Version 3.0" << nl << "vtk output" << nl
        << "ASCII" << nl << "DATASET POLYDATA" << endl;

    // Writing points
    mps << "POINTS " << nLines_*(nSegments_+1) << " float" << endl;

    // Loop through nLines and nNodes for each line
    // GetNodePos(int LineNum, int NodeNum, double pos[3]);
    for(int line=0;line<nLines_;line++)
    {
        point p = Zero;

        for(int node=0;node<nSegments_+1;node++)
        {
            GetNodePos(line, node, &p[0]);
            mps << p.x() << " " << p.y() << " " << p.z() << endl;
        }
    }

    // Writing lines
    mps << "LINES " << nLines_ << " " << nLines_*(nSegments_+2) << endl;

    label start_node(0);
    for(int line=0;line<nLines_;line++)
    {
        mps << nSegments_+1;

        for(int j=0;j<nSegments_+1;j++)
        {
            mps << " " << start_node+j;
        }
        mps << endl;

        start_node += nSegments_+1;
    }

}
*/

// ************************************************************************* //
