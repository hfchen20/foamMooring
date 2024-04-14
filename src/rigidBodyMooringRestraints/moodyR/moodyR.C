/*---------------------------------------------------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | www.openfoam.com
     \\/     M anipulation  |
-------------------------------------------------------------------------------
    Copyright (C) 2016 OpenFOAM Foundation
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

References
    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with
    mooring dynamics: Coupling MoorDyn with OpenFOAM. Applied Ocean
    Research, 124, 103210. https://doi.org/10.1016/j.apor.2022.103210
    
    Chen, H., Medina, T. A., & Cercos-Pita, J. L. (2024). CFD simulation of multiple
    moored floating structures using OpenFOAM: An open-access mooring restraints 
    library. Ocean Engineering, 303, 117697.
    https://doi.org/10.1016/j.oceaneng.2024.117697
    
\*---------------------------------------------------------------------------*/

#include "moodyR.H"
#include "rigidBodyModel.H"
#include "addToRunTimeSelectionTable.H"

// include Moody header
#include "moodyWrapper.h"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace RBD
{
namespace restraints
{
    defineTypeNameAndDebug(moodyR, 0);

    addToRunTimeSelectionTable
    (
        restraint,
        moodyR,
        dictionary
    );
}
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::RBD::restraints::moodyR::moodyR
(
    const word& name,
    const dictionary& dict,
    const rigidBodyModel& model
)
:
    restraint(name, dict, model),
    bodyIDs_(),
    bodyIndices_()
{
    read(dict);
    
    initialized_ = false;

    Info<< "Create moodyR, a mooring dynamics restraint (Palm et al. 2017)" << endl;

    nCouplingDof_ = 3 * refAttachmentPt_.size();
    Info<< "\tSupport couplingMode 'externalPoint' only, nCouplingDof = " << nCouplingDof_ 
        << endl;
    
    // If different bodies are attached to moodyR
    if (coeffs_.found("bodies") )
    {
        // size of bodies_ = nAttachments_
        Info<< "Multiple bodies specified in moodyR restraint: " << bodies_ << endl;
    }
}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::RBD::restraints::moodyR::~moodyR()
{
    if (initialized_)
    {
        // Close Moody call
        moodyClose();
    }
}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

void Foam::RBD::restraints::moodyR::restrain
(
    scalarField& tau,
    Field<spatialVector>& fx,
    const rigidBodyModelState& state
) const
{
    scalar t = state.t();
    scalar tprev = t-state.deltaT();

    //double t = model_.time().value();
    //double tPrev = t - model_.time().deltaTValue();
    
    pointField fairPos = vectorField(int(nCouplingDof_/3), vector::zero);
    vectorField fairForce =vectorField(int(nCouplingDof_/3), vector::zero);

    for (int ii = 0; ii<refAttachmentPt_.size(); ++ii)
    {
        // Transform the starting points to last motion state
        fairPos[ii] = bodyPoint(refAttachmentPt_[ii], bodyIDs_[ii]);
    }
    
    if (!initialized_)
    {   
        // Initialize Moody
        // int moodyInit(const char* fName, int nVals, double initialValues[], double startTime );
        moodyInit(fname_.c_str(), nCouplingDof_,  &fairPos[0][0], tprev);

        Info<< "\tMoody module initialized!\n" << endl;
        initialized_ = true;
    }

    // void moodySolve(const double X[], double F[], double t1, double t2);
    moodySolve(&fairPos[0][0], &fairForce[0][0], tprev, t);

    // Sum forces and calculate moments
    for(int pt=0; pt<refAttachmentPt_.size(); pt++)
    {
        vector force = -fairForce[pt];

        vector moment =  fairPos[pt] ^ force;

        // Accumulate the force for the restrained body
        //fx[bodyIndex_] += spatialVector(moment, force);
        fx[bodyIndices_[pt]] += spatialVector(moment, force);

        Info<< " attachmentPt[" << pt << "] " << fairPos[pt]
            << " force " << force
            << " moment " << moment
            << endl;
    
    }

}


bool Foam::RBD::restraints::moodyR::read
(
    const dictionary& dict
)
{
    restraint::read(dict);

    coeffs_.readEntry("inputFile", fname_);
    coeffs_.readEntry("refAttachmentPt", refAttachmentPt_);

    scalar nAttachments = refAttachmentPt_.size();

    // Initialise the sizes of bodyIDs, and bodyIndices
    bodyIDs_ = List<label>(nAttachments, bodyID_);
    bodyIndices_ = List<label>(nAttachments, bodyIndex_);
    
    // If different bodies are attached to moody moorings:
    if (coeffs_.found("bodies") )
    {
        coeffs_.lookup("bodies") >> bodies_;
        // size of bodies_ = nAttachments_
    
        for(int ii=0; ii<nAttachments; ii++) 			
        {
            bodyIDs_[ii] = model_.bodyID(bodies_[ii]);
            bodyIndices_[ii] = model_.master(bodyIDs_[ii]);
        }
    }

    return true;
}


void Foam::RBD::restraints::moodyR::write
(
    Ostream& os
) const
{
    restraint::write(os);

    os.writeEntry("inputFile", fname_);
    if (int(bodies_.size())>0)
	   	os.writeEntry("bodies",bodies_);
    os.writeEntry("refAttachmentPt", refAttachmentPt_);
    
}


// ************************************************************************* //
