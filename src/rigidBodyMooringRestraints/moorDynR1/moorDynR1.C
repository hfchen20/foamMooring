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

#include "moorDynR1.H"
#include "rigidBodyModel.H"
#include "addToRunTimeSelectionTable.H"

// include MoorDyn header
#include "MoorDynv1.h"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace RBD
{
namespace restraints
{
    defineTypeNameAndDebug(moorDynR1, 0);

    addToRunTimeSelectionTable
    (
        restraint,
        moorDynR1,
        dictionary
    );
}
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::RBD::restraints::moorDynR1::moorDynR1
(
    const word& name,
    const dictionary& dict,
    const rigidBodyModel& model
)
:
    restraint(name, dict, model)
{
    read(dict);
    
    initialized_ = false;

    Info<< "Create moorDynR1 using MoorDyn v1." << endl;

    // If different bodies are attached to moodyR
    if (coeffs_.found("bodies") )
    {
        // size of bodies_ = nAttachments_
        Info<< "Multiple bodies specified in moorDynR1 restraint: "
            << bodies_ << endl;
    }

}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::RBD::restraints::moorDynR1::~moorDynR1()
{
    if (initialized_)
    {
        // Close MoorDyn call
        LinesClose();
    }
}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

void Foam::RBD::restraints::moorDynR1::restrain
(
    scalarField& tau,
    Field<spatialVector>& fx,
    const rigidBodyModelState& state
) const
{
    scalar deltaT = state.deltaT();
    scalar tprev = state.t() - deltaT;
	
    const spatialTransform CofR(model_.X0(bodyID_));
    const spatialVector vCofR(model_.v(bodyID_, Zero));
	
    point CoR = CofR.r();
	
    vector rotationAngle
    (
       quaternion(CofR.E()).eulerAngles(quaternion::XYZ)
    );
	
    vector v = vCofR.l();
    vector omega = vCofR.w();

    double X[6], XD[6];
    for (int ii=0; ii<3; ii++)
    {
       X[ii] = CoR[ii];
       X[ii+3] = rotationAngle[ii];
       XD[ii] = v[ii];
       XD[ii+3] = omega[ii];
    }

    if (!initialized_)
    {
        // Initialize MoorDyn
        //LinesInit(&CoM[0], &v[0]);
        LinesInit(X, XD);
        Info<< "MoorDyn module initialized!" << endl;
        initialized_ = true;
    }
	
    scalar nFair = refAttachmentPt_.size();
    pointField fairPos = vectorField(nFair, vector::zero);
    vectorField fairVel = vectorField(nFair, vector::zero);
    vectorField fairForce = vectorField(nFair, vector::zero);
    
    for (int ii = 0; ii<nFair; ++ii)
    {
        // Transform the starting points to last motion state
        fairPos[ii] = bodyPoint(refAttachmentPt_[ii], bodyIDs_[ii]);

        // fairlead point velocity
        fairVel[ii] = bodyPointVelocity(refAttachmentPt_[ii], bodyIDs_[ii]).l();
    }

    Info<< "X[6]: " << vector(X[0], X[1], X[2]) << ", " << vector(X[3], X[4], X[5])
        << endl;

    // Call LinesCalc() to obtain forces and moments, Flines(1x6)
    // LinesCalc(double X[], double XD[], double Flines[], double* t_in, double* dt_in)
    //LinesCalc(X, XD, Flines, &tprev, &deltaT);

    //FairleadsCalc2(double rFairIn[], double rdFairIn[], double fFairIn[], double* t_in, double *dt_in);
    FairleadsCalc2(&fairPos[0][0], &fairVel[0][0], &fairForce[0][0], &tprev, &deltaT);

    //FairleadsCalc(double **rFairIn, double **rdFairIn, double ** fFairIn, double* t_in, double *dt_in)
    //FairleadsCalc(&fairPos[0], &fairVel[0], &fairForce[0], &tprev, &deltaT);

    // Sum forces and calculate moments
    for(int pt=0; pt<nFair; pt++)
    {
        vector force = fairForce[pt];

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


bool Foam::RBD::restraints::moorDynR1::read
(
    const dictionary& dict
)
{
    restraint::read(dict);
    
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


void Foam::RBD::restraints::moorDynR1::write
(
    Ostream& os
) const
{
    restraint::write(os);

    if (int(bodies_.size())>0)
        os.writeEntry("bodies",bodies_);
    os.writeEntry("refAttachmentPt", refAttachmentPt_);
}


// ************************************************************************* //
