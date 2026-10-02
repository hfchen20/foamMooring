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

\*---------------------------------------------------------------------------*/

#include "cableDynR.H"
#include "rigidBodyModel.H"
#include "addToRunTimeSelectionTable.H"

// include CableDyn header
#include "CableDyn_CAPI.h"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace RBD
{
namespace restraints
{
    defineTypeNameAndDebug(cableDynR, 0);

    addToRunTimeSelectionTable
    (
        restraint,
        cableDynR,
        dictionary
    );
}
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::RBD::restraints::cableDynR::cableDynR
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

    Info<< "Create mooring restraint using CableDyn.";

    int major, minor, patch, abi, err = CD_C_OK;
    CableDyn_GetVersion(&major, &minor, &patch, &abi);

    if (abi != CABLEDYN_CAPI_ABI_VERSION)
    {
        FatalErrorInFunction
            << " library speaks C ABI " << abi 
            << ", header expects " << CABLEDYN_CAPI_ABI_VERSION
            << exit(FatalError);
    }
    else
    {
        Info<< " Version: major " << major << ", minor " << minor
            << endl;
    }
    //CableDyn_GetVersionString(char *version, int version_len);
    // char version[50];
    // int version_len;
    // CableDyn_GetVersionString(version, version_len);
    // Info<< "  version: " << string(version) << endl;

    CableDyn_Create(&handle_, &err);

    // CableDyn_InitDeck(h, deck, (int)strlen(deck), &err); /* parse + static solve */

    // could get attachment pts from this (static solution)
    // CableDyn_GetCoupledMotion(h, q, v, a, n, &err);

    // const int n = CableDyn_NCoupledDOF(h, &err);

    // If different bodies are attached
    // if (coeffs_.found("bodies") )
    // {
    //     // size of bodies_ = nAttachments_
    //     Info<< "Multiple bodies specified in cableDynR restraint: "
    //         << bodies_ << endl;
    // }

}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::RBD::restraints::cableDynR::~cableDynR()
{
    if (handle_ && initialized_)
    {
        int err = CD_C_OK;
        CableDyn_Close(&handle_, &err);
    }
}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

void Foam::RBD::restraints::cableDynR::restrain
(
    scalarField& tau,
    Field<spatialVector>& fx,
    const rigidBodyModelState& state
) const
{
    scalar deltaT = state.deltaT();
    // scalar tprev = state.t() - deltaT;
    
    static bool first = true;
    
    int err = CD_C_OK; // status = 1
    char msg[1024];

    bool converged, stalled;
    int n_iter;

    if (!initialized_)
    {
        // Initialize CableDyn
        const char* deck = inputFile_.c_str();
        CableDyn_InitDeck(handle_, deck, static_cast<int>(strlen(deck)), &err); // (int)strlen(deck)
        /* parse + static solve */
        
        if (err != CD_C_OK)
        {
            CableDyn_GetLastError(handle_, msg, static_cast<int>(sizeof msg)); // (int)sizeof msg
            
            FatalErrorInFunction
                << " CableDyn error " << err
                << " " << string(msg)
                << exit(FatalError);
        }
        
        const int n = CableDyn_NCoupledDOF(handle_, &err);
        
        Info<< "CableDyn initialized! " 
            << CableDyn_NLines(handle_, &err) << " lines "
            << CableDyn_NPoints(handle_, &err) << " points "
            //<< n << " coupled DOFs "
            << endl;
        
        Info<< "CableDyn_NCoupledDOF = " << n
            << " while we have " << refAttachmentPt_.size() << " attchment pts"
            << ", cplPtStartIndex = " << cplPtStartIndex_
            << endl;
        
        // if (n != 3 * refAttachmentPt_.size())
        // {
        //     FatalErrorInFunction
        //         << " refAttachmentPt size: " << refAttachmentPt_.size() 
        //         << " not equal to # coupled points " << int(n/3)
        //         << " defined in CableDyn input!"
        //         << exit(FatalError);
        // }

        initialized_ = true;
    }

    const scalar nFair = refAttachmentPt_.size();
    //const int ndof = 3*nFair;
    
    const int ndof = CableDyn_NCoupledDOF(handle_, &err);
    const label nPts = int(ndof/3);

    // could get attachment pts from this (after static init)
    // CableDyn_GetCoupledMotion(h, q, v, a, n, &err);

    pointField fairPos = vectorField(nPts, vector::zero);
    vectorField fairVel = vectorField(nPts, vector::zero);
    vectorField fairAcc = vectorField(nPts, vector::zero);
    vectorField fairForce = vectorField(nPts, vector::zero);
    
    // populate coupled dofs
    // could fetch fairlead coordinate if not specified in restraint
    // fetch anchor positions as cabledyn api requires these in coupled dofs
    // todo: fetch once and save for later use
    CableDyn_GetCoupledMotion
    (
        handle_, 
        &fairPos[0][0], &fairVel[0][0], &fairAcc[0][0],
        ndof, &err
    );
    
    // Info<< "CableDyn_GetCoupledMotion" << nl
    //     << "  pos " << fairPos << nl
    //     << "  vel " << fairVel << nl
    //     << "  acc " << fairAcc << nl
    //     << endl;
    
    // update coupled dofs at fairleads: pos, vel, accel
    // should skip anchor points
    for (int ii = 0; ii<nFair; ++ii)
    {
        point p = refAttachmentPt_[ii];
        
        // fairlead point index in cabledyn
        label fpIndex = ii + cplPtStartIndex_;

        // Transform local points to last motion state
        fairPos[fpIndex] = bodyPoint(bodyIDs_[ii], p);

        // fairlead point velocity
        fairVel[fpIndex] = model_.v(bodyIDs_[ii], p).l();

        // fairlead point acceleration
        fairAcc[fpIndex] = pointAcceleration(bodyIDs_[ii], p);
        
        if (first)
        {
            fairAcc[fpIndex] -= model_.g();
        }
        
        // Info<< "Pt[" << fpIndex << "]: pos " << fairPos[fpIndex]
        //     << " vel " << fairVel[fpIndex] << " acc " << fairAcc[fpIndex]
        //     << endl;
    }
    if (first) first = false;
    
    CableDyn_Step
    (
        handle_, deltaT, 
        &fairPos[0][0], &fairVel[0][0], &fairAcc[0][0],
        ndof, &converged, &stalled, &n_iter, &err
    );

    CableDyn_CalcOutput(handle_, &fairForce[0][0], ndof, &err);

    // Sum forces and calculate moments
    for(int pt=cplPtStartIndex_; pt<nPts; pt++)
    {
        vector force = fairForce[pt];
        
        vector moment = fairPos[pt] ^ force;

        // Accumulate the force for the restrained body
        // fx[bodyIndex_] += spatialVector(moment, force);

        fx[bodyIndices_[pt-cplPtStartIndex_]] += spatialVector(moment, force);

        Info<< "Pt[" << pt << "] " << fairPos[pt]
            << " vel " << fairVel[pt] << " acc " << fairAcc[pt]
            << " force " << force
            //<< " moment " << moment
            << endl;
    }
}


bool Foam::RBD::restraints::cableDynR::read
(
    const dictionary& dict
)
{
    restraint::read(dict);
    
    coeffs_.readEntry("inputFile", inputFile_);
    coeffs_.readEntry("cplPtStartIndex", cplPtStartIndex_);
    coeffs_.readEntry("refAttachmentPt", refAttachmentPt_);
    
    scalar nAttachments = refAttachmentPt_.size();

    // Initialise the sizes of bodyIDs, and bodyIndices
    bodyIDs_ = List<label>(nAttachments, bodyID_);
    bodyIndices_ = List<label>(nAttachments, bodyIndex_);
    
    // If different bodies are attached to moorings:
    // if (coeffs_.found("bodies") )
    // {
    //     coeffs_.lookup("bodies") >> bodies_;
    //     // size of bodies_ = nAttachments_
    
    //     for(int ii=0; ii<nAttachments; ii++)
    //     {
    //         bodyIDs_[ii] = model_.bodyID(bodies_[ii]);
    //         bodyIndices_[ii] = model_.master(bodyIDs_[ii]);
    //     }
    // }

    return true;
}


void Foam::RBD::restraints::cableDynR::write
(
    Ostream& os
) const
{
    restraint::write(os);

    os.writeEntry("inputFile", inputFile_);

    // if (int(bodies_.size())>0)
    //     os.writeEntry("bodies",bodies_);
    os.writeEntry("refAttachmentPt", refAttachmentPt_);
    os.writeEntry("cplPtStartIndex", cplPtStartIndex_);
}


// ************************************************************************* //
