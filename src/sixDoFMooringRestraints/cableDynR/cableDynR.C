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

References
    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with
    mooring dynamics: Coupling MoorDyn with OpenFOAM. Applied Ocean
    Research, 124, 103210. https://doi.org/10.1016/j.apor.2022.103210
    
    Chen, H., Medina, T. A., & Cercos-Pita, J. L. (2024). CFD simulation of multiple
    moored floating structures using OpenFOAM: An open-access mooring restraints 
    library. Ocean Engineering, 303, 117697.
    https://doi.org/10.1016/j.oceaneng.2024.117697

\*---------------------------------------------------------------------------*/

#include "cableDynR.H"
#include "addToRunTimeSelectionTable.H"
#include "sixDoFRigidBodyMotion.H"
#include "fvMesh.H"
#include "OFstream.H"
#include "quaternion.H"

// include CableDyn header
#include "CableDyn_CAPI.h"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace sixDoFRigidBodyMotionRestraints
{
    defineTypeNameAndDebug(cableDynR, 0);

    addToRunTimeSelectionTable
    (
        sixDoFRigidBodyMotionRestraint,
        cableDynR,
        dictionary
    );
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::cableDynR::cableDynR
(
    const word& name,
    const dictionary& sDoFRBMRDict
)
:
    sixDoFRigidBodyMotionRestraint(name, sDoFRBMRDict)
{
    read(sDoFRBMRDict);
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

    CableDyn_Create(&handle_, &err);
}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::cableDynR::~cableDynR()
{
    if (handle_ && initialized_)
    {
        int err = CD_C_OK;
        CableDyn_Close(&handle_, &err);
    }
}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //


void Foam::sixDoFRigidBodyMotionRestraints::cableDynR::restrain
(
    const sixDoFRigidBodyMotion& motion,
    vector& restraintPosition,
    vector& restraintForce,
    vector& restraintMoment
) const
{
    scalar deltaT = motion.time().deltaTValue();
    scalar t = motion.time().value();
//     scalar tprev = t-deltaT;

    point CoR = motion.centreOfRotation();
    
//     point CoM = motion.centreOfMass();
//     vector rotationAngle
//     (
//        quaternion(motion.orientation()).eulerAngles(quaternion::XYZ)
//     );

//     vector v = motion.v();
//     vector omega = motion.omega();

//     double X[6], XD[6];
//     for (int ii=0; ii<3; ii++)
//     {
//        X[ii] = CoM[ii];
//        X[ii+3] = rotationAngle[ii];
//        XD[ii] = v[ii];
//        XD[ii+3] = omega[ii];
//     }

    int err = CD_C_OK; // status = 1
    char msg[1024];

    bool converged, stalled;
    int n_iter;

    if (!initialized_)
    {
        // Initialize CableDyn
        const char* deck = inputFile_.c_str();
        CableDyn_InitDeck(handle_, deck, static_cast<int>(strlen(deck)), &err); //  (int)strlen(deck)
        /* parse + static solve */
        
        if (err != CD_C_OK)
        {
            CableDyn_GetLastError(handle_, msg, static_cast<int>(sizeof msg)); // (int)sizeof msg
            
            FatalErrorInFunction
                << "CableDyn error " << err
                << " " << string(msg)
                << exit(FatalError);
        }
        
        const int ndof = CableDyn_NCoupledDOF(handle_, &err);
        
        Info<< "CableDyn initialized! " 
            << CableDyn_NLines(handle_, &err) << " lines "
            << CableDyn_NPoints(handle_, &err) << " points "
            << endl;
        
        Info<< "CableDyn_NCoupledDOF = " << ndof
            << " while we have " << refAttachmentPt_.size() << " attchment pts"
            << ", cplPtStartIndex = " << cplPtStartIndex_
            << endl;

        initialized_ = true;
        
        nCpldPts_ = int(ndof/3);

        cpldPos_.resize(nCpldPts_, Zero);
        cpldVel_.resize(nCpldPts_, Zero);
        cpldAcc_.resize(nCpldPts_, Zero);

        // populate coupled vectors, could fetch here fairlead coordinates
        // if not specified in restraint
        // fetch anchor positions as well, required by cabledyn
        CableDyn_GetCoupledMotion
        (
            handle_, 
            &cpldPos_[0][0], &cpldVel_[0][0], &cpldAcc_[0][0],
            ndof, &err
        );
    }

    const scalar nfair = refAttachmentPt_.size(); 
    const int ndof = nCpldPts_*3;
    
    // update coupled dofs at fairleads: pos, vel, accel
    // skip anchor points
    for (int ii = 0; ii<nfair; ++ii)
    {
        point p = refAttachmentPt_[ii];
        
        // cpldlead point index in cabledyn
        label fpIndex = ii + cplPtStartIndex_;
        
        // current position transformed from original global coordinate
        cpldPos_[fpIndex] = motion.transform(p);
        
        // velocity
        cpldVel_[fpIndex] = motion.velocity(cpldPos_[fpIndex]);
        
        // acceleration
        // todo: either direct calculation or differencing from velocity
        //cpldAcc_[fpIndex] = motion.acceleration(p);
        
        Info<< "Pt[" << fpIndex << "] " //<< cpldPos[fpIndex]
            << " vel " << cpldVel_[fpIndex] << " acc " << cpldAcc_[fpIndex]
            << endl;
    }
    
    CableDyn_Step
    (
        handle_, deltaT, 
        &cpldPos_[0][0], &cpldVel_[0][0], &cpldAcc_[0][0],
        ndof, &converged, &stalled, &n_iter, &err
    );

    // mooring forces on body at coupled points
    vectorField force(nCpldPts_, Zero);
    CableDyn_CalcOutput(handle_, &force[0][0], ndof, &err);
    
    for(int pt=cplPtStartIndex_; pt<nCpldPts_; pt++)
    {
        restraintForce += force[pt];

        restraintMoment += (cpldPos_[pt] - CoR) ^ force[pt];

        Info<< "Pt[" << pt << "] " << cpldPos_[pt]
            << " force " << force[pt]
            << endl;
    }

    // Since moment is already calculated, set to
    // centreOfRotation to be sure of no spurious moment
    restraintPosition = motion.centreOfRotation();

    if (motion.report())
    {
        Info<< t << ": force " << restraintForce
            << ", moment " << restraintMoment
            << endl;
    }
}


bool Foam::sixDoFRigidBodyMotionRestraints::cableDynR::read
(
    const dictionary& sDoFRBMRDict
)
{
    sixDoFRigidBodyMotionRestraint::read(sDoFRBMRDict);

    sDoFRBMRCoeffs_.readEntry("inputFile", inputFile_);
    sDoFRBMRCoeffs_.readEntry("cplPtStartIndex", cplPtStartIndex_);
    sDoFRBMRCoeffs_.readEntry("refAttachmentPt", refAttachmentPt_);
    
    return true;
}


void Foam::sixDoFRigidBodyMotionRestraints::cableDynR::write
(
    Ostream& os
) const
{
    os.writeEntry("inputFile", inputFile_);
    os.writeEntry("cplPtStartIndex", cplPtStartIndex_);
    os.writeEntry("refAttachmentPt", refAttachmentPt_);
}

// ************************************************************************* //
