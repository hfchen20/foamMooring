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

    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with mooring dynamics: 
        Coupling MoorDyn with OpenFOAM. Applied Ocean Research, 124, 103210.
        https://doi.org/10.1016/j.apor.2022.103210

\*---------------------------------------------------------------------------*/

#include "moodyR.H"
#include "addToRunTimeSelectionTable.H"
#include "sixDoFRigidBodyMotion.H"
#include "Time.H"
#include "fvMesh.H"
#include "OFstream.H"
#include "quaternion.H"

// include Moody header
#include "moodyWrapper.h"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace sixDoFRigidBodyMotionRestraints
{
    defineTypeNameAndDebug(moodyR, 0);

    addToRunTimeSelectionTable
    (
        sixDoFRigidBodyMotionRestraint,
        moodyR,
        dictionary
    );
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::moodyR::moodyR
(
    const word& name,
    const dictionary& sDoFRBMRDict
)
:
    sixDoFRigidBodyMotionRestraint(name, sDoFRBMRDict)
{
    read(sDoFRBMRDict);
    
    initialized_ = false;

    Info << "Create moodyR, a mooring dynamics restraint (Palm et al. 2017)" << endl;
    
    if (mode_ == word("externalPoint"))
    {
        nCouplingDof_ = 3 * refAttachmentPt_.size();
        Info << "\tCoupling mode: " << mode_ << ", nCouplingDof = " << nCouplingDof_ 
             << endl;
    }
    else
    {
        Info << "Coupling mode (default) externalRigidBody. nCouplingDof = 6 (Mooring system initialization error lingering)"
             << endl;
    }

}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::moodyR::~moodyR()
{
    if (initialized_)
    {
        // Close Moody call
        moodyClose();
    }
}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //


void Foam::sixDoFRigidBodyMotionRestraints::moodyR::restrain
(
    const sixDoFRigidBodyMotion& motion,
    vector& restraintPosition,
    vector& restraintForce,
    vector& restraintMoment
) const
{
    const Time& time = motion.time();
    
    scalar t = time.value();
    scalar tprev = t - time.deltaTValue();
    
    pointField fairPos = vectorField(int(nCouplingDof_/3), vector::zero);
    vectorField fairForce =vectorField(int(nCouplingDof_/3), vector::zero);
    
    // If coupling mode is externalRigidBody, X and fairPos are in fact body's 6DoF motion;
    // Flines is mooring forces and moments on the body (may need to reverse the sign).
    double* X = &fairPos[0][0];
    double* Flines = &fairForce[0][0];
    
    if (mode_ == word("externalPoint"))
    {
        // Calculate fairlead position
        for(int pt=0; pt<refAttachmentPt_.size(); pt++)
        {
            fairPos[pt] = motion.transform(refAttachmentPt_[pt]);
        }
    }
    else
    {
        point CoM = motion.centreOfMass();
        vector rotationAngle
        (
           quaternion(motion.orientation()).eulerAngles(quaternion::XYZ)
        );
        
        for (int ii=0; ii<3; ii++)
        {
           X[ii] = CoM[ii];
           X[ii+3] = rotationAngle[ii];
        }
    }
    
    Info<< "\n\ttprev = " << tprev << ", X[6]/fairPosition: " << fairPos << endl;  

    if (!initialized_)
    {
		// Initialize Moody
        // int moodyInit(const char* fName, int nVals, double initialValues[], double startTime );
        //moodyInit(fname_.c_str(), nCouplingDof_,  &fairPos_[0][0], tprev);
        
        moodyInit(fname_.c_str(), nCouplingDof_, X, tprev);

		if (waveKinematics_)
		{
			nNodes_ = moodyGetNumberOfPoints();
    
            Info << "\tEnabled setting flow info (wave kinematics). Total # of mooring points passing in from Moody: " 
                 << nNodes_ << endl;

			nodesPos_ = vectorField(nNodes_, vector::zero);
			velNodes_ = vectorField(nNodes_, vector::zero);
			accelNodes_ = vectorField(nNodes_, vector::zero);
            rhoF_ = scalarField(nNodes_, 0);
		}

		Info<< "\tMoody module initialized!\n" << endl;
		initialized_ = true;
    }

	if (waveKinematics_)
	{   
        // void moodyGetPositions(int nPointsIn, double xyz[] );
		moodyGetPositions(nNodes_, &nodesPos_[0][0]);

        Info<< "\tFirst 2 nodes positions: " << nodesPos_[0] << nodesPos_[1]
            << "; Last 2 nodes: " << nodesPos_[nNodes_-2] << nodesPos_[nNodes_-1]
		    << endl;
            
        // void moodySetFlow(const double t, int nPoints, const double vF[] ,const double aF[],const double rhoF[]);
		moodySetFlow(t, nNodes_, &velNodes_[0][0], &accelNodes_[0][0], &rhoF_[0]);
	}

    // void moodySolve(const double X[], double F[], double t1, double t2);
    moodySolve(X, Flines, tprev, t);
    
    //if (mode_ == word("externalPoint"))
    //    moodySolve(&fairPos[0][0], &fairForce[0][0], tprev, t);
    
    if (mode_ == word("externalPoint"))
    {        
        // Sum forces and calculate moments
        point CoR = motion.centreOfRotation();
    
        for(int pt=0; pt<refAttachmentPt_.size(); pt++)
        {
            restraintForce -= fairForce[pt];
            
            restraintMoment -= (fairPos[pt] - CoR) ^ fairForce[pt];
        }
    }
    else
    {
        for(int i=0;i<3;i++)
        {
            // Reverse sign
            restraintForce[i] = -Flines[i]; 
            restraintMoment[i] = -Flines[i+3];
        }
        // Alternatively,
        // restraintForce = fairForce[0]; restraintMoment = fairForce[1];
    }
    
    // Since moment is already calculated by Moody, set to
    // centreOfRotation to be sure of no spurious moment
    restraintPosition = motion.centreOfRotation();

    if (motion.report())
    {
        Info<< t << ": force " << restraintForce
	        << ", moment " << restraintMoment
            << endl;
    }
}


bool Foam::sixDoFRigidBodyMotionRestraints::moodyR::read
(
    const dictionary& sDoFRBMRDict
)
{
    sixDoFRigidBodyMotionRestraint::read(sDoFRBMRDict);
    sDoFRBMRCoeffs_.readEntry("inputFile", fname_);
    sDoFRBMRCoeffs_.readEntry("couplingMode", mode_);
    nCouplingDof_ = sDoFRBMRCoeffs_.getOrDefault<scalar>("nCouplingDof", 6);
    if (mode_ == word("externalPoint"))
        sDoFRBMRCoeffs_.readEntry("refAttachmentPt", refAttachmentPt_);
    waveKinematics_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("waveKinematics", false);
    twoD_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("twoD", false);

    return true;
}


void Foam::sixDoFRigidBodyMotionRestraints::moodyR::write
(
    Ostream& os
) const
{
    os.writeEntry("inputFile", fname_);
    os.writeEntry("couplingMode", mode_);
    os.writeEntry("nCouplingDof", nCouplingDof_);
    if (mode_ == word("externalPoint"))
        os.writeEntry("refAttachmentPt", refAttachmentPt_);
    os.writeEntry("waveKinematics", waveKinematics_);
    os.writeEntry("twoD", twoD_);
    os.writeEntry("nNodes", nNodes_);

}

void Foam::sixDoFRigidBodyMotionRestraints::moodyR::updateWaveKinematics(const Time& time) const
{
	const fvMesh& mesh = time.lookupObject<fvMesh>("region0");
	const volVectorField& U = mesh.lookupObject<volVectorField>("U");
    const volScalarField& rho = mesh.lookupObject<volScalarField>("rho");
    
    velNodes_ *= 0;
    accelNodes_ *= 0;
    rhoF_ *= 0;
    
    scalar yMid = 0.005;
    if (twoD_)
    {
        const boundBox& globalBb = mesh.bounds();
        yMid = (globalBb.max().y() + globalBb.min().y())/2;
    }
    
	label count = 0;
	forAll(nodesPos_, ni)
	{
		point p = nodesPos_[ni];

		// If 2D simulation, set y-coordinate to mid of y-span
		// p[1] = twoD_? 0.005 : p[1];
		if (twoD_) { p[1] = yMid; }

		label celli = mesh.findCell(p);
		if (celli != -1)
		{
			velNodes_[ni] = U[celli];
			accelNodes_[ni] = fvc::ddt(U)()[celli];
            rhoF_[ni] = rho[celli];
		}
		else
        {
            // Info << "Not found node[" << ni << "] = " << p << endl;
		    count++;
        }
	}

    Info << "\tMesh cells not found for " << count << " mooring points in moodyR::updateWaveKinematics." <<endl;
}

/*
localCellId [Pstream::myProcNo()]= mesh().findCell(location);
reduce(localCellId, sumOp<List<label> >());
*/

// ************************************************************************* //
