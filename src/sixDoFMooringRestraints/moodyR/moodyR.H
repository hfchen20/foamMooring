/*---------------------------------------------------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | www.openfoam.com
     \\/     M anipulation  |
-------------------------------------------------------------------------------
    Copyright (C) 2019 OpenCFD Ltd.
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

Class
    Foam::sixDoFRigidBodyMotionRestraints::moodyR

Description
    sixDoFRigidBodyMotionRestraints model.  Mooring dynamics resolved.


    J. Palm, C. Eskilsson, and L. Bergdahl. An hp-adaptive discontinuous Galerkin
    method for modelling snap loads in mooring cables. Ocean Engineering, 144:266–
    276, 2017.

    J. Palm, C. Eskilsson, G. Paredes and L. Bergdahl,
    Coupled mooring analysis for floating wave energy converters using CFD: Formulation 
    and validation, Int. J. of Marine Energy, 16. 83-99 (2016)

SourceFiles
    moodyR.C

References

    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with mooring dynamics: 
        Coupling MoorDyn with OpenFOAM. Applied Ocean Research, 124, 103210.
        https://doi.org/10.1016/j.apor.2022.103210
        
\*---------------------------------------------------------------------------*/

#ifndef moodyR_H
#define moodyR_H

#include "fvCFD.H"

#include "sixDoFRigidBodyMotionRestraint.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{
// Forward declarations
class fvMesh;

namespace sixDoFRigidBodyMotionRestraints
{

/*---------------------------------------------------------------------------*\
                          Class moodyR Declaration
\*---------------------------------------------------------------------------*/

class moodyR
:
    public sixDoFRigidBodyMotionRestraint
{
    // Private data
		
        //- Moody input filename
        fileName fname_;
        
        //- Coupling mode: 'externalPoint', otherwise 'externalRigidBody'
        word mode_; 
        
        //- Fairlead initial position
        pointField refAttachmentPt_;
        
        //- Number of coupling Dof
        // 'externalPoint': nCouplingDof_ = nRefPt x 3
        // 'externalRigidBody': nCouplingDof_ = 6
        scalar nCouplingDof_;
        
        //- Flag for extract from CFD and send fluid velocity/acceleration to Moody
		bool waveKinematics_;
	
        //- Flag for 2D simulation (x, z). If true, set nodesPos[y-cmpt] equal to
        //  mid y-coordinate of the 2D computational domain
        bool twoD_;
	
	    //- Number of total nodes for all mooring lines
        mutable scalar nNodes_;
		
		//- Array of mooring nodes coordinates
        mutable	vectorField nodesPos_;
		
		//-  Fluid velocity at mooring nodes
		mutable vectorField velNodes_;
		
		//-  Fluid accel at mooring nodes
		mutable vectorField accelNodes_;
        
        //-  Fluid density
		mutable scalarField rhoF_;
        
		//- Initialize Moody at the first time step
        mutable bool initialized_;


public:

    //- Runtime type information
    TypeName("moodyR");


    // Constructors

        //- Construct from components
        moodyR
        (
            const word& name,
            const dictionary& sDoFRBMRDict
        );

        //- Construct and return a clone
        virtual autoPtr<sixDoFRigidBodyMotionRestraint> clone() const
        {
            return autoPtr<sixDoFRigidBodyMotionRestraint>
            (
                new moodyR(*this)
            );
        }


    //- Destructor
    virtual ~moodyR();


    // Member Functions

        //- Calculate the restraint position, force and moment.
        //  Global reference frame vectors.
        virtual void restrain
        (
            const sixDoFRigidBodyMotion& motion,
            vector& restraintPosition,
            vector& restraintForce,
            vector& restraintMoment
        ) const;

        //- Update properties from given dictionary
        virtual bool read(const dictionary& sDoFRBMRCoeff);

        //- Write
        virtual void write(Ostream&) const;

		//- update wave kinematics (fluid velocity & acceleration @ nodes coordinates)
        void updateWaveKinematics(const Time&) const;

};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace solidBodyMotionFunctions
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //
