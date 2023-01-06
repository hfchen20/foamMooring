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
    Foam::sixDoFRigidBodyMotionRestraints::moorDynR2

Description
    sixDoFRigidBodyMotionRestraints model.  Mooring dynamics resolved.

    Calling MoorDyn: a lumped-mass based mooring dynamics model by
    M. Hall and A. Goupee, “Validation of a lumped-mass mooring line model with
    DeepCwind semisubmersible model test data,” Ocean Engineering, vol. 104,
    pp. 590–603, Aug. 2015.

    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with mooring dynamics: Coupling 
    MoorDyn with OpenFOAM. Applied Ocean Research, 124, 103210.
    https://doi.org/10.1016/j.apor.2022.103210

SourceFiles
    moorDynR2.C

\*---------------------------------------------------------------------------*/

#ifndef moorDynR2_H
#define moorDynR2_H

#include "sixDoFRigidBodyMotionRestraint.H"
#include "MoorDyn2.h"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{

namespace sixDoFRigidBodyMotionRestraints
{

/*---------------------------------------------------------------------------*\
                          Class moorDynR2 Declaration
\*---------------------------------------------------------------------------*/

class moorDynR2
:
    public sixDoFRigidBodyMotionRestraint
{
    // Private data

	//- Initialize MoorDyn at the first time step ()
        mutable bool initialized_;


public:

    //- Runtime type information
    TypeName("moorDynR2");


    // Constructors

        //- Construct from components
        moorDynR2
        (
            const word& name,
            const dictionary& sDoFRBMRDict
        );

        //- Construct and return a clone
        virtual autoPtr<sixDoFRigidBodyMotionRestraint> clone() const
        {
            return autoPtr<sixDoFRigidBodyMotionRestraint>
            (
                new moorDynR2(*this)
            );
        }


    //- Destructor
    virtual ~moorDynR2();


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

private:

    // The MoorDyn system instance
    MoorDyn moordyn_;
    // The coupled body to be controlled by this MoorDyn instance
    MoorDynBody moordyn_body_;
};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace solidBodyMotionFunctions
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //