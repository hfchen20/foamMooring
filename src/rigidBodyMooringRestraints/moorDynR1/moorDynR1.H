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

Class
    Foam::RBD::restraints::moorDynR1

Group
    grpRigidBodyDynamicsRestraints

Description
    sixDoFRigidBodyMotionRestraints model.  Mooring dynamics resolved.

    Calling MoorDyn: a lumped-mass based mooring dynamics model by
    M. Hall and A. Goupee, “Validation of a lumped-mass mooring line model with
    DeepCwind semisubmersible model test data,” Ocean Engineering, vol. 104,
    pp. 590–603, Aug. 2015.

SourceFiles
    moorDynR1.C

References
    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with
    mooring dynamics: Coupling MoorDyn with OpenFOAM. Applied Ocean
    Research, 124, 103210. https://doi.org/10.1016/j.apor.2022.103210
    
    Chen, H., Medina, T. A., & Cercos-Pita, J. L. (2024). CFD simulation of multiple
    moored floating structures using OpenFOAM: An open-access mooring restraints 
    library. Ocean Engineering, 303, 117697.
    https://doi.org/10.1016/j.oceaneng.2024.117697
    
\*---------------------------------------------------------------------------*/

#ifndef RBD_restraints_moorDynR1_H
#define RBD_restraints_moorDynR1_H

#include "rigidBodyRestraint.H"
#include "pointField.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{
namespace RBD
{
namespace restraints
{

/*---------------------------------------------------------------------------*\
                          Class moorDynR1 Declaration
\*---------------------------------------------------------------------------*/

class moorDynR1
:
    public restraint
{
    // Private data

        //- Reference point of attachment to the solid body
        pointField refAttachmentPt_;

        //- List of body names for each attachment point. Default is empty list.
		// Allow for multiple bodies to be attached to the same restraint
        List<word> bodies_;		
                
        //- List of body IDs for each attachment point. Default is bodyID_ for all.
        List<label> bodyIDs_;		

        //- List of body indices for each attachment point. Default is bodyID_ for all.
        List<label> bodyIndices_;

        mutable bool initialized_;



public:

    //- Runtime type information
    TypeName("moorDynR1");


    // Constructors

        //- Construct from components
        moorDynR1
        (
            const word& name,
            const dictionary& dict,
            const rigidBodyModel& model
        );

        //- Construct and return a clone
        virtual autoPtr<restraint> clone() const
        {
            return autoPtr<restraint>
            (
                new moorDynR1(*this)
            );
        }


    //- Destructor
    virtual ~moorDynR1();


    // Member Functions

        //- Accumulate the restraint internal joint forces into the tau field and
        //  external forces into the fx field
        virtual void restrain
        (
            scalarField& tau,
            Field<spatialVector>& fx,
            const rigidBodyModelState& state
        ) const;
        
        //- Transform body point for a given bodyID
        // Necessary to resolve restraints associated with multiple bodies
        inline point bodyPoint(const point& p, const label bodyID) const
        {
            return (model_.X0(bodyID).inv() && spatialVector(Zero, p)).l();
        }

        //- Transform body point velocity for a given bodyID
        // Necessary to resolve restraints associated with multiple bodies
        inline spatialVector bodyPointVelocity(const point& p, const label bodyID) const
        {
            return model_.v(bodyID, p);
        }

        //- Update properties from given dictionary
        virtual bool read(const dictionary& dict);

        //- Write
        virtual void write(Ostream&) const;
};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace restraints
} // End namespace RBD
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //
