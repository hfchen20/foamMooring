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
    Foam::RBD::restraints::moodyR

Group
    grpRigidBodyDynamicsRestraints

Description
    Mooring dynamics resolved.

    J. Palm, C. Eskilsson, and L. Bergdahl. An hp-adaptive discontinuous Galerkin
    method for modelling snap loads in mooring cables. Ocean Engineering, 144:266–
    276, 2017.

    J. Palm, C. Eskilsson, G. Paredes and L. Bergdahl,
    Coupled mooring analysis for floating wave energy converters using CFD: Formulation
    and validation, Int. J. of Marine Energy, 16. 83-99 (2016)

SourceFiles
    moodyR.C

References
    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with
    mooring dynamics: Coupling MoorDyn with OpenFOAM. Applied Ocean
    Research, 124, 103210. https://doi.org/10.1016/j.apor.2022.103210
    
    Chen, H., Medina, T. A., & Cercos-Pita, J. L. (2024). CFD simulation of multiple
    moored floating structures using OpenFOAM: An open-access mooring restraints 
    library. Ocean Engineering, 303, 117697.
    https://doi.org/10.1016/j.oceaneng.2024.117697

    moodyR
    {
        type                   moodyR;
        body                   floatingBody;

        inputFile              "Mooring/boxWu_exPoint.m";
        
        refAttachmentPt
        (
            (-0.1      0.1    -0.077)
            (-0.1     -0.1    -0.077)
            ( 0.1      0.1    -0.077)
            ( 0.1     -0.1    -0.077)
        );

        // If bodies present, refAttachmentPt are associated with multiple bodies
        bodies (bodyA bodyA bodyA bodyB);
        // bodies.size == refAttachmentPt.size
        
        // which one is better?
        bodies (bodyA bodyB);
        refAttachmentPt
        (
            bodyA
            (
                (-0.1      0.1    -0.077)
                (-0.1     -0.1    -0.077)
                ( 0.1      0.1    -0.077)
            )
            bodyB
            (
                ( 0.1     -0.1    -0.077)
            )  
        );
    }

\*---------------------------------------------------------------------------*/

#ifndef RBD_restraints_moodyR_H
#define RBD_restraints_moodyR_H

#include "fvCFD.H"
#include "rigidBodyRestraint.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{
namespace RBD
{
namespace restraints
{

/*---------------------------------------------------------------------------*\
                          Class moodyR Declaration
\*---------------------------------------------------------------------------*/

class moodyR
:
    public restraint
{
    // Private data

        //- Moody input filename
        fileName fname_;

        //- List of body names for each attachment point. Default is empty list.
        // Allow for multiple bodies to be attached to the same moodyR restraint:
        List<word> bodies_;		
                
        //- List of body IDs for each attachment point. Default is bodyID_ for all.
        List<label> bodyIDs_;		

        //- List of body indices for each attachment point. Default is bodyID_ for all.
        List<label> bodyIndices_;

        //- Reference point of attachment to the solid body
        pointField refAttachmentPt_;

        //- Moody API coupling ndof, support only couplingMode "externalPoint"
        label nCouplingDof_;

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
            const dictionary& dict,
            const rigidBodyModel& model
        );

        //- Construct and return a clone
        virtual autoPtr<restraint> clone() const
        {
            return autoPtr<restraint>
            (
                new moodyR(*this)
            );
        }


    //- Destructor
    virtual ~moodyR();


    // Member Functions

        //- Accumulate the restraint internal joint forces into the tau field and
        //  external forces into the fx field
        virtual void restrain
        (
            scalarField& tau,
            Field<spatialVector>& fx,
            const rigidBodyModelState& state
        ) const;

        // Transform of the previous state body point for a given bodyID
        // Necessary to resolve restraints associated with multiple bodies
        inline point bodyPoint(const point& p,const label bodyID) const
        {
            return (model_.X0(bodyID).inv() && spatialVector(Zero, p)).l();
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
