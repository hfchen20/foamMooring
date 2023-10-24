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
    Foam::RBD::restraints::linearSpringGroup

Group
    grpRigidBodyDynamicsRestraints

Description
    Linear spring restraint.

SourceFiles
    linearSpringGroup.C

    linearSpringGroup
    {
        type                 linearSpringGroup;
        body                 floatingBody;

        anchor
        (
            (-0.4009  0.34 -0.6)
            (-0.4009 -0.34 -0.6)
            ( 0.4009  0.34 -0.6)
            ( 0.4009 -0.34 -0.6)
        );
        refAttachmentPt
        (
            (-0.2    0.34 -0.252)
            (-0.2   -0.34 -0.252)
            ( 0.2    0.34 -0.252)
            ( 0.2   -0.34 -0.252)
        );

        numberOfSprings       4;

        // identicalSprings = true, specify a scalar for stiffness, damping, restLength
        // otherwise, speicfy a list for each spring
        identicalSprings      true;

        stiffness             1.0E6;
        damping               8000;
        restLength            0.4017923;

        writeForce            true;
        writeVTK              true;
        compression           true; // false = no spring force when compressed
    }
\*---------------------------------------------------------------------------*/

#ifndef RBD_restraints_linearSpringGroup_H
#define RBD_restraints_linearSpringGroup_H

#include "rigidBodyRestraint.H"
#include "fvCFD.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{

class OFstream;

namespace RBD
{
namespace restraints
{

/*---------------------------------------------------------------------------*\
                          Class linearSpringGroup Declaration
\*---------------------------------------------------------------------------*/

class linearSpringGroup
:
    public restraint
{
    // Private data

        //- Number of springs in the group
        scalar numberOfSprings_;

        //- Anchor point, where the spring is attached to an immovable object
        pointField anchor_;

        //- Reference point of attachment to the solid body
        pointField refAttachmentPt_;

        //- Are the springs having the same properties: stiffness, damping, restLength?
        bool identicalSprings_;

        // Assume identical springs at the moment
        // non-identical springs to be implemented

        //- Spring stiffness coefficient (N/m)
        scalar stiffness_;

        //- Damping coefficient (Ns/m)
        scalar damping_;

        //- Rest length - length of spring when no forces are applied to it
        scalar restLength_;

        //-  Write springs' tension to file
        bool writeForce_;

        //-  Write springs' position to file
        bool writePos_;

        //-  Write springs' position to vtk
        bool writeVTK_;

        //- Mooring vtk filename prefix
        word vtkPrefix_;

        //- Start time to write VTK, default to runTime.start
        scalar vtkStartTime_;

        //- Sequence number to be included in vtk filename
        mutable label vtkCounter_;

        //- Current time, to count iteration
        mutable scalar curTime_;

        //- Iteration count, write VTK for the last iteration only
        mutable label iteration_;

        //- Outer corrector, times mooring states updated each time step
        label outerCorrector_;

        //- Allow spring compression
        bool compression_;

        //- Output stream for springs' tension
        mutable autoPtr<OFstream> tensionFile_;

        //- Output stream for springs' attachment position
        mutable autoPtr<OFstream> posFile_;

        //- Cache old restrain force
        mutable vector oldForce_;

        //- Cache old restrain moment
        mutable vector oldMoment_;

        //- Relaxation factor
        scalar frelax_;


public:

    //- Runtime type information
    TypeName("linearSpringGroup");


    // Constructors

        //- Construct from components
        linearSpringGroup
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
                new linearSpringGroup(*this)
            );
        }


    //- Destructor
    virtual ~linearSpringGroup();


    // Member Functions

        //- Accumulate the restraint internal joint forces into the tau field and
        //  external forces into the fx field
        virtual void restrain
        (
            scalarField& tau,
            Field<spatialVector>& fx,
            const rigidBodyModelState& state
        ) const;

        //- Update properties from given dictionary
        virtual bool read(const dictionary& dict);

        //- Write
        virtual void write(Ostream&) const;

        //- Output vtk files for mooring
        void writeVTK(const Time&, const pointField&) const;
};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace restraints
} // End namespace RBD
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //

