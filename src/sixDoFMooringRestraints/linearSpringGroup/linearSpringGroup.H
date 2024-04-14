/*---------------------------------------------------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | www.openfoam.com
     \\/     M anipulation  |
-------------------------------------------------------------------------------
    Copyright (C) 2011-2013 OpenFOAM Foundation
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
    Foam::sixDoFRigidBodyMotionRestraints::linearSpringGroup

Description
    sixDoFRigidBodyMotionRestraints model.  Linear spring.

SourceFiles
    linearSpringGroup.C

    linearSpringGroup
    {
        sixDoFRigidBodyMotionRestraint linearSpringGroup;
        
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

References
    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with
    mooring dynamics: Coupling MoorDyn with OpenFOAM. Applied Ocean
    Research, 124, 103210. https://doi.org/10.1016/j.apor.2022.103210
    
    Chen, H., Medina, T. A., & Cercos-Pita, J. L. (2024). CFD simulation of multiple
    moored floating structures using OpenFOAM: An open-access mooring restraints 
    library. Ocean Engineering, 303, 117697.
    https://doi.org/10.1016/j.oceaneng.2024.117697
    
\*---------------------------------------------------------------------------*/

#ifndef linearSpringGroup_H
#define linearSpringGroup_H

#include "sixDoFRigidBodyMotionRestraint.H"
#include "fvCFD.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{

class OFstream;

namespace sixDoFRigidBodyMotionRestraints
{

/*---------------------------------------------------------------------------*\
                          Class linearSpringGroup Declaration
\*---------------------------------------------------------------------------*/

class linearSpringGroup
:
    public sixDoFRigidBodyMotionRestraint
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
        mutable vector oldRestraintForce_;

        //- Cache old restrain moment
        mutable vector oldRestraintMoment_;

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
            const dictionary& sDoFRBMRDict
        );

        //- Construct and return a clone
        virtual autoPtr<sixDoFRigidBodyMotionRestraint> clone() const
        {
            return autoPtr<sixDoFRigidBodyMotionRestraint>
            (
                new linearSpringGroup(*this)
            );
        }


    //- Destructor
    virtual ~linearSpringGroup();


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

        //- Output vtk files for mooring
        void writeVTK(const Time&, const pointField&) const;
};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace solidBodyMotionFunctions
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //
