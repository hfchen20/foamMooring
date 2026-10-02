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
    Foam::sixDoFRigidBodyMotionRestraints::cableDynR

Description
    CableDyn mooring restraint
    
    J. H. Seo, J. Lim, K. Shim, J. Song. CableDyn: Curvature-resolving implicit 
    finite-element analysis of mooring lines and dynamic power cables for floating 
    offshore wind. Ocean Engineering 368, 128332, 2026.    
    https://doi.org/10.1016/j.oceaneng.2026.128332
    
SourceFiles
    cableDynR.C

References
    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with
    mooring dynamics: Coupling MoorDyn with OpenFOAM. Applied Ocean
    Research, 124, 103210. https://doi.org/10.1016/j.apor.2022.103210
    
    Chen, H., Medina, T. A., & Cercos-Pita, J. L. (2024). CFD simulation of multiple
    moored floating structures using OpenFOAM: An open-access mooring restraints 
    library. Ocean Engineering, 303, 117697.
    https://doi.org/10.1016/j.oceaneng.2024.117697

\*---------------------------------------------------------------------------*/

#ifndef cableDynR_H
#define cableDynR_H

#include "sixDoFRigidBodyMotionRestraint.H"
#include "pointField.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{

namespace sixDoFRigidBodyMotionRestraints
{

/*---------------------------------------------------------------------------*\
                          Class cableDynR Declaration
\*---------------------------------------------------------------------------*/

class cableDynR
:
    public sixDoFRigidBodyMotionRestraint
{
    // Private data

        //- Mooring input filename
        fileName inputFile_;

        //- Reference point of attachment to the solid body
        pointField refAttachmentPt_;
        
        //- Start index of fairlead points in CableDyn model
        label cplPtStartIndex_;
        
        //- Void pointer to CableDyn handler
        void* handle_=NULL;
        
        //- Number of coupling Dofs
        label nCouplingDof_;
        
        //- Flag true if CableDyn is initialized
        mutable bool initialized_;


public:

    //- Runtime type information
    TypeName("cableDynR");


    // Constructors

        //- Construct from components
        cableDynR
        (
            const word& name,
            const dictionary& sDoFRBMRDict
        );

        //- Construct and return a clone
        virtual autoPtr<sixDoFRigidBodyMotionRestraint> clone() const
        {
            return autoPtr<sixDoFRigidBodyMotionRestraint>
            (
                new cableDynR(*this)
            );
        }


    //- Destructor
    virtual ~cableDynR();


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

};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace solidBodyMotionFunctions
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //
