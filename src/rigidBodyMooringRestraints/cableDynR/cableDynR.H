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
    Foam::RBD::restraints::cableDynR

Group
    grpRigidBodyDynamicsRestraints

Description
    Mooring restraint using CableDyn

SourceFiles
    cableDynR.C

Example usage
    cableDyn
    {
        type              cableDynR;
        inputFile         "Mooring/cabledyn.txt";
        body              floatingBody;
        refAttachmentPt
        (
            (-0.2    0.34 -0.252)
            (-0.2   -0.34 -0.252)
            ( 0.2    0.34 -0.252)
            ( 0.2   -0.34 -0.252)
        );
        cplPtStartIndex   4;
    }
    
References
    J. H. Seo, J. Lim, K. Shim, J. Song. CableDyn: Curvature-resolving implicit 
    finite-element analysis of mooring lines and dynamic power cables for floating 
    offshore wind. Ocean Engineering 368, 128332, 2026.    
    https://doi.org/10.1016/j.oceaneng.2026.128332
    
\*---------------------------------------------------------------------------*/

#ifndef RBD_restraints_cableDynR_H
#define RBD_restraints_cableDynR_H

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
                          Class cableDynR Declaration
\*---------------------------------------------------------------------------*/

class cableDynR
:
    public restraint
{
    // Private data

        //- Mooring input filename
        fileName inputFile_;
        
        //- Reference point of attachment to the solid body
        pointField refAttachmentPt_;

        //- List of body names for each attachment point. Default is empty list.
        // Allow for multiple bodies to be attached to the same restraint
        List<word> bodies_;
                
        //- List of body IDs for each attachment point. Default is bodyID_ for all.
        List<label> bodyIDs_;
        
        //- List of body indices for each attachment point. Default is bodyID_ for all.
        List<label> bodyIndices_;

        //- Void pointer to CableDyn handler
        void* handle_=NULL;

        //- Flag true if CableDyn is initialized
        mutable bool initialized_;
        
        //- Start index of fairlead points in CableDyn model
        label cplPtStartIndex_;



public:

    //- Runtime type information
    TypeName("cableDynR");


    // Constructors

        //- Construct from components
        cableDynR
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
                new cableDynR(*this)
            );
        }


    //- Destructor
    virtual ~cableDynR();


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
        inline point bodyPoint(const label bodyID, const point& p) const
        {
            return (model_.X0(bodyID).inv() && spatialVector(Zero, p)).l();
            
            // Obtain the spatial transform from local-to-global frame
            // X00() calculates the absolute current transformation matrix to the global frame
            // Foam::RBD::spatialTransform localToGlobal = motion.X00(bodyID);

            // Transform the point to get its actual global position
            // vector globalPosition = localToGlobal.transformPoint(localPoint);
        }

        //- Transform body point velocity for a given bodyID
        // inline spatialVector bodyPointVelocity(const point& p, const label bodyID) const
        // {
        //     return model_.v(bodyID, p);
        // }
        
        //- Transform body point acceleration for a given bodyID
        inline vector pointAcceleration(const label bodyID, const point& p) const
        {
            const label mbd = model_.master(bodyID);
            return
            (
                (
                    spatialTransform
                    (
                        model_.X0(mbd).E().T(),
                        model_.masterPoint(bodyID, p)
                    )
                  & model_.a(mbd)
                )
              + spatialVector(Zero, model_.g())
            ).l();
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
