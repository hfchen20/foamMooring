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
    Foam::RBD::restraints::map3R

Group
    grpRigidBodyDynamicsRestraints

Description
    sixDoFRigidBodyMotionRestraints model.  Quasi-static mooring model.

    M., Masciola, J., Jonkman, and A. Robertson. Implementation of a multisegmented, 
    quasi-static cable model. In The Twenty-third International Offshore and Polar 
    Engineering Conference. International Society of Offshore and Polar Engineers, 2013

SourceFiles
    map3R.C

References
    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with
    mooring dynamics: Coupling MoorDyn with OpenFOAM. Applied Ocean
    Research, 124, 103210. https://doi.org/10.1016/j.apor.2022.103210
    
    Chen, H., Medina, T. A., & Cercos-Pita, J. L. (2024). CFD simulation of multiple
    moored floating structures using OpenFOAM: An open-access mooring restraints 
    library. Ocean Engineering, 303, 117697.
    https://doi.org/10.1016/j.oceaneng.2024.117697

\*---------------------------------------------------------------------------*/

#ifndef RBD_restraints_map3R_H
#define RBD_restraints_map3R_H

#include "fvCFD.H"
#include "rigidBodyRestraint.H"
#include "mapFoamInterface.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{
namespace RBD
{
namespace restraints
{

/*---------------------------------------------------------------------------*\
                          Class map3R Declaration
\*---------------------------------------------------------------------------*/

class map3R
:
    public restraint
{
    // Private data

        //- mapFoamInterface object: initialie, update, and close MAP calls
        mutable mapFoamInterface map_;
        
        //- MAP intput file
        fileName inputFile_;
        
        //- MAP summary file
        fileName summaryFile_;
        
        //- MAP mooring fairlead force file
        fileName outputFile_;

        //- Water depth
        scalar depth_;
        
        //- List of body names for each attachment point. Default is empty list.
        // Allow for multiple bodies to be attached to the same moodyR restraint:
        List<word> bodies_;		

        //- List of body IDs for each attachment point. Default is bodyID_ for all.
        List<label> bodyIDs_;		

        //- List of body indices for each attachment point. Default is bodyID_ for all.
        List<label> bodyIndices_;

        //- Number of mooring lines, not equal to fairleads if connecting nodes present
        mutable scalar nLines_;

        //- Number of mooring fairleads
        mutable scalar nFairleads_;

        //- Initialize MoorDyn at the first time step
        mutable bool initialized_;

        //- Reference point of attachment to the solid body
        pointField refAttachmentPt_;

        //-  Write mooring fairlead forces to file
        bool writeForces_;

        //-  Write mooring fairlead forces to file
        bool writeVTK_;

        //- Start time to write VTK, default to runTime.start
        scalar vtkStartTime_;

        //- Write interval (sec) to output VTK
        //scalar vtkInterval_;

        //- Sequence number to be included in vtk filename
        mutable label vtkCounter_;

        //- nodes # for all lines
        label nNodes_;

        //- List of nodes # per mooring line
        //scalarField nodesPerLine_;
        mutable List<label> nodesPerLine_;

        //- Current time, to count iteration
        mutable scalar curTime_;

        //- Iteration count, write VTK for the last iteration only
        mutable label iteration_;
        
        //- Outer corrector, times mooring states updated each time step
        label outerCorrector_;

        //- Output stream for mooring fairlead forces
        mutable autoPtr<OFstream> mps_;

        //- Output stream for mooring VTK
        //mutable autoPtr<OFstream> vtk_;

public:

    //- Runtime type information
    TypeName("map3R");


    // Constructors

        //- Construct from components
        map3R
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
                new map3R(*this)
            );
        }


    //- Destructor
    virtual ~map3R();


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
        inline point bodyPoint(const point& p,const label bodyID) const
        {
            return (model_.X0(bodyID).inv() && spatialVector(Zero, p)).l();
        }
        
        //- Update properties from given dictionary
        virtual bool read(const dictionary& dict);

        //- Write
        virtual void write(Ostream&) const;

        //- Initialize MAP++ & assign some variables
        void initializeMAP(const Time&) const;

        //- Update MAP++ state
        void updateMAPState(const Time&) const;

        //- Output vtk files for mooring
        void writeVTK(const Time&) const;
};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace restraints
} // End namespace RBD
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //
