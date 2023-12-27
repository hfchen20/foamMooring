/*---------------------------------------------------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | Copyright (C) 2011-2013 OpenFOAM Foundation
     \\/     M anipulation  |
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
    Foam::sixDoFRigidBodyMotionRestraints::waves2FoamMooring

Description
    sixDoFRigidBodyMotionRestraints model.  Mooring Line
    
    Example specification in dynamicMeshDict:
    \verbatim
     restraints
     {
        catenaryLine
        {            
            sixDoFRigidBodyMotionRestraint  waves2FoamMooring;
            anchor
            (
                (-0.889    0.3725    -0.6)
                (-0.889   -0.3725    -0.6)
                ( 0.889    0.3725    -0.6)
                ( 0.889   -0.3725    -0.6)  
            );
            refAttachmentPt
            (
                (-0.25     0.3725   -0.16)
                (-0.25    -0.3725   -0.16)
                ( 0.25     0.3725   -0.16)
                ( 0.25    -0.3725   -0.16)  
            );
            identicalProperties true; // if false, specify a list for mass and line length
            massPerLength       0.149; // Wet mass/unit length
            lineLength          0.809; // Length of the mooring line
            gravityVector       (0 0 -9.81); // Gravity vector
            writeForce          true; // optional, default false
            writeVTK            true; // optional, default false
            //nNodes              10; // optional, default 10
            //nodesPerLine        (8 6 7 9); // optional, default (nNodes); if present overwrite nNodes 
            //vtkPrefix           "wfoamMoor"; // optional, default restraint name
            //vtkStartTime        0; // optional, default 0
            //outerCorrector      3; // optional, default 3
        }
     }
    \endverbatim

SourceFiles
    waves2FoamMooring.C

Author
    Niels G. Jacobsen, Deltares

Modifications    
    Haifei Chen - December 2023
      Added support for multiple lines and VTK files generation at runtime

\*---------------------------------------------------------------------------*/

#ifndef waves2FoamMooring_H
#define waves2FoamMooring_H

#include "sixDoFRigidBodyMotionRestraint.H"
#include "fvCFD.H"
#include "catenaryShape.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{

namespace sixDoFRigidBodyMotionRestraints
{

/*---------------------------------------------------------------------------*\
                          Class waves2FoamMooring Declaration
\*---------------------------------------------------------------------------*/

class waves2FoamMooring
:
    public sixDoFRigidBodyMotionRestraint
{
    // Private data

        //- Anchor point
        pointField anchor_;

        //- Reference point of attachment to the solid body
        pointField refAttachmentPt_;

        //- Number of mooring lines
        label nLines_;
        
        //- Are the moorings having the same properties: mass, length, etc.
        bool identical_;

        //- Mass per unit length
        scalar mass_;

        //- Length of the mooring line
        scalar length_;
        
        //- Mass per unit length for all lines
        scalarList massList_;

        //- Length of all the mooring lines
        scalarList lengthList_;

        //- Magnitude of the acceleration due to gravity
        scalar gMag_;

        //- Gravity vector
        vector gravityVector_;

        //- Unit vertical direction
        vector unitVert_;
        
        //-  Write mooring fairlead forces to file
        bool writeForces_;

        //- Write mooring line coordinates to VTK files
        bool writeVTK_;
        
        //- Mooring vtk filename prefix
        word vtkPrefix_;

        //- Start time to write VTK, default to 0
        scalar vtkStartTime_;

        //- Sequence number to be included in vtk filename
        mutable label vtkCounter_;

        //- Nodes # for each mooring line
        label nNodes_;

        //- List of nodes # per mooring line
        mutable List<label> nodesPerLine_;

        //- Current time, to count iteration
        mutable scalar curTime_;

        //- Iteration count, write VTK for the last iteration only
        mutable label iteration_;

        //- Outer corrector, times mooring states updated each time step
        label outerCorrector_;
        
        //- Output stream for fairlead forces
        mutable autoPtr<OFstream> tensionFile_;
        
        //- Maximum slope of the line with respect to vertical
        const scalar maxAngle_ = 88.0;

        //- Keywords for mooring line states
        word simpleState_;
        word restingState_;
        word hangingState_;
        
        //- States for all mooring lines
        mutable wordList mooringState_;

        //- Checks the state of the line li
        void setState(const point&, const point&, const label li) const;

        //- Returns the length of suspended line in restingState_
        void restingLength(const point&, point&, scalar&) const;

public:

    //- Runtime type information
    TypeName("waves2FoamMooring");


    // Constructors

        //- Construct from components
        waves2FoamMooring
        (
            const word& name,
            const dictionary& sDoFRBMRDict
        );

        //- Construct and return a clone
        virtual autoPtr<sixDoFRigidBodyMotionRestraint> clone() const
        {
            return autoPtr<sixDoFRigidBodyMotionRestraint>
            (
                new waves2FoamMooring(*this)
            );
        }


    //- Destructor
    virtual ~waves2FoamMooring();


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

        //- Write the centre line
        pointField centreLine(const point&, const point&, const label) const;
        
        //- Write VTK files for mooring lines configuration
        void writeVTK(const Time&, const pointField&) const;

};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace solidBodyMotionFunctions
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //
