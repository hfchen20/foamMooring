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
    Foam::sixDoFRigidBodyMotionRestraints::map3R

Description
    sixDoFRigidBodyMotionRestraints model.  Quasi-static mooring model.

    M., Masciola, J., Jonkman, and A. Robertson. Implementation of a multisegmented, 
    quasi-static cable model. In The Twenty-third International Offshore and Polar 
    Engineering Conference. International Society of Offshore and Polar Engineers, 2013

    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with mooring dynamics: Coupling 
    MoorDyn with OpenFOAM. Applied Ocean Research, 124, 103210.
    https://doi.org/10.1016/j.apor.2022.103210

SourceFiles
    map3R.C

\*---------------------------------------------------------------------------*/

#ifndef map3R_H
#define map3R_H

#include "fvCFD.H"
#include "sixDoFRigidBodyMotionRestraint.H"

#include "mapFoamInterface.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

//class mapFoamInterface;

namespace Foam
{
// Forward declarations
class fvMesh;

namespace sixDoFRigidBodyMotionRestraints
{

/*---------------------------------------------------------------------------*\
                          Class map3R Declaration
\*---------------------------------------------------------------------------*/

class map3R
:
    public sixDoFRigidBodyMotionRestraint
{
    // Private data
    
    //- mapFoamInterface object, initialie, update, and close MAP calls
    mutable mapFoamInterface map_;
    
    //- MAP intput file
    fileName inputFile_;
    
    //- MAP summary file
    fileName summaryFile_;
    
    //- Water depth
    scalar depth_;
    
    //- Number of mooring lines, = # fairleads
    mutable scalar nLines_;

    //- Number of segments per line, assuming each line has same number of segments
    scalar nSegments_;

    //- Write mooring line coordinates to VTK files
    bool writeVTK_;

    //- Initialize MoorDyn at the first time step ()
    mutable bool initialized_;
    
    //- Fairlead initial position, relative to body center of rotation
    // size equal to nLines_
    pointField refAttachmentPt_;
    
    //- Instantaneous fairlead position
    //pointField fairPos_;
    
    //- Fairlead force
    //vectorField fairForce_;


public:

    //- Runtime type information
    TypeName("map3R");


    // Constructors

        //- Construct from components
        map3R
        (
            const word& name,
            const dictionary& sDoFRBMRDict
        );

        //- Construct and return a clone
        virtual autoPtr<sixDoFRigidBodyMotionRestraint> clone() const
        {
            return autoPtr<sixDoFRigidBodyMotionRestraint>
            (
                new map3R(*this)
            );
        }


    //- Destructor
    virtual ~map3R();


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

	//- write VTK files for mooring lines configuration
        void writeVTK(fvMesh&) const;
};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace solidBodyMotionFunctions
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //
