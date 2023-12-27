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

Author
    Niels G. Jacobsen, Deltares

Modifications    
    Haifei Chen - December 2023
      Added support for multiple lines and VTK files generation at runtime
    
\*---------------------------------------------------------------------------*/

#include "waves2FoamMooring.H"
#include "addToRunTimeSelectionTable.H"
#include "sixDoFRigidBodyMotion.H"
#include "OFstream.H"
#include "foamVtkSeriesWriter.H"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace sixDoFRigidBodyMotionRestraints
{
    defineTypeNameAndDebug(waves2FoamMooring, 0);

    addToRunTimeSelectionTable
    (
        sixDoFRigidBodyMotionRestraint,
        waves2FoamMooring,
        dictionary
    );
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::waves2FoamMooring::waves2FoamMooring
(
    const word& name,
    const dictionary& sDoFRBMRDict
)
:
    sixDoFRigidBodyMotionRestraint(name, sDoFRBMRDict),
    anchor_(),
    refAttachmentPt_(),
    mass_(),
    length_(),
    gMag_(),
    gravityVector_(),
    unitVert_(),
    simpleState_("simpleState"),
    restingState_("restingState"),
    hangingState_("hangingState")
    //mooringState_("NULL")
{
    Info << "Create waves2FoamMooring restraint ..." << endl;
        
    read(sDoFRBMRDict);
        
    mooringState_ =  wordList(nLines_, "NULL");
    
    vtkCounter_ = 0;
    curTime_ = -1;
    iteration_ = 0;
            
    if (writeVTK_)
    {
        mkDir("Mooring/VTK");
        
        Info<< "To write mooring VTK starting from time " << vtkStartTime_
            << ", for the last outerCorrector only " <<  outerCorrector_
            << endl;
    }
    
    if (writeForces_)
    {
        tensionFile_.reset( new OFstream(fileName("Mooring/wfoamForce.dat")) );
        // Writing header
        tensionFile_() << "Time history of mooring forces at fairlead: # lines " << nLines_ << endl;
    }
}


// * * * * * * * * * * * * * * * * Destructors * * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::waves2FoamMooring::~waves2FoamMooring()
{
    // combine vtk files into a series
    vtk::seriesWriter writer;

    writer.scan("Mooring/VTK/" + vtkPrefix_ + ".vtk");

    Info<< "Writing vtk series file" << nl;

    fileName vtkSeries("Mooring/VTK/" + vtkPrefix_ + ".vtk");
    writer.write(vtkSeries);

}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

Foam::pointField Foam::sixDoFRigidBodyMotionRestraints::waves2FoamMooring::centreLine
(
    const point& start,
    const point& end,
    const label li
) const
{
    //setState(start, end);
    
    word state = mooringState_[li];

    //Info << "  centreLine for line " << li << ", state: " << state << endl;
    
    label nCells_ = nodesPerLine_[li];
    pointField cl(nCells_+1, vector::zero);
    
    if (state == simpleState_)
    {
        // Create the catenary shape
        catenaryShape cs(start, end, lengthList_[li], massList_[li], gravityVector_);

        // Calculate the centreline points
        cs.centreLine(cl);
    }
    else if (state == restingState_)
    {
        // Find the touchdown point
        point E = end;
        scalar newLength(0);
        restingLength(start, E, newLength);

        // Calculate the catenary shape between fairlead and seabed
        catenaryShape cs(start, E, newLength, massList_[li], gravityVector_);

        // Remove one point from the centreline and calculate centreline points
        cl.setSize(nCells_);

        cs.centreLine(cl);

        // Add the endpoint to the centreline - the horizontal part of the line
        cl.setSize(nCells_ + 1);
        cl[nCells_] = end;
    }
    else if (state == hangingState_)
    {
        point midPoint = start + ((end - start) & unitVert_)*unitVert_;

        label N0 = nCells_/2;
        label N1 = nCells_ + 1 - N0;

        for (int i = 0; i < N0; i++)
        {
            scalar factor = static_cast<scalar>(i)/static_cast<scalar>(N0);
            cl[i] = start + factor*(midPoint - start);
        }

        for (int i = 0; i < N1; i++)
        {
            scalar factor = static_cast<scalar>(i)/static_cast<scalar>(N1 - 1);
            cl[i + N0] = midPoint + factor*(end - midPoint);
        }
    }
    else
    {
        notImplemented("The state is not defined.");
    }

/*
    // Expand the centerline to a four points with the centre line in the middle
    pointField pp(cl.size()*4, point::zero);

    DELETED

*/
    
    return cl;
}


void Foam::sixDoFRigidBodyMotionRestraints::waves2FoamMooring::setState
(
    const point& start,
    const point& end,
    const label li
) const
{
    vector span = (start - end);
    span -= (span & unitVert_)*unitVert_;

    // Making sure that fairlead is not above anchor point
    if (Foam::mag(span) < 0.001*length_)
    {
        mooringState_[li] = hangingState_;

        return;
    }

    // Investigate, whether the angle at the fairlead exceeds 88 degrees.
    // Inside scope for cleanness
    {
        // Maximum top slope is set to 88 degrees
        scalar topSlope(Foam::tan(maxAngle_/180.*M_PI));
        scalar lk2 = Foam::asinh(topSlope);

        // Shape parameter (assuming 88 degrees at the top)
        scalar k =
            1.0/Foam::mag((start - end) & unitVert_)*(Foam::cosh(lk2) - 1.0);

        // Half length of the sag
        scalar L2 = 1/k*topSlope;

        // Half span distance of the sag
        scalar l2 = Foam::asinh(L2*k)/k;

        scalar maximumLength = L2 + Foam::mag(span) - l2;

        if (maximumLength < length_)
        {
            mooringState_[li] = hangingState_;

            return;
        }
    }

    // Investigate, whether the "top" point is in between the fairlead and the
    // anchor points.
    // It is now 'safe' to create the catenary shape, because it is known that
    // the line is not in a hanging state
    catenaryShape cs(start, end, length_, mass_, gravityVector_);

    if (cs.isUShaped())
    {
        mooringState_[li] = restingState_;
    }
    else
    {
        mooringState_[li] = simpleState_;
    }
    
}


void Foam::sixDoFRigidBodyMotionRestraints::waves2FoamMooring::restingLength
(
    const point& start,
    point& end,
    scalar& lNew
) const
{
    scalar lengthNew = 0;

    // Horizontal and vertical span
    vector span = end - start;
    span -= (span & unitVert_)*unitVert_;

    scalar h = Foam::mag((end - start) & unitVert_);

    // Define limits for the search
    scalar alphaMax = maxAngle_/180.0*M_PI;

    // The lower limit is based on the angle for the catenary shape, that has
    // a horizontal gradient at the anchor point
    scalar alphaMin = 0.0;

    // Enclosed in a scope for cleanness
    {
        scalar kMin = 1.0e-10;
        // The factor 1.1 is just to be sure
        scalar kMax = 1.1*Foam::mag(Foam::asinh(Foam::tan(alphaMax)))/Foam::mag(span);

        while (true)
        {
            scalar kAv = 0.5*(kMin + kMax);

            scalar valAv = -h*kAv - (1.0 - Foam::cosh(kAv*Foam::mag(span)));

            if (Foam::mag(valAv) < 1e-7)
            {
                alphaMin = Foam::atan(Foam::sinh(kAv*Foam::mag(span)));
                break;
            }

            if (valAv < 0.0)
            {
                kMin = kAv;
            }
            else
            {
                kMax = kAv;
            }
        }
    }

    // Find the resting length and the touchdown point
    label count = 0;

    while (true)
    {
        scalar alphaAv = 0.5*(alphaMin + alphaMax);

        scalar slope = tan(alphaAv);
        scalar lk2 = Foam::asinh(slope);
        scalar k = 1.0/h*(Foam::cosh(lk2) - 1);
        scalar L2 = 1.0/(k)*slope;
        scalar l2 = asinh(L2*k)/k;

        scalar totalLength = L2 + Foam::mag(span) - l2;

        if (totalLength < length_)
        {
            alphaMin = alphaAv;
        }
        else
        {
            alphaMax = alphaAv;
        }

        if (Foam::mag(totalLength - length_) < 1.0e-7*length_ || count == 100)
        {
            lengthNew = L2;

            end = start + l2*span/Foam::mag(span) - h*unitVert_;

            break;
        }
        count++;
    }

    lNew = lengthNew;
}


void Foam::sixDoFRigidBodyMotionRestraints::waves2FoamMooring::restrain
(
    const sixDoFRigidBodyMotion& motion,
    vector& restraintPosition,
    vector& restraintForce,
    vector& restraintMoment
) const
{
    const Time& time = motion.time();
    scalar t = time.value();
    if (t == curTime_)
    {
        iteration_++;
    }
    else if (t > curTime_)
    {
        iteration_ = 1;
        curTime_ = t;
    }
    
    point CoR = motion.centreOfRotation();
    
    tmp<pointField> tpoints = motion.transform(refAttachmentPt_);
    pointField fairPos = tpoints();
    
    //Info<< "fairlead coord " << fairPos << endl;
    
    vectorField fairForce(nLines_, vector::zero);
    
    // Set restraint position, force and moment
    restraintForce = vector::zero;
    restraintMoment = vector::zero;
    
    // if there is only one mooring line, we can set the attachment point
    // as the restraintPosition, and let motion solver calculate the moment;
    // but if there is more than one restraint, we sum the forces/moments
    // manually and set the restraintPosition as body CoR

    for(int i=0; i<nLines_; i++)
    {
        // Rename
        point pos0 = fairPos[i];
        point pos1 = anchor_[i];

        vector fi = vector::zero;

        // Set the state of the mooring line
        setState(fairPos[i], anchor_[i], i);
        
        // deal with state for all the lines
        word state = mooringState_[i];

        // Compute the force as a function of the state
        if (state == simpleState_)
        {
            // Get access to the catenary shape
            catenaryShape cs(pos0, pos1, lengthList_[i], massList_[i], gravityVector_);

            // Get the forces
            vector H0 = cs.H0();
            vector R0 = cs.R0();

            fi = (H0 + R0);
        }
        else if (state == restingState_)
        {
            // Find the touchdown point
            point E = pos1;
            scalar newLength(0);
            restingLength(pos0, E, newLength);

            // Calculate the catenary shape between fairlead and seabed
            catenaryShape cs(pos0, E, newLength, massList_[i], gravityVector_);

            // Get the forces
            vector H0 = cs.H0();
            vector R0 = cs.R0();

            fi = (H0 + R0);
        }
        else if (state == hangingState_)
        {
            // The vertical force is equal to the submerged weight of the line
            // hanging from the fairlead to the seabed (assumed at the level of
            // pos1)
            fi = ((pos1 - pos0) & unitVert_)*unitVert_*massList_[i]*gMag_;
        }
        else
        {
            notImplemented("The force calculation is not implemented");
        }
        
        fairForce[i] = fi;
        
        Info<< "  fairlead[" << i << "]: " << fairPos[i]
            << ", force " << fairForce[i]  
            << endl;
        
        restraintForce += fi;
        restraintMoment += (fairPos[i] - CoR) ^ fi;
        
    }
    
    // Since moment is already calculated as above, set to
    // centreOfRotation to be sure of no spurious moment
    restraintPosition = motion.centreOfRotation();

    if (motion.report())
    {
        Info<< "  sum mooring force " << restraintForce
            << ", moment " << restraintMoment
            << endl;
    }
    
    if (iteration_ == outerCorrector_)
    {
        if (writeForces_)
        {
            // Write fairlead force to file
            tensionFile_() << t;
            for(int pt=0; pt<nLines_; pt++)
            {
                tensionFile_() << "\t" << fairForce[pt];
            }  
            tensionFile_() << endl;
        }

        if (writeVTK_)
        {
            if (t >= vtkStartTime_ && time.outputTime())
            {
                Info<< "  Write moorings VTK ..." << endl;
                writeVTK(time, fairPos);
            }
        }
    }

}


bool Foam::sixDoFRigidBodyMotionRestraints::waves2FoamMooring::read
(
    const dictionary& sDoFRBMRDict
)
{
    sixDoFRigidBodyMotionRestraint::read(sDoFRBMRDict);

    sDoFRBMRCoeffs_.readEntry("anchor", anchor_);
    sDoFRBMRCoeffs_.readEntry("refAttachmentPt", refAttachmentPt_);
    
    nLines_ = refAttachmentPt_.size();

    if (nLines_ != anchor_.size())
    {
        FatalErrorInFunction
            << "Sizes of anchor or refAttachmentPt unequal"
            << abort(FatalError);
    }
    
    identical_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("identicalProperties", true);

    if (identical_)
    {
        // read a scalar and assign
        sDoFRBMRCoeffs_.lookup("massPerLength") >> mass_;
        sDoFRBMRCoeffs_.lookup("lineLength") >> length_;
        
        massList_ = scalarField(nLines_, mass_);
        lengthList_ = scalarField(nLines_, length_);
    }
    else
    {
        sDoFRBMRCoeffs_.readEntry("massPerLength", massList_);
        sDoFRBMRCoeffs_.readEntry("lineLength", lengthList_);
    }

    sDoFRBMRCoeffs_.lookup("gravityVector") >> gravityVector_;
    
    gMag_ = Foam::mag(gravityVector_);
    unitVert_ = gravityVector_/gMag_;
    unitVert_ = Foam::cmptMultiply(unitVert_, unitVert_);
    
    writeForces_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("writeForce", false);
    writeVTK_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("writeVTK", false);
    
    if (writeVTK_)
    {
        vtkPrefix_ = sDoFRBMRCoeffs_.getOrDefault<word>("vtkPrefix", this->name());
        vtkStartTime_ = sDoFRBMRCoeffs_.getOrDefault<scalar>("vtkStartTime", 0);
        outerCorrector_ = sDoFRBMRCoeffs_.getOrDefault<scalar>("outerCorrector", 3);
        
        nNodes_ = sDoFRBMRCoeffs_.getOrDefault<label>("nNodes", 10);
        if (sDoFRBMRCoeffs_.found("nodesPerLine"))
        {
            sDoFRBMRCoeffs_.readEntry("nodesPerLine", nodesPerLine_);
            
            if (nodesPerLine_.size() != nLines_)
            {
                    FatalErrorInFunction
                    << "Sizes of nodesPerLine unequal to nLines"
                    << abort(FatalError);
            }
        }
        else
        {
            nodesPerLine_ = labelList(nLines_, nNodes_);
        }
    }

    return true;
}


void Foam::sixDoFRigidBodyMotionRestraints::waves2FoamMooring::write
(
    Ostream& os
) const
{  
    os.writeEntry("anchor", anchor_);
    os.writeEntry("refAttachmentPt", refAttachmentPt_);
    os.writeEntry("identicalProperties", identical_);
    
    if (identical_)
    {
        os.writeEntry("massPerLength", mass_);
        os.writeEntry("lineLength", length_);
    }
    else
    {
        os.writeEntry("massPerLength", massList_);
        os.writeEntry("lineLength", lengthList_);
    }
    
    os.writeEntry("gravityVector", gravityVector_);
    
    os.writeEntry("writeForce", writeForces_);
    os.writeEntry("writeVTK", writeVTK_);

    if (writeVTK_)
    {
        os.writeEntry("vtkPrefix", vtkPrefix_);
        os.writeEntry("vtkStartTime", vtkStartTime_);
        os.writeEntry("outerCorrector", outerCorrector_);
        
        if (sDoFRBMRCoeffs_.found("nNodes"))
        {
            os.writeEntry("nNodes", nNodes_);
        }
        if (sDoFRBMRCoeffs_.found("nodesPerLine"))
        {
            os.writeEntry("nodesPerLine", nodesPerLine_);
        }

    }

}

void Foam::sixDoFRigidBodyMotionRestraints::waves2FoamMooring::writeVTK(
    const Time& time,
    const pointField& fairPos
) const
{
    fileName name("Mooring/VTK/" + vtkPrefix_ + "_");
    OFstream mps(name + Foam::name(++vtkCounter_) + ".vtk");
    mps.precision(4);

    // Writing header
    mps << "# vtk DataFile Version 3.0" << nl << "waves2FoamMooring vtk output time=" << time.timeName() 
        << nl << "ASCII" << nl << "DATASET POLYDATA" << endl;
 
    // Writing points
    mps << "\nPOINTS " << sum(nodesPerLine_+1) << " float" << endl;

    for(int i=0; i<nLines_; i++)
    {
        pointField coord = centreLine(fairPos[i], anchor_[i], i);

        for(int p=0; p<coord.size(); p++)
        //for(int p=0; p<nodesPerLine_[i]; p++)
        {
            mps << coord[p][0] << " " << coord[p][1] << " " << coord[p][2] << endl;
        }
    }
    
    // Writing lines, seems each line having nodesPerLine_+1 nodes
    mps << "\nLINES " << nLines_ << " " << sum(nodesPerLine_+2) << endl;

    label start_node(0);
    for(int i=0; i<nLines_; i++)
    {       
        mps << nodesPerLine_[i]+1;
        
        for(int j=0; j<nodesPerLine_[i]+1; j++)
        {
            mps << " " << start_node+j;
        }
        mps << endl;
        
        start_node += nodesPerLine_[i]+1;
    }

}


// ************************************************************************* //
