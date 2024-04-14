/*---------------------------------------------------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | www.openfoam.com
     \\/     M anipulation  |
-------------------------------------------------------------------------------
    Copyright (C) 2011-2016 OpenFOAM Foundation
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

References
    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with
    mooring dynamics: Coupling MoorDyn with OpenFOAM. Applied Ocean
    Research, 124, 103210. https://doi.org/10.1016/j.apor.2022.103210
    
    Chen, H., Medina, T. A., & Cercos-Pita, J. L. (2024). CFD simulation of multiple
    moored floating structures using OpenFOAM: An open-access mooring restraints 
    library. Ocean Engineering, 303, 117697.
    https://doi.org/10.1016/j.oceaneng.2024.117697
    
\*---------------------------------------------------------------------------*/

#include "linearSpringGroup.H"
#include "addToRunTimeSelectionTable.H"
#include "sixDoFRigidBodyMotion.H"
#include "OFstream.H"
#include "foamVtkSeriesWriter.H"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace sixDoFRigidBodyMotionRestraints
{
    defineTypeNameAndDebug(linearSpringGroup, 0);

    addToRunTimeSelectionTable
    (
        sixDoFRigidBodyMotionRestraint,
        linearSpringGroup,
        dictionary
    );
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::linearSpringGroup::linearSpringGroup
(
    const word& name,
    const dictionary& sDoFRBMRDict
)
:
    sixDoFRigidBodyMotionRestraint(name, sDoFRBMRDict),
    numberOfSprings_(),
    anchor_(),
    refAttachmentPt_(),
    stiffness_(),
    damping_(),
    restLength_()
{
    oldRestraintForce_ = Zero;
    oldRestraintMoment_ = Zero;

    vtkCounter_ = 0;
    curTime_ = -1;
    iteration_ = 0;
    
    read(sDoFRBMRDict);

    if (writeVTK_)
    {
        mkDir("Mooring/VTK");
    }

    if (writeForce_)
    {
        tensionFile_.reset( new OFstream(fileName("Mooring/tension.dat")) );
        // Writing header
        tensionFile_() << "Time history of springs' tension. # springs: " << numberOfSprings_ << endl;
    }

    if (writePos_)
    {
        posFile_.reset( new OFstream(fileName("Mooring/position.dat") ) );
        posFile_() << "Time history of springs' attachment points. # springs: " << numberOfSprings_ << endl;     
    }

    Info << "Created linearSpringGroup, # Springs " << numberOfSprings_ 
         << ", writeForce " << writeForce_
         << ", writeVTK " << writeVTK_
         << ", allow compression " << compression_
         << ", frelax " << frelax_
         << endl;
}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::linearSpringGroup::~linearSpringGroup()
{
    // combine vtk files into a series
    {
        vtk::seriesWriter writer;

        writer.scan("Mooring/VTK/" + vtkPrefix_ + ".vtk");
        
        Info<< "Writing springs vtk series file" << nl;

        fileName vtkSeries("Mooring/VTK/" + vtkPrefix_ + ".vtk");
        writer.write(vtkSeries);
    }

}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

void Foam::sixDoFRigidBodyMotionRestraints::linearSpringGroup::restrain
(
    const sixDoFRigidBodyMotion& motion,
    vector& restraintPosition,
    vector& restraintForce,
    vector& restraintMoment
) const
{
    static bool first = true;
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

    scalarField tension = scalarField(numberOfSprings_, 0.0);

    // Since this is a group of springs, calculate moment manually
    // and set restraintPosition to CoR

    for(int pt=0; pt<numberOfSprings_; pt++)
    {
        vector r = fairPos[pt] - anchor_[pt];

        scalar magR = mag(r);
        r /= (magR + VSMALL);
        
        vector v = motion.velocity(fairPos[pt]);

        scalar delta = magR - restLength_;

        if (compression_)
            tension[pt] =  stiffness_*delta;
        else
            tension[pt] = (delta > VSMALL) ? stiffness_*delta : 0.0;

        // vector fi = -stiffness_*delta*r - damping_*(r & v)*r;
        vector fi = -tension[pt]*r - damping_*(r & v)*r;

        // Sum forces and moments
        restraintForce += fi;
        
        restraintMoment += (fairPos[pt] - CoR) ^ fi;
        
    }

    // Relax force/moment on all but first iteration
    if (!first)
    {
        restraintForce = frelax_*restraintForce + (1-frelax_)*oldRestraintForce_;
        restraintMoment = frelax_*restraintMoment + (1-frelax_)*oldRestraintMoment_;
    }
    else
    {
        first = false;
    }
    
    oldRestraintForce_ = restraintForce;
    oldRestraintMoment_ = restraintMoment;

    // Since moment is already calculated as above, set to
    // centreOfRotation to be sure of no spurious moment
    restraintPosition = motion.centreOfRotation();

    if (motion.report())
    {
        Info<< " tension @ spring0 " << tension[0]
            << " sum force " << restraintForce
            << " sum moment " << restraintMoment
            << endl;
    }

    if (iteration_ == outerCorrector_)
    {
        if (writeForce_)
        {
            // Write spring tension to file
            tensionFile_() << t;
            for(int pt=0; pt<numberOfSprings_; pt++)
            {
                tensionFile_() << "\t" << tension[pt];
            }  
            tensionFile_() << endl;
        }

        if (writePos_)
        {
            // Write spring attachment pts to file
            posFile_() << t;
            for(int pt=0; pt<numberOfSprings_; pt++)
            {
                vector pos = fairPos[pt];
                posFile_() << "\t" << pos[0] << "\t" << pos[1] << "\t" << pos[2];
            }  
            posFile_() << endl;
        }

        if (writeVTK_)
        {
            if (t >= vtkStartTime_ && time.outputTime())
            {
                Info<< "Write springs VTK ..." << endl;
                writeVTK(time, fairPos);
            }
        }
    }

}


bool Foam::sixDoFRigidBodyMotionRestraints::linearSpringGroup::read
(
    const dictionary& sDoFRBMRDict
)
{
    sixDoFRigidBodyMotionRestraint::read(sDoFRBMRDict);

    sDoFRBMRCoeffs_.readEntry("numberOfSprings", numberOfSprings_);
    sDoFRBMRCoeffs_.readEntry("anchor", anchor_);
    sDoFRBMRCoeffs_.readEntry("refAttachmentPt", refAttachmentPt_);

    if (numberOfSprings_ != refAttachmentPt_.size() || numberOfSprings_ != anchor_.size())
    {
        FatalErrorInFunction
            << "Size of anchor or refAttachmentPt unequal to numberOfSprings"
            << abort(FatalError);
    }

    identicalSprings_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("identicalSprings", true);

    if (identicalSprings_)
    {
        // read a scalar and assign to stiffness, damping, restLength
        sDoFRBMRCoeffs_.readEntry("stiffness", stiffness_);
        sDoFRBMRCoeffs_.readEntry("damping", damping_);
        sDoFRBMRCoeffs_.readEntry("restLength", restLength_);
    }

    // non-identical springs to be implemented
    
    writeForce_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("writeForce", true);
    writePos_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("writePosition", false);
    writeVTK_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("writeVTK", false);
    compression_ = sDoFRBMRCoeffs_.getOrDefault<Switch>("compression", false);
    frelax_ = sDoFRBMRCoeffs_.getOrDefault<scalar>("frelax", 0.8);

    if (writeVTK_)
    {
        vtkPrefix_ = sDoFRBMRCoeffs_.getOrDefault<word>("vtkPrefix", "sprGroup");
        vtkStartTime_ = sDoFRBMRCoeffs_.getOrDefault<scalar>("vtkStartTime", 0);
        outerCorrector_ = sDoFRBMRCoeffs_.getOrDefault<scalar>("outerCorrector", 3);
    }

    return true;
}


void Foam::sixDoFRigidBodyMotionRestraints::linearSpringGroup::write
(
    Ostream& os
) const
{
    os.writeEntry("numberOfSprings", numberOfSprings_);
    os.writeEntry("anchor", anchor_);
    os.writeEntry("refAttachmentPt", refAttachmentPt_);
    os.writeEntry("identicalSprings", identicalSprings_);
    os.writeEntry("stiffness", stiffness_);
    os.writeEntry("damping", damping_);
    os.writeEntry("restLength", restLength_);
    os.writeEntry("writeForce", writeForce_);
    os.writeEntry("writePosition", writePos_);
    os.writeEntry("writeVTK", writeVTK_);
    os.writeEntry("compression", compression_);
    os.writeEntryIfDifferent<scalar>("frelax", 0.8, frelax_);

    if (writeVTK_)
    {
        os.writeEntry("vtkPrefix", vtkPrefix_);
        os.writeEntry("vtkStartTime", vtkStartTime_);
        os.writeEntry("outerCorrector", outerCorrector_);
    }
}

void Foam::sixDoFRigidBodyMotionRestraints::linearSpringGroup::writeVTK
(
    const Time& time,
    const pointField& fairPos
) const
{
    int nLines = numberOfSprings_;
    labelList nodesPerLine(nLines, 2);

    fileName name("Mooring/VTK/" + vtkPrefix_ + "_");
    OFstream mps(name + Foam::name(++vtkCounter_) + ".vtk");
    mps.precision(4);

    // Writing header
    mps << "# vtk DataFile Version 3.0" << nl << "linearSpringGroup vtk output time=" << time.timeName() 
        << nl << "ASCII" << nl << "DATASET POLYDATA" << endl;
 
    // Writing points
    mps << "\nPOINTS " << 2 * nLines << " float" << endl;

    for(int i=0; i<nLines; i++)
    {
        mps << anchor_[i][0] << " " << anchor_[i][1] << " " << anchor_[i][2] << endl;
        mps << fairPos[i][0] << " " << fairPos[i][1] << " " << fairPos[i][2] << endl;
    }
    
    // Writing lines
    mps << "\nLINES " << nLines << " " << sum(nodesPerLine+1) << endl;

    label start_node(0);
    for(int i=0; i<nLines; i++)
    {       
        mps << nodesPerLine[i];
        
        for(int j=0; j<nodesPerLine[i]; j++)
        {
            mps << " " << start_node+j;
        }
        mps << endl;
        
        start_node += nodesPerLine[i];
    }

}
// ************************************************************************* //
