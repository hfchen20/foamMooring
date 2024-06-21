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
#include "rigidBodyModel.H"
#include "addToRunTimeSelectionTable.H"
#include "foamVtkSeriesWriter.H"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace RBD
{
namespace restraints
{
    defineTypeNameAndDebug(linearSpringGroup, 0);

    addToRunTimeSelectionTable
    (
        restraint,
        linearSpringGroup,
        dictionary
    );
}
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::RBD::restraints::linearSpringGroup::linearSpringGroup
(
    const word& name,
    const dictionary& dict,
    const rigidBodyModel& model
)
:
    restraint(name, dict, model),
    numberOfSprings_(),
    anchor_(),
    refAttachmentPt_(),
    stiffness_(),
    damping_(),
    restLength_()
{
    oldForce_ = Zero;
    oldMoment_ = Zero;

    vtkCounter_ = 0;
    curTime_ = -1;
    iteration_ = 0;

    read(dict);

    Info << "Created linearSpringGroup, # Springs " << numberOfSprings_
         << ", writeTension " << writeTension_
         << ", writeVTK " << writeVTK_
         << ", allow compression " << compression_
         << ", frelax " << frelax_
         << endl;

    if (Pstream::master())
    {
        if (writeVTK_)
        {
            mkDir("Mooring/VTK");
        }

        if (writeTension_)
        {
            word fname(vtkPrefix_ + "_tension.dat");
            tensionFile_.reset(new OFstream(fileName("Mooring/" + fname)));
            // Writing header
            tensionFile_()<< "Time history of springs' tension. # springs: "
                << numberOfSprings_ << endl;
        }
    }
}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::RBD::restraints::linearSpringGroup::~linearSpringGroup()
{
    if (Pstream::master() && writeVTK_)
    {
        vtk::seriesWriter writer;

        writer.scan("Mooring/VTK/" + vtkPrefix_ + ".vtk");

        Info<< "Writing springs vtk series file" << nl;

        fileName vtkSeries("Mooring/VTK/" + vtkPrefix_ + ".vtk");
        writer.write(vtkSeries);
    }
}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

void Foam::RBD::restraints::linearSpringGroup::restrain
(
    scalarField& tau,
    Field<spatialVector>& fx,
    const rigidBodyModelState& state
) const
{
    static bool first = true;
    const Time& time = model_.time();

    scalar t = state.t();
    if (t == curTime_)
    {
        iteration_++;
    }
    else if (t > curTime_)
    {
        iteration_ = 1;
        curTime_ = t;
    }

    vector force = Zero;
    vector moment = Zero;

    scalarField tension = scalarField(numberOfSprings_, 0.0);
    vectorField fairPos = vectorField(numberOfSprings_, vector::zero);

    for(int pt=0; pt<numberOfSprings_; pt++)
    {
        fairPos[pt] = bodyPoint(refAttachmentPt_[pt], bodyIDs_[pt]);

        // Current axis of the spring
        vector r = fairPos[pt] - anchor_[pt];
        scalar magR = mag(r);
        r /= (magR + VSMALL);

        // Velocity of the attached end of the spring
        vector v = bodyPointVelocity(refAttachmentPt_[pt], bodyIDs_[pt]);

        scalar delta = magR - restLength_;

        if (compression_)
            tension[pt] =  stiffness_*delta;
        else
            tension[pt] = (delta > VSMALL) ? stiffness_*delta : 0.0;

        // Force and moment on the master body including optional damping
        vector fi = -tension[pt]*r - damping_*(r & v)*r;

        force += fi;
        moment += fairPos[pt] ^ fi;
    }

    // Relax force/moment on all but first iteration
    if (!first)
    {
        force = frelax_*force + (1-frelax_)*oldForce_;
        moment = frelax_*moment + (1-frelax_)*oldMoment_;
    }
    else
    {
        first = false;
    }

    oldForce_ = force;
    oldMoment_ = moment;

    Info<< " tension @ spring0 " << tension[0]
        << " sum force " << force
        << " sum moment " << moment
        << endl;

    // Accumulate the force for the restrained body
    // Move into numberOfSprings_ for loop?
    fx[bodyIndex_] += spatialVector(moment, force);

    if (iteration_ == outerCorrector_)
    {
        if (writeTension_)
        {
            // Write spring tension to file
            tensionFile_() << t;
            for(int pt=0; pt<numberOfSprings_; pt++)
            {
                tensionFile_() << "  " << tension[pt];
            }
            tensionFile_() << endl;
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


bool Foam::RBD::restraints::linearSpringGroup::read
(
    const dictionary& dict
)
{
    restraint::read(dict);

    coeffs_.readEntry("numberOfSprings", numberOfSprings_);
    coeffs_.readEntry("anchor", anchor_);
    coeffs_.readEntry("refAttachmentPt", refAttachmentPt_);

    label nAttachments = refAttachmentPt_.size();

    // Initialise the sizes of bodyIDs, and bodyIndices
    bodyIDs_ = List<label>(nAttachments, bodyID_);
    bodyIndices_ = List<label>(nAttachments, bodyIndex_);
        
    if (numberOfSprings_ != nAttachments || numberOfSprings_ != anchor_.size())
    {
        FatalErrorInFunction
            << "Size of anchor or refAttachmentPt not equal to numberOfSprings"
            << abort(FatalError);
    }

    // If the moorings are attached to different bodies
    if (coeffs_.found("bodies") )
    {
        coeffs_.lookup("bodies") >> bodies_;
        if (bodies_.size() != nAttachments)
        {
            FatalErrorInFunction
                << "Number of attachment points not equal to number of bodies " << bodies_
                << abort(FatalError);
        }

        for(int ii=0; ii<nAttachments; ii++)
        {
            bodyIDs_[ii] = model_.bodyID(bodies_[ii]);
            bodyIndices_[ii] = model_.master(bodyIDs_[ii]);
        }
    }

    identicalSprings_ = coeffs_.getOrDefault<Switch>("identicalSprings", true);

    if (identicalSprings_)
    {
        coeffs_.readEntry("stiffness", stiffness_);
        coeffs_.readEntry("damping", damping_);
        coeffs_.readEntry("restLength", restLength_);
    }
    else
        FatalErrorInFunction
            << "Inhomogeneous springs not implemented yet!"
            << abort(FatalError);

    writeTension_ =
        coeffs_.getOrDefault<Switch>
        (
            "writeForce",
            coeffs_.getOrDefault<Switch>("writeTension", false)
        );
    writeVTK_ = coeffs_.getOrDefault<Switch>("writeVTK", false);
    compression_ = coeffs_.getOrDefault<Switch>("compression", false);
    frelax_ = coeffs_.getOrDefault<scalar>("frelax", 0.8);

    if (writeVTK_)
    {
        vtkPrefix_ = coeffs_.getOrDefault<word>("vtkPrefix", this->name());
        vtkStartTime_ = coeffs_.getOrDefault<scalar>("vtkStartTime", 0);
        outerCorrector_ = coeffs_.getOrDefault<scalar>("outerCorrector", 3);
    }

    return true;
}


void Foam::RBD::restraints::linearSpringGroup::write
(
    Ostream& os
) const
{
    restraint::write(os);

    os.writeEntry("numberOfSprings", numberOfSprings_);
    os.writeEntry("anchor", anchor_);
    os.writeEntry("refAttachmentPt", refAttachmentPt_);
    os.writeEntry("identicalSprings", identicalSprings_);
    os.writeEntry("stiffness", stiffness_);
    os.writeEntry("damping", damping_);
    os.writeEntry("restLength", restLength_);
    os.writeEntry("writeTension", writeTension_);
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

void Foam::RBD::restraints::linearSpringGroup::writeVTK
(
    const Time& time,
    const pointField& fairPos
) const
{
    int nLines = numberOfSprings_;
    labelList nodesPerLine(nLines, 2);

    fileName name("Mooring/VTK/" + vtkPrefix_);
    OFstream mps(name + word::printf("_%04d", vtkCounter_++) + ".vtk");

    mps.precision(4);

    // Writing header
    mps << "# vtk DataFile Version 3.0" << nl
        << "linearSpringGroup vtk output time=" << time.timeName() << nl
        << "ASCII" << nl
        << "DATASET POLYDATA" << endl;

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

