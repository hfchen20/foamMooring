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

#include "moorDynR2.H"
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
    defineTypeNameAndDebug(moorDynR2, 0);

    addToRunTimeSelectionTable
    (
        restraint,
        moorDynR2,
        dictionary
    );
}
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::RBD::restraints::moorDynR2::moorDynR2
(
    const word& name,
    const dictionary& dict,
    const rigidBodyModel& model
)
:
    restraint(name, dict, model)
{
    read(dict);
    
    initialized_ = false;
    vtkCounter_ = 0;
    curTime_ = -1;
    iteration_ = 0;
    
    Info<< "Creating moorDynR2 using MoorDyn v2." << endl;
    
    moordyn_backup_.t = 0.0;
    moordyn_backup_.data = nullptr;

    // Create the MoorDyn system
    // Check coupling mode: POINT vs BODY
    // nCoupledDofs = nfairleads x 3 or nbodies x 6

    if (Pstream::master())
    {
        int moordyn_err = MOORDYN_SUCCESS;
        moordyn_ = MoorDyn_Create(fname_.c_str());
        if (!moordyn_) {
            FatalErrorInFunction
                << "MoorDyn v2 cannot be created!" << exit(FatalError);
        }
        unsigned int n;
        moordyn_err = MoorDyn_NCoupledDOF(moordyn_, &n);
        if (moordyn_err != MOORDYN_SUCCESS) {
            FatalErrorInFunction
                << "NCoupledDOF error on the MoorDyn definition file" 
                << exit(FatalError);
        }
        nCouplingDof_ = int(n);
        //nCouplingDof_ = n;

        if (couplingMode_ == word("POINT"))
        {
            Info<< "\tCoupling mode: " << couplingMode_ << ", expecting nCouplingDof="
                << 3 * refAttachmentPt_.size() << endl;
            if (nCouplingDof_ != 3 * refAttachmentPt_.size())
            {
                FatalErrorInFunction
                    << " refAttachmentPt size: " << refAttachmentPt_.size() 
                    << " not equal to # coupled points " << int(nCouplingDof_/3)
                    << " defined in MoorDyn input!"
                    << exit(FatalError);
            }
        }
        else if (couplingMode_ == word("BODY"))
        {   
            Info<< "Coupling mode: " << couplingMode_ << ", expecting nCouplingDof=6 * nCoupledBodies"
                << endl;
            WarningInFunction
                << "The 'BODY' coupling mode may NOT work as intended."
                << endl;
            if (nCouplingDof_ != 6 * bodies_.size())
            {
                FatalErrorInFunction
                    << " bodies size " << bodies_.size() 
                    << " not equal to # coupled bodies " << int(nCouplingDof_/6)
                    << " defined in MoorDyn input!"
                    << exit(FatalError);
            } 
        }
        else
        {
            FatalErrorInFunction
                << "Two coupling modes supported only: 'POINT' or 'BODY'. Recommend to use 'POINT'."
                << exit(FatalError);
        }

        // If different bodies are attached to moodyR
        if (coeffs_.found("bodies") )
        {
            Info<< "Multiple bodies specified in moorDynR2 restraint: " << bodies_ << nl
                << " body IDs " << bodyIDs_
                << endl << endl;
        }
        
        if (writeVTK_)
        {
#ifndef MOORDYN2_HAVE_VTK
            if (!legacyVTK_) {
                FatalErrorInFunction
                    << "vtkLegacyFormat=false option has been set. "
                    << "However, MoorDyn v2 has been compiled without VTK "
                    << "support. Please set vtkLegacyFormat=true or install "
                    << "VTK and recompile foamMooring"
                    << exit(FatalError);
            }
#endif
            mkDir("Mooring/VTK");
        }
    }
}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::RBD::restraints::moorDynR2::~moorDynR2()
{
    if (Pstream::master() && initialized_)
    {
        // Close MoorDyn call
        MoorDyn_Close(moordyn_);
        if (moordyn_backup_.data)
            free(moordyn_backup_.data);

        if (legacyVTK_) {
            vtk::seriesWriter writer;

            writer.scan("Mooring/VTK/" + vtkPrefix_ + ".vtk");

            Info<< "Writing mooring vtk series file" << nl;

            fileName vtkSeries("Mooring/VTK/" + vtkPrefix_ + ".vtk");
            writer.write(vtkSeries);
        }
    }
}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

void Foam::RBD::restraints::moorDynR2::restrain
(
    scalarField& tau,
    Field<spatialVector>& fx,
    const rigidBodyModelState& state
) const
{
    int moordyn_err = MOORDYN_SUCCESS;
    
    const Time& time = model_.time();
    scalar t = state.t();
    scalar deltaT = state.deltaT();
    scalar tprev = state.t() - deltaT;
    
    if (t == curTime_)
    {
        iteration_++;
    }
    else if (t > curTime_)
    {
        iteration_ = 1;
        curTime_ = t;
    }
    
    pointField fairPos = vectorField(int(nCouplingDof_/3), vector::zero);
    vectorField fairVel = vectorField(int(nCouplingDof_/3), vector::zero);
    vectorField fairForce =vectorField(int(nCouplingDof_/3), vector::zero);

    // If coupling mode is 'BODY', X and fairPos are in fact body's 6DoF motion.
    // Flines is mooring forces and moments on the body (may need to reverse the sign).
    double* X = &fairPos[0][0];
    double* XD = &fairVel[0][0];
    double* Flines = &fairForce[0][0];

    if (couplingMode_ == word("BODY"))
    {
        // loop through bodies
        forAll(bodyIDs_, b)
        {
            spatialTransform CofR(model_.X0(bodyIDs_[b]));
            spatialVector vCofR(model_.v(bodyIDs_[b], Zero));
            
            point CoR = CofR.r();
            
            vector rotationAngle
            (
                quaternion(CofR.E()).eulerAngles(quaternion::XYZ)
            );
            
            vector v = vCofR.l();
            vector omega = vCofR.w();

            for (int ii=0; ii<3; ii++)
            {
                X[b*6+ii] = CoR[ii];
                X[b*6+ii+3] = rotationAngle[ii];
                XD[b*6+ii] = v[ii];
                XD[b*6+ii+3] = omega[ii];
            }
        }
    }
    else if (couplingMode_ == word("POINT"))
    {
        for (int ii = 0; ii<refAttachmentPt_.size(); ++ii)
        {
            // Transform the starting points to last motion state
            fairPos[ii] = bodyPoint(refAttachmentPt_[ii], bodyIDs_[ii]);

            // fairlead point velocity
            fairVel[ii] = bodyPointVelocity(refAttachmentPt_[ii], bodyIDs_[ii]).l();
        }
    }

    if (!initialized_)
    {
        // Initialize MoorDyn
        moordyn_err = MoorDyn_Init(moordyn_, X, XD);
        if (moordyn_err != MOORDYN_SUCCESS) {
            FatalErrorInFunction
                << "MoorDyn could not be initialized"
                << exit(FatalError);
        }

        Info<< "MoorDyn module initialized!" << endl;
        initialized_ = true;
        save_mooring(tprev);
        if (writeVTK_) {
            autoPtr<Time> dummy_t = Time::New();
            dummy_t->setTime(0.0, 0);
            writeVTK(dummy_t.ref());
        }

        curTime_ = t;
        iteration_ = 1;
    } else if (tprev - moordyn_backup_.t >= 1.e-3 * deltaT) {
        // We have successfully advanced forward in time
        save_mooring(tprev);
        Info<< "MoorDyn module saved at t = " << tprev << " s" << endl;
    } else {
        // We are repeating the same time step because the implicit scheme
        load_mooring();
        Info<< "MoorDyn module restored to t = " << moordyn_backup_.t << " s" << endl;
    }

    // Step MoorDyn to get forces on bodies
    moordyn_err = MoorDyn_Step
    (
        moordyn_, &fairPos[0][0], &fairVel[0][0], &fairForce[0][0], &tprev, &deltaT
    );

    if (moordyn_err != MOORDYN_SUCCESS) {
        FatalErrorInFunction
            << "Error computing MoorDyn step " << tprev
            << "s -> " << tprev + deltaT << "s" << exit(FatalError);
    }    
    
    if (couplingMode_ == word("BODY"))
    {
        // forces/moments are already summarized by moordyn
        forAll(bodyIDs_, b)
        {
            // Get the force and moment over the body
            vector force = vector(Flines[b*6], Flines[b*6+1], Flines[b*6+2]);
            vector moment = vector(Flines[b*6+3], Flines[b*6+4], Flines[b*6+5]);
            // Change the measuring point from the body center to the global center
            moment += model_.X0(bodyID_).r() ^ force;

            fx[bodyIndices_[b]] += spatialVector(moment, force);

            Info<< "X[6dof]: " << vector(X[b*6], X[b*6+1], X[b*6+2]) << ", "
                << vector(X[b*6+3], X[b*6+4], X[b*6+5])
                << endl;
            Info<< " body " << bodies_[b]
                << " force " << force
                << " moment " << moment
                << endl;
        }
    }
    else if (couplingMode_ == word("POINT"))
    {
        // Sum forces and calculate moments for all refAttachmentPts
        for(int pt=0; pt<refAttachmentPt_.size(); pt++)
        {
            vector force = fairForce[pt];
            vector moment =  fairPos[pt] ^ force;

            // Accumulate the force for the restrained body
            fx[bodyIndices_[pt]] += spatialVector(moment, force);

            Info<< " attachmentPt[" << pt << "] " << fairPos[pt]
                << " force " << force
                << " moment " << moment
                << endl;   
        }
    }

    if (writeVTK_ && iteration_ == outerCorrector_)
    {
        if (t >= vtkStartTime_ && time.outputTime())
        {
            Info<< "Write mooring VTK ..." << endl;
            writeVTK(time);
        }
    }

}

bool Foam::RBD::restraints::moorDynR2::read
(
    const dictionary& dict
)
{
    Info << "***  Foam::RBD::restraints::moorDynR2::read" << endl;
    restraint::read(dict);

    coeffs_.readEntry("inputFile", fname_);
    couplingMode_ = coeffs_.getOrDefault<word>("couplingMode", "POINT");

    if (couplingMode_ == word("POINT"))
    {
        coeffs_.readEntry("refAttachmentPt", refAttachmentPt_);

        label nAttachments = refAttachmentPt_.size();

        // Initialise the sizes of bodyIDs, and bodyIndices
        bodyIDs_ = List<label>(nAttachments, bodyID_);
        bodyIndices_ = List<label>(nAttachments, bodyIndex_);
        
        // If the moorings are attached to different bodies
        if (coeffs_.found("bodies") )
        {
            coeffs_.lookup("bodies") >> bodies_;
            // size of bodies_ = nAttachments_
        
            for(int ii=0; ii<nAttachments; ii++)
            {
                bodyIDs_[ii] = model_.bodyID(bodies_[ii]);
                bodyIndices_[ii] = model_.master(bodyIDs_[ii]);
            }
        }
    }
    else if (couplingMode_ == word("BODY"))
    {
        // default to one body only, specified by keyword 'body'
        bodyIDs_ = List<label>(1, bodyID_);
        bodyIndices_ = List<label>(1, bodyIndex_);

        if (coeffs_.found("bodies"))
        {
            coeffs_.lookup("bodies") >> bodies_;

            /*
            {
                // remove duplicate body names
                List<word> tmpBodies;
                forAll(bodies_, b)
                {
                    if (!tmpBodies.found(bodies_[b]))
                    {
                        tmpBodies.append(bodies_[b]);
                    }
                }
                bodies_.transfer(tmpBodies);

                bodyIDs_.setSize(bodies_.size());
                bodyIndices_.setSize(bodies_.size());
            }
            */

            for(int ii=0; ii<bodies_.size(); ii++)
            {
                bodyIDs_[ii] = model_.bodyID(bodies_[ii]);
                bodyIndices_[ii] = model_.master(bodyIDs_[ii]);
            }
        }
    }

    writeVTK_ =
        coeffs_.getOrDefault<Switch>
        (
            "writeMooringVTK",
            coeffs_.getOrDefault<Switch>("writeVTK", false)
        );

    if (writeVTK_)
    {
#ifdef MOORDYN2_HAVE_VTK
        const bool default_legacy_vtk = false;
#else
        const bool default_legacy_vtk = true;
#endif
        vtkPrefix_ = coeffs_.getOrDefault<word>("vtkPrefix", this->name());
        vtkStartTime_ = coeffs_.getOrDefault<scalar>("vtkStartTime", 0);
        legacyVTK_ = coeffs_.getOrDefault<Switch>("vtkLegacyFormat",
                                                  default_legacy_vtk);
        outerCorrector_ = coeffs_.getOrDefault<scalar>("outerCorrector", 1);
    }

    return true;
}


void Foam::RBD::restraints::moorDynR2::write
(
    Ostream& os
) const
{
    Info << "***  Foam::RBD::restraints::moorDynR2::write" << endl;
    restraint::write(os);

    os.writeEntry("inputFile", fname_);
    os.writeEntry("couplingMode", couplingMode_);
    if (couplingMode_ == word("POINT"))
        os.writeEntry("refAttachmentPt", refAttachmentPt_);
    if (int(bodies_.size())>0)
        os.writeEntry("bodies",bodies_);
    
    os.writeEntry("writeVTK", writeVTK_);
    if (writeVTK_)
    {
        os.writeEntry("vtkPrefix", vtkPrefix_);
        os.writeEntry("vtkStartTime", vtkStartTime_);
        os.writeEntry("vtkLegacyFormat", legacyVTK_);
        os.writeEntry("outerCorrector", outerCorrector_);
    }
}


void Foam::RBD::restraints::moorDynR2::writeVTK(const Time& time) const
{
    fileName name(vtkPrefix_ + word::printf("_%04d", vtkCounter_++));

    if (!legacyVTK_) {
        name += ".vtm";
        int moordyn_err = MoorDyn_SaveVTK(moordyn_,
                                          ("Mooring/VTK/" + name).c_str());
        if (moordyn_err != MOORDYN_SUCCESS) {
            FatalError << "Error saving the VTK file \""
                       << name << " for time " << time.timeName() << " s"
                       << exit(FatalError);
        }
        // Update the vtp file
        if (!has_pvd()) {
            make_pvd();
        }
        auto lines = read_pvd(time);
        OFstream os(name_pvd());
        for (auto line : lines) {
            os << line.c_str() << nl;
        }
        os << "    <DataSet timestep=\"" << time.timeName()
           << "\" group=\"\" part=\"0\" "
           << "file=\"" << name.c_str() << "\"/>" << nl
           << "  </Collection>" << nl
           << "</VTKFile>" << nl;

        return;
    }

    name += ".vtk";
    OFstream mps("Mooring/VTK/" + name);
    mps.precision(4);

    unsigned int nLines, nSeg;
    MoorDyn_GetNumberLines(moordyn_, &nLines);

    labelList nodesPerLine(nLines, -1);
    for(int i=0; i<int(nLines); i++)
    {
        MoorDynLine line = MoorDyn_GetLine(moordyn_, i+1);
        MoorDyn_GetLineN(line, &nSeg);
        nodesPerLine[i] = nSeg+1;
    }

    double coord[max(nodesPerLine)][3];

    // Writing header
    mps << "# vtk DataFile Version 3.0" << nl
        << "MoorDyn v2 vtk output time=" << time.timeName()
        << nl << "ASCII" << nl << "DATASET POLYDATA" << endl;
 
    // Writing points
    mps << "\nPOINTS " << sum(nodesPerLine) << " float" << endl;

    for(int i=0; i<int(nLines); i++)
    {   
        //map_.getNodeCoordinates(i, nodesPerLine_[i], &coord[0][0]);
        MoorDynLine line = MoorDyn_GetLine(moordyn_, i+1);
        
        for(int p=0; p<nodesPerLine[i]; p++)
        {
            MoorDyn_GetLineNodePos(line, p, &coord[p][0]);
            
            mps << coord[p][0] << " " << coord[p][1] << " " << coord[p][2] << endl;
        }
    }
    
    // Writing lines
    mps << "\nLINES " << nLines << " " << sum(nodesPerLine+1) << endl;

    label start_node(0);
    for(int i=0; i<int(nLines); i++)
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
