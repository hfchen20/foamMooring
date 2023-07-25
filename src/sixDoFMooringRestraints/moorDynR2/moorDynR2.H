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
    Foam::sixDoFRigidBodyMotionRestraints::moorDynR2

Description
    sixDoFRigidBodyMotionRestraints model.  Mooring dynamics resolved.

    Calling MoorDyn: a lumped-mass based mooring dynamics model by
    M. Hall and A. Goupee, “Validation of a lumped-mass mooring line model with
    DeepCwind semisubmersible model test data,” Ocean Engineering, vol. 104,
    pp. 590–603, Aug. 2015.

    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with mooring dynamics: Coupling 
    MoorDyn with OpenFOAM. Applied Ocean Research, 124, 103210.
    https://doi.org/10.1016/j.apor.2022.103210

SourceFiles
    moorDynR2.C

\*---------------------------------------------------------------------------*/

#ifndef moorDynR2_H
#define moorDynR2_H

#include "sixDoFRigidBodyMotionRestraint.H"
#include "MoorDyn2.h"
#include "IFstream.H"
#include "OFstream.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{

namespace sixDoFRigidBodyMotionRestraints
{

/*---------------------------------------------------------------------------*\
                          Class moorDynR2 Declaration
\*---------------------------------------------------------------------------*/

class moorDynR2
:
    public sixDoFRigidBodyMotionRestraint
{
    // Private data

    //- Initialize MoorDyn at the first time step ()
    mutable bool initialized_;

    //- MoorDyn intput file
    fileName inputFile_;

    //- Write mooring line coordinates to VTK files
    bool writeVTK_;

public:

    //- Runtime type information
    TypeName("moorDynR2");


    // Constructors

        //- Construct from components
        moorDynR2
        (
            const word& name,
            const dictionary& sDoFRBMRDict
        );

        //- Construct and return a clone
        virtual autoPtr<sixDoFRigidBodyMotionRestraint> clone() const
        {
            return autoPtr<sixDoFRigidBodyMotionRestraint>
            (
                new moorDynR2(*this)
            );
        }


    //- Destructor
    virtual ~moorDynR2();


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
        void writeVTK(const scalar& t) const;

protected:
    //- Check if a PVD file exists
    inline bool has_pvd() const
    {
        IFstream is_file("moorDynR2.pvd");
        return is_file.good();
    }

    //- Create an empty PVD file
    inline void make_pvd() const
    {
        OFstream os("moorDynR2.pvd");
        os << "<?xml version=\"1.0\"?>" << nl
           << "<VTKFile type=\"Collection\" version=\"0.1\"" << nl
           << "         byte_order=\"LittleEndian\"" << nl
           << "         compressor=\"vtkZLibDataCompressor\">" << nl
           << "  <Collection>" << nl
           << "  </Collection>" << nl
           << "</VTKFile>" << nl;
    }

    //- Read the PVD up to the part where new data whall be appended
    inline std::vector<string> read_pvd() const
    {
        IFstream is("moorDynR2.pvd");
        if (!is.good()) {
            FatalError << "Error updating the PVD file: "
                << "'moorDynR2.pvd' cannot be read"
                << exit(FatalError);        
        }
        std::vector<string> lines;
        while (true) {
            string line;
            is.getLine(line);
            if (line.ends_with("</Collection>")) {
                return lines;
            }
            lines.push_back(line);
        }
        FatalError << "Error updating the PVD file: "
            << "'moorDynR2.pvd' is ill-formatted"
            << exit(FatalError);        
    }

private:

    // The MoorDyn system instance
    MoorDyn moordyn_;
    // The coupled body to be controlled by this MoorDyn instance
    MoorDynBody moordyn_body_;
    // A backup of the Mooring system to can restore in between implicit time
    // steps
    typedef struct _moordyn_backup {
        double t;
        uint64_t *data;
    } moordyn_backup;
    mutable moordyn_backup moordyn_backup_;

    //- Create a backup of the mooring system
    inline void save_mooring(const double t) const
    {
        int err;
        size_t data_size;
        err = MoorDyn_Serialize(moordyn_, &data_size, NULL);
        if (err != MOORDYN_SUCCESS) {
            FatalError << "MoorDyn system size cannot be retrieved"
                       << exit(FatalError);
        }
        if (moordyn_backup_.data)
            free(moordyn_backup_.data);
        moordyn_backup_.data = (uint64_t*)malloc(data_size);
        if (!moordyn_backup_.data) {
            FatalError << "Failure allocating " << data_size << " bytes"
                       << exit(FatalError);
        }
        err = MoorDyn_Serialize(moordyn_, NULL, moordyn_backup_.data);
        if (err != MOORDYN_SUCCESS) {
            FatalError << "MoorDyn system cannot be saved"
                       << exit(FatalError);
        }
        moordyn_backup_.t = t;
    }

    //- Restore the backup of the mooring system
    inline void load_mooring() const
    {
        if (!moordyn_backup_.data) {
            FatalError << "No available backup"
                       << exit(FatalError);
        }
        const int err = MoorDyn_Deserialize(moordyn_, moordyn_backup_.data);
        if (err != MOORDYN_SUCCESS) {
            FatalError << "MoorDyn system cannot be restored"
                       << exit(FatalError);
        }
    }
};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace solidBodyMotionFunctions
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //
