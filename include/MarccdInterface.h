//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################
#ifndef MARCCD_INTERFACE_H
#define MARCCD_INTERFACE_H

#include "lima/HwInterface.h"
#include "MarccdCamera.h"
#include "MarccdDetInfoCtrlObj.h"
#include "MarccdBufferCtrlObj.h"
#include "MarccdSyncCtrlObj.h"
#include "MarccdRoiCtrlObj.h"
#include "MarccdBinCtrlObj.h"

namespace lima
{
    namespace Marccd
    {
        //*******************************************************************
        // * \class Interface
        // * \brief Marccd hardware interface
        //*******************************************************************/
        class Interface : public HwInterface
        {
            DEB_CLASS_NAMESPC(DebModCamera, "Interface", "MarCCD");

        public:
            Interface(Camera& cam);
            virtual ~Interface();

            //- From HwInterface
            virtual void getCapList(CapList&) const;
            virtual void reset(ResetLevel reset_level);
            virtual void prepareAcq();
            virtual void startAcq();
            virtual void stopAcq();
            virtual void getStatus(StatusType& status);
            virtual int getNbHwAcquiredFrames();

            //! get the camera object to access it directly from client
            Camera& getCamera() 
                { return m_cam; }

            //- Specific to MarCCD
            int* getHeader();
            void setTimeout(int TO);
            void setWaitFileOnDiskTime(double value);
            double getWaitFileOnDiskTime(void);
            void setNbRetry(int value);

        private:
            Camera& m_cam;
            CapList m_cap_list;
            DetInfoCtrlObj m_det_info;
            BufferCtrlObj m_buffer;
            SyncCtrlObj m_sync;
            RoiCtrlObj m_roi;
            BinCtrlObj m_bin;
        };

    } // namespace Marccd
} // namespace lima

#endif // MARCCD_INTERFACE_H
