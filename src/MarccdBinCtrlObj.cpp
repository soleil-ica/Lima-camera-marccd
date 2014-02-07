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

#include "MarccdBinCtrlObj.h"
#include "MarccdCamera.h"

using namespace lima;
using namespace lima::Marccd;

#define MINBINX 1
#define MAXBINX 8
#define MINBINY 1
#define MAXBINY 8

/*******************************************************************
 * \brief BinCtrlObj constructor
 *******************************************************************/
BinCtrlObj::BinCtrlObj(Camera& cam) :
  m_cam(cam)
{
    DEB_CONSTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
BinCtrlObj::~BinCtrlObj()
{
    DEB_DESTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BinCtrlObj::setBin(const Bin& aBin)
{
    DEB_MEMBER_FUNCT();
    Bin myBin = aBin;
    checkBin(myBin);
    m_cam.setBinning(myBin);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BinCtrlObj::getBin(Bin &aBin)
{
    DEB_MEMBER_FUNCT();
    m_cam.getBinning(aBin);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BinCtrlObj::checkBin(Bin &aBin)
{
    DEB_MEMBER_FUNCT();
	m_cam.checkBin(aBin);
}
