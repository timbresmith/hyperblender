/**
 * $Id$
 *
 * ***** BEGIN GPL/BL DUAL LICENSE BLOCK *****
 *
 * The contents of this file may be used under the terms of either the GNU
 * General Public License Version 2 or later (the "GPL", see
 * http://www.gnu.org/licenses/gpl.html ), or the Blender License 1.0 or
 * later (the "BL", see http://www.blender.org/BL/ ) which has to be
 * bought from the Blender Foundation to become active, in which case the
 * above mentioned GPL option does not apply.
 *
 * The Original Code is Copyright (C) 2002 by NaN Holding BV.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL/BL DUAL LICENSE BLOCK *****
 */
#ifndef __KX_PROPERTYEVENTMANAGER
#define __KX_PROPERTYEVENTMANAGER

#include "SCA_EventManager.h"

#include <vector>
using namespace std;


class SCA_PropertyEventManager : public SCA_EventManager
{
	class SCA_LogicManager*	m_logicmgr;

public:
	SCA_PropertyEventManager(class SCA_LogicManager* logicmgr);
	virtual ~SCA_PropertyEventManager();
	virtual void NextFrame(double curtime,double deltatime);
	virtual void	RegisterSensor(SCA_ISensor* sensor);
	//SCA_LogicManager* GetLogicManager() { return m_logicmgr;}
};
#endif //__KX_PROPERTYEVENTMANAGER
