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
 * Scenegraph controller for ipos.
 */

#ifdef WIN32
// This warning tells us about truncation of __long__ stl-generated names.
// It can occasionally cause DevStudio to have internal compiler warnings.
#pragma warning( disable : 4786 )     
#endif

#include "KX_IPO_SGController.h"
#include "KX_ScalarInterpolator.h"
#include "KX_GameObject.h"

// All objects should start on frame 1! Will we ever need an object to 
// start on another frame, the 1.0 should change.
KX_IpoSGController::KX_IpoSGController() 
: m_ipotime(1.0),
  m_modify_position(false),
  m_modify_orientation(false),
  m_modify_scaling(false),
  m_modified(true),
  m_ipo_as_force(false),
  m_force_ipo_acts_local(false)
{
	m_sumo_object = NULL;
	m_game_object = NULL;

}


void KX_IpoSGController::SetOption(
	int option,
	int value)
{
	switch (option) {
	case SG_CONTR_IPO_IPO_AS_FORCE:
		m_ipo_as_force = (value != 0);
		m_modified = true;
		break;
	case SG_CONTR_IPO_FORCES_ACT_LOCAL:
		m_force_ipo_acts_local = (value != 0);
		m_modified = true;
		break;
	default:
		; /* just ignore the rest */
	}
}

	void 
KX_IpoSGController::UpdateSumoReference(
	)
{
	if (m_game_object) {
		m_sumo_object = 0;//m_game_object->GetSumoObject();
	}
}

	void 
KX_IpoSGController::SetGameObject(
	KX_GameObject* go
	)
{
	m_game_object = go;
}



bool KX_IpoSGController::Update(double currentTime)
{
	if (m_modified)
	{
		T_InterpolatorList::iterator i;
		for (i = m_interpolators.begin(); !(i == m_interpolators.end()); ++i) {
			(*i)->Execute(m_ipotime);//currentTime);
		}
		
		SG_Spatial* ob = (SG_Spatial*)m_pObject;
		
		if (m_modify_position) {
			if (m_ipo_as_force) {
				/*
				UpdateSumoReference();
				if (m_sumo_object && ob) {
					m_sumo_object->applyCenterForce(m_force_ipo_acts_local ?
						ob->GetWorldOrientation() * m_ipo_xform.GetPosition() :
						m_ipo_xform.GetPosition());
					m_sumo_object->calcXform();
				}
				*/

			} else {
				ob->SetLocalPosition(m_ipo_xform.GetPosition());
			}
		}
		if (m_modify_orientation) {
			if (m_ipo_as_force) {
				/*
				UpdateSumoReference();
				if (m_sumo_object && ob) {
					m_sumo_object->applyTorque(m_force_ipo_acts_local ?
						ob->GetWorldOrientation() * m_ipo_xform.GetEulerAngles() :
						m_ipo_xform.GetEulerAngles());
					m_sumo_object->calcXform();
				}
				*/

			} else {
				ob->SetLocalOrientation(MT_Matrix3x3(m_ipo_xform.GetEulerAngles()));
			}
		}
		if (m_modify_scaling)
			ob->SetLocalScale(m_ipo_xform.GetScaling());

		m_modified=false;
	}
	return false;
}


void KX_IpoSGController::AddInterpolator(KX_IInterpolator* interp)
{
	this->m_interpolators.push_back(interp);
}

SG_Controller*	KX_IpoSGController::GetReplica(class SG_Node* destnode)
{
	KX_IpoSGController* iporeplica = new KX_IpoSGController(*this);
	// clear object that ipo acts on in the replica.
	iporeplica->ClearObject();

	// dirty hack, ask Gino for a better solution in the ipo implementation
	// hacken en zagen, in what we call datahiding, not written for replication :(

	T_InterpolatorList oldlist = m_interpolators;
	iporeplica->m_interpolators.clear();

	T_InterpolatorList::iterator i;
	for (i = oldlist.begin(); !(i == oldlist.end()); ++i) {
		KX_ScalarInterpolator* copyipo = new KX_ScalarInterpolator(*((KX_ScalarInterpolator*)*i));
		iporeplica->AddInterpolator(copyipo);

		MT_Scalar* scaal = ((KX_ScalarInterpolator*)*i)->GetTarget();
		int orgbase = (int)&m_ipo_xform;
		int orgloc = (int)scaal;
		int offset = orgloc-orgbase;
		int newaddrbase = (int)&iporeplica->m_ipo_xform;
		newaddrbase += offset;
		MT_Scalar* blaptr = (MT_Scalar*) newaddrbase;
		copyipo->SetNewTarget((MT_Scalar*)blaptr);
	}
	
	return iporeplica;
}

KX_IpoSGController::~KX_IpoSGController()
{

	T_InterpolatorList::iterator i;
	for (i = m_interpolators.begin(); !(i == m_interpolators.end()); ++i) {
		delete (*i);
	}
	
}
