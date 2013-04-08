/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2005 Blender Foundation.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): Daniel Dunbar
 *                 Ton Roosendaal,
 *                 Ben Batt,
 *                 Brecht Van Lommel,
 *                 Campbell Barton,
 * 				   Micah Fitch :)
 * 
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/modifiers/intern/MOD_none.c
 *  \ingroup modifiers
 */

#include <stdio.h>

#include "DNA_object_types.h"

#include "BLI_utildefines.h"
#include "BLI_math.h"

#include "BKE_modifier.h"
#include "BKE_cdderivedmesh.h"

#include "MOD_modifiertypes.h"
#include "MOD_util.h"


#include "depsgraph_private.h"


static void calc_moebius_transform(Object *control, Object *target, float co[3]) {
	float mat[4][4];	
	float off[3];
	float radius;
	float leftQ[4]; 
	float rightQ[4];
	float hRot[4][4];
	float leftMat[4][4];
	float rightMat[4][4];
	float dist;
	float h[4];
	radius = 1.0f;
	
	sub_v3_v3v3(off,target->obmat[3],control->obmat[3]); //should be control.pos - target.pos
	mat4_to_quat(leftQ,control->obmat);
	mat4_to_quat(rightQ,control->obmat);
	
	leftMat[0][0] = leftMat[1][1] = leftMat[2][2] = leftMat[3][3] = leftQ[0];
	leftMat[0][1] = leftMat[2][3] = -leftQ[1];
	leftMat[0][2] = leftMat[3][1] = -leftQ[2];
	leftMat[0][3] = leftMat[1][2] = -leftQ[3];
	leftMat[2][1] = leftMat[3][0] = leftQ[3];
	leftMat[1][3] = leftMat[2][0] = leftQ[2];
	leftMat[1][0] = leftMat[3][2] = leftQ[1];
	
	rightMat[0][0] = rightMat[1][1] = leftMat[2][2] = leftMat[3][3] = rightQ[0];
	rightMat[0][1] = rightMat[3][2] = -rightQ[1];
	rightMat[0][2] = rightMat[1][3] = -rightQ[2];
	rightMat[0][3] = rightMat[2][1] = -rightQ[3];
	rightMat[1][2] = rightMat[3][0] = rightQ[3];
	rightMat[3][1] = rightMat[2][0] = rightQ[2];
	rightMat[1][0] = rightMat[2][3] = rightQ[1];
	
	/* quick notes from GLSL implementation
	 * mat4(a,-b,-c,-d///b,a,-d,c;;;c,d,a,-b///d,-c,b,a)
	 * mat4(aa,-bb,-cc,-dd///bb,aa,dd,-cc///cc,-dd,aa,bb///dd,cc,-bb,aa);*/
        
	mult_m4_m4m4(hRot,rightMat,leftMat);
	add_v3_v3(co,off);
	mul_v3_fl(co,1.0f/radius);
	dist= co[0]*co[0]+co[1]*co[1]+co[2]*co[2];
	h[0] = 2*co[0];
	h[1] = 2*co[1];
	h[2] = 2*co[2];
	h[3] = dist-1.0f;
	
	mul_v4_fl(h,1.0f/(1.0f+dist));
	mul_m4_v4(hRot,h);
	co[0] = h[0];
	co[1] = h[1];
	co[2] = h[2];
	mul_v3_fl(co,radius/(1.0f-h[3]));
	sub_v3_v3(co,off);	
}

static void moebius_transform_verts(Object *control, Object *target, DerivedMesh *dm,
                          float (*vertexCos)[3], int numVerts)
{
	int a;
	for (a = 0; a < numVerts; a++) {
		calc_moebius_transform(control, target, vertexCos[a]);
	}
}


static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *md)
{
	MoebiusModifierData *smd = (MoebiusModifierData *)md;
	CustomDataMask dataMask = 0;

	/* ask for vertexgroups if we need them 
	if (smd->vgroup_name[0])
		dataMask |= CD_MASK_MDEFORMVERT;*/

	return dataMask;
}

static void deformVerts(ModifierData *md, Object *ob,
                        DerivedMesh *derivedData,
                        float (*vertexCos)[3],
                        int numVerts,
                        ModifierApplyFlag UNUSED(flag))
{
	MoebiusModifierData *mmd = (MoebiusModifierData *) md;
	DerivedMesh *dm = derivedData;
	CustomDataMask dataMask = requiredDataMask(ob, md);

	/* we implement requiredDataMask but thats not really useful since
	 * mesh_calc_modifiers pass a NULL derivedData */
	if (dataMask)
		dm = get_dm(ob, NULL, dm, NULL, 0);
		
	moebius_transform_verts(mmd->control, ob, derivedData, vertexCos, numVerts/*, mmd->group*/);
	
	if (dm != derivedData)
		dm->release(dm);
}

static void deformVertsEM(
        ModifierData *md, Object *ob, struct BMEditMesh *em,
        DerivedMesh *derivedData, float (*vertexCos)[3], int numVerts)
{
	DerivedMesh *dm = derivedData;

	if (!derivedData) dm = CDDM_from_editbmesh(em, FALSE, FALSE);

	deformVerts(md, ob, dm, vertexCos, numVerts, 0);

	if (!derivedData) dm->release(dm);
}


/* Moebius Transform */
static void initData(ModifierData *md)
{
	MoebiusModifierData *smd = (MoebiusModifierData *) md;
}

static void copyData(ModifierData *md, ModifierData *target)
{
	MoebiusModifierData *smd  = (MoebiusModifierData *)md;
	MoebiusModifierData *tsmd = (MoebiusModifierData *)target;

	tsmd->control  = smd->control;
}

static void foreachObjectLink(ModifierData *md, Object *ob,
                              void (*walk)(void *userData, Object *ob, Object **obpoin), void *userData)
{
	MoebiusModifierData *mmd  = (MoebiusModifierData *)md;
	walk(userData, ob, &mmd->control);
}

static int isDisabled(ModifierData *md, int UNUSED(userRenderParams))
{
	MoebiusModifierData *mmd = (MoebiusModifierData *) md;

	return !mmd->control;
}

static void updateDepgraph(ModifierData *md, DagForest *forest,
                           struct Scene *UNUSED(scene),
                           Object *UNUSED(ob),
                           DagNode *obNode)
{
	MoebiusModifierData *smd  = (MoebiusModifierData *)md;

	if (smd->control)
		dag_add_relation(forest, dag_get_node(forest, smd->control), obNode, DAG_RL_OB_DATA, "Moebius Transformation Modifier");
}

ModifierTypeInfo modifierType_Moebius = {
	/* name */              "Moebius",
	/* structName */        "MoebiusModifierData",
	/* structSize */        sizeof(MoebiusModifierData),
	/* type */              eModifierTypeType_OnlyDeform,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
	                        eModifierTypeFlag_AcceptsCVs |
							eModifierTypeFlag_SupportsEditmode,

	/* copyData */          copyData,
	/* deformVerts */       deformVerts,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     deformVertsEM,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     NULL,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  requiredDataMask,
	/* freeData */          NULL,
	/* isDisabled */        isDisabled,
	/* updateDepgraph */    updateDepgraph,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */	NULL,
	/* foreachObjectLink */ foreachObjectLink,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};
