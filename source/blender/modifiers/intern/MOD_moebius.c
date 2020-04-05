#include "BKE_modifier.h"
#include "DNA_mesh_types.h"

static Mesh *moebius_applyModifier(struct ModifierData *md,
                                   const struct ModifierEvalContext *ctx,
                                   struct Mesh *mesh)
{
  return mesh;
}

ModifierTypeInfo modifierType_Moebius = {
    /* name */ "Moebius Transformation",
    /* structName */ "MoebiusModifierData",
    /* structSize */ sizeof(MoebiusModifierData),
    /* type */ eModifierTypeType_DeformOrConstruct,
    /* flags */ 0
      | eModifierTypeFlag_AcceptsMesh
      | eModifierTypeFlag_SupportsEditmode
      | eModifierTypeFlag_AcceptsCVs,
    /* copyData */ NULL,

    /* deformVerts */ NULL,
    /* deformMatrices */ NULL,
    /* deformVertsEM */ NULL,
    /* deformMatricesEM */ NULL,
    /* applyModifier */ moebius_applyModifier,

    /* initData */ NULL,
    /* requiredDataMask */ NULL,
    /* freeData */ NULL,
    /* isDisabled */ NULL,
    /* updateDepsgraph */ NULL,
    /* dependsOnTime */ NULL,
    /* dependsOnNormals */ NULL,
    /* foreachObjectLink */ NULL,
    /* foreachIDLink */ NULL,
    /* foreachTexLink */ NULL,
    /* freeRuntimeData */ NULL,
};
