#ifndef MOD_SKELETAL_ANIMATEVERTICES_VSX_H
#define MOD_SKELETAL_ANIMATEVERTICES_VSX_H

#include "quakedef.h"

#ifdef VSX_PRESENT
void Mod_Skeletal_AnimateVertices_VSX(const model_t * RESTRICT model, const frameblend_t * RESTRICT frameblend, const skeleton_t *skeleton, float * RESTRICT vertex3f, float * RESTRICT normal3f, float * RESTRICT svector3f, float * RESTRICT tvector3f);
#endif

#endif
