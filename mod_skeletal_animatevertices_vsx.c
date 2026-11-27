#include "mod_skeletal_animatevertices_vsx.h"

#ifdef VSX_PRESENT

#ifdef MATRIX4x4_OPENGLORIENTATION
#error "VSX skeletal requires D3D matrix layout"
#endif

#include <altivec.h>
// altivec.h turns these into keywords
#undef vector
#undef pixel
#undef bool

typedef __vector float vfloat4;

// On little endian the natural order vector loads and stores of POWER8 need an extra instruction each to swap the
// two halves of the vector, which costs about a quarter of the vertex transform time, so this loads and stores as
// two doublewords instead (a plain lxvd2x/stxvd2x) and keeps all vectors swapped: float i in memory is element i^2
typedef unsigned long long __attribute__((__may_alias__)) vfloat4mem_t;
#define LOAD(p) ((vfloat4)vec_xl_be(0, (const vfloat4mem_t *)(p)))
#define STORE(v, p) vec_xst_be((__vector unsigned long long)(v), 0, (vfloat4mem_t *)(p))
#define SPLAT(v, i) vec_splat((v), (i) ^ 2)
#define SET(f0, f1, f2, f3) ((vfloat4){(f2), (f3), (f0), (f1)})

// Note that all the math here is done as separate multiplies and adds in the same order as the generic codepath,
// so the results are identical to it

// stores the bone matrix b0-b3 and its relative deformation matrix (bone matrix * base pose inverse n)
static inline void Mod_Skeletal_StoreBone_VSX(vfloat4 b0, vfloat4 b1, vfloat4 b2, vfloat4 b3, const float * RESTRICT n, matrix4x4_t * RESTRICT b, matrix4x4_t * RESTRICT r)
{
	vfloat4 r0, r1, r2, r3, nr;
	STORE(b0, b->m[0]);
	STORE(b1, b->m[1]);
	STORE(b2, b->m[2]);
	STORE(b3, b->m[3]);
	nr = LOAD(n);
	r0 = vec_mul(b0, SPLAT(nr, 0));
	r1 = vec_mul(b0, SPLAT(nr, 1));
	r2 = vec_mul(b0, SPLAT(nr, 2));
	r3 = vec_mul(b0, SPLAT(nr, 3));
	nr = LOAD(n+4);
	r0 = vec_add(r0, vec_mul(b1, SPLAT(nr, 0)));
	r1 = vec_add(r1, vec_mul(b1, SPLAT(nr, 1)));
	r2 = vec_add(r2, vec_mul(b1, SPLAT(nr, 2)));
	r3 = vec_add(r3, vec_mul(b1, SPLAT(nr, 3)));
	nr = LOAD(n+8);
	r0 = vec_add(r0, vec_mul(b2, SPLAT(nr, 0)));
	r1 = vec_add(r1, vec_mul(b2, SPLAT(nr, 1)));
	r2 = vec_add(r2, vec_mul(b2, SPLAT(nr, 2)));
	r3 = vec_add(r3, vec_mul(b2, SPLAT(nr, 3)));
	r3 = vec_add(r3, b3);
	STORE(r0, r->m[0]);
	STORE(r1, r->m[1]);
	STORE(r2, r->m[2]);
	STORE(r3, r->m[3]);
}

void Mod_Skeletal_AnimateVertices_VSX(const model_t * RESTRICT model, const frameblend_t * RESTRICT frameblend, const skeleton_t *skeleton, float * RESTRICT vertex3f, float * RESTRICT normal3f, float * RESTRICT svector3f, float * RESTRICT tvector3f)
{
	// vertex weighted skeletal
	int i, k;
	int blends;
	matrix4x4_t *bonepose;
	matrix4x4_t *boneposerelative;
	const blendweights_t * RESTRICT weights;
	int num_vertices = model->surfmesh.num_vertices;
	int num_vertices_batched = num_vertices & ~3;
	// these pack the xyz of four transformed vertices (four vectors with an unused fourth float) into three vectors
	const __vector unsigned char pack0 = { 0,  1,  2,  3, 24, 25, 26, 27,  8,  9, 10, 11, 12, 13, 14, 15};
	const __vector unsigned char pack1 = {24, 25, 26, 27, 28, 29, 30, 31, 12, 13, 14, 15,  0,  1,  2,  3};
	const __vector unsigned char pack2 = {28, 29, 30, 31, 16, 17, 18, 19,  0,  1,  2,  3, 24, 25, 26, 27};

	bonepose = (matrix4x4_t *) Mod_Skeletal_AnimateVertices_AllocBuffers(sizeof(matrix4x4_t) * (model->num_bones*2 + model->surfmesh.num_blends));
	boneposerelative = bonepose + model->num_bones;

	if (skeleton && !skeleton->relativetransforms)
		skeleton = NULL;

	// interpolate matrices
	// Note that the matrices are 4x4 and transposed compared to the generic codepath, each vector is one column
	if (skeleton)
	{
		for (i = 0;i < model->num_bones;i++)
		{
			const matrix4x4_t * RESTRICT s = &skeleton->relativetransforms[i];
			vfloat4 b0, b1, b2, b3;
			vfloat4 s0 = LOAD(s->m[0]), s1 = LOAD(s->m[1]), s2 = LOAD(s->m[2]);
			if (model->data_bones[i].parent >= 0)
			{
				const matrix4x4_t * RESTRICT p = &bonepose[model->data_bones[i].parent];
				vfloat4 pr = LOAD(p->m[0]);
				b0 = vec_mul(pr, SPLAT(s0, 0));
				b1 = vec_mul(pr, SPLAT(s0, 1));
				b2 = vec_mul(pr, SPLAT(s0, 2));
				b3 = vec_mul(pr, SPLAT(s0, 3));
				pr = LOAD(p->m[1]);
				b0 = vec_add(b0, vec_mul(pr, SPLAT(s1, 0)));
				b1 = vec_add(b1, vec_mul(pr, SPLAT(s1, 1)));
				b2 = vec_add(b2, vec_mul(pr, SPLAT(s1, 2)));
				b3 = vec_add(b3, vec_mul(pr, SPLAT(s1, 3)));
				pr = LOAD(p->m[2]);
				b0 = vec_add(b0, vec_mul(pr, SPLAT(s2, 0)));
				b1 = vec_add(b1, vec_mul(pr, SPLAT(s2, 1)));
				b2 = vec_add(b2, vec_mul(pr, SPLAT(s2, 2)));
				b3 = vec_add(b3, vec_mul(pr, SPLAT(s2, 3)));
				b3 = vec_add(b3, LOAD(p->m[3]));
			}
			else
			{
				// transpose
				b0 = SET(s->m[0][0], s->m[1][0], s->m[2][0], s->m[3][0]);
				b1 = SET(s->m[0][1], s->m[1][1], s->m[2][1], s->m[3][1]);
				b2 = SET(s->m[0][2], s->m[1][2], s->m[2][2], s->m[3][2]);
				b3 = SET(s->m[0][3], s->m[1][3], s->m[2][3], s->m[3][3]);
			}
			Mod_Skeletal_StoreBone_VSX(b0, b1, b2, b3, model->data_baseboneposeinverse + i * 12, &bonepose[i], &boneposerelative[i]);
		}
	}
	else
	{
		for (i = 0;i < model->num_bones;i++)
		{
			// blend by transform each quaternion/translation into a dual-quaternion first, then blending
			float m[12];
			vfloat4 b0, b1, b2, b3;
			const short * RESTRICT firstpose7s = model->data_poses7s + 7 * (frameblend[0].subframe * model->num_bones + i);
			float firstlerp = frameblend[0].lerp,
				firsttx = firstpose7s[0], firstty = firstpose7s[1], firsttz = firstpose7s[2],
				rx = firstpose7s[3] * firstlerp,
				ry = firstpose7s[4] * firstlerp,
				rz = firstpose7s[5] * firstlerp,
				rw = firstpose7s[6] * firstlerp,
				dx = firsttx*rw + firstty*rz - firsttz*ry,
				dy = -firsttx*rz + firstty*rw + firsttz*rx,
				dz = firsttx*ry - firstty*rx + firsttz*rw,
				dw = -firsttx*rx - firstty*ry - firsttz*rz,
				scale, sx, sy, sz, sw;
			for (blends = 1;blends < MAX_FRAMEBLENDS && frameblend[blends].lerp > 0;blends++)
			{
				const short * RESTRICT blendpose7s = model->data_poses7s + 7 * (frameblend[blends].subframe * model->num_bones + i);
				float blendlerp = frameblend[blends].lerp,
					blendtx = blendpose7s[0], blendty = blendpose7s[1], blendtz = blendpose7s[2],
					qx = blendpose7s[3], qy = blendpose7s[4], qz = blendpose7s[5], qw = blendpose7s[6];
				if(rx*qx + ry*qy + rz*qz + rw*qw < 0) blendlerp = -blendlerp;
				qx *= blendlerp;
				qy *= blendlerp;
				qz *= blendlerp;
				qw *= blendlerp;
				rx += qx;
				ry += qy;
				rz += qz;
				rw += qw;
				dx += blendtx*qw + blendty*qz - blendtz*qy;
				dy += -blendtx*qz + blendty*qw + blendtz*qx;
				dz += blendtx*qy - blendty*qx + blendtz*qw;
				dw += -blendtx*qx - blendty*qy - blendtz*qz;
			}
			// generate a matrix from the dual-quaternion, implicitly normalizing it in the process
			scale = 1.0f / (rx*rx + ry*ry + rz*rz + rw*rw);
			sx = rx * scale;
			sy = ry * scale;
			sz = rz * scale;
			sw = rw * scale;
			m[0] = sw*rw + sx*rx - sy*ry - sz*rz;
			m[1] = 2*(sx*ry - sw*rz);
			m[2] = 2*(sx*rz + sw*ry);
			m[3] = model->num_posescale*(dx*sw - dy*sz + dz*sy - dw*sx);
			m[4] = 2*(sx*ry + sw*rz);
			m[5] = sw*rw + sy*ry - sx*rx - sz*rz;
			m[6] = 2*(sy*rz - sw*rx);
			m[7] = model->num_posescale*(dx*sz + dy*sw - dz*sx - dw*sy);
			m[8] = 2*(sx*rz - sw*ry);
			m[9] = 2*(sy*rz + sw*rx);
			m[10] = sw*rw + sz*rz - sx*rx - sy*ry;
			m[11] = model->num_posescale*(dy*sx + dz*sw - dx*sy - dw*sz);
			if (i == r_skeletal_debugbone.integer)
				m[r_skeletal_debugbonecomponent.integer % 12] += r_skeletal_debugbonevalue.value;
			m[3] *= r_skeletal_debugtranslatex.value;
			m[7] *= r_skeletal_debugtranslatey.value;
			m[11] *= r_skeletal_debugtranslatez.value;
			if (model->data_bones[i].parent >= 0)
			{
				const matrix4x4_t * RESTRICT p = &bonepose[model->data_bones[i].parent];
				vfloat4 pr = LOAD(p->m[0]);
				b0 = vec_mul(pr, vec_splats(m[0]));
				b1 = vec_mul(pr, vec_splats(m[1]));
				b2 = vec_mul(pr, vec_splats(m[2]));
				b3 = vec_mul(pr, vec_splats(m[3]));
				pr = LOAD(p->m[1]);
				b0 = vec_add(b0, vec_mul(pr, vec_splats(m[4])));
				b1 = vec_add(b1, vec_mul(pr, vec_splats(m[5])));
				b2 = vec_add(b2, vec_mul(pr, vec_splats(m[6])));
				b3 = vec_add(b3, vec_mul(pr, vec_splats(m[7])));
				pr = LOAD(p->m[2]);
				b0 = vec_add(b0, vec_mul(pr, vec_splats(m[8])));
				b1 = vec_add(b1, vec_mul(pr, vec_splats(m[9])));
				b2 = vec_add(b2, vec_mul(pr, vec_splats(m[10])));
				b3 = vec_add(b3, vec_mul(pr, vec_splats(m[11])));
				b3 = vec_add(b3, LOAD(p->m[3]));
			}
			else
			{
				b0 = SET(m[0], m[4], m[8], 0.0f);
				b1 = SET(m[1], m[5], m[9], 0.0f);
				b2 = SET(m[2], m[6], m[10], 0.0f);
				b3 = SET(m[3], m[7], m[11], 1.0f);
			}
			Mod_Skeletal_StoreBone_VSX(b0, b1, b2, b3, model->data_baseboneposeinverse + i * 12, &bonepose[i], &boneposerelative[i]);
		}
	}

	// generate matrices for all blend combinations
	weights = model->surfmesh.data_blendweights;
	for (i = 0;i < model->surfmesh.num_blends;i++, weights++)
	{
		const matrix4x4_t * RESTRICT m = &boneposerelative[weights->index[0]];
		vfloat4 fv = vec_splats(weights->influence[0] * (1.0f / 255.0f));
		vfloat4 b0 = vec_mul(LOAD(m->m[0]), fv);
		vfloat4 b1 = vec_mul(LOAD(m->m[1]), fv);
		vfloat4 b2 = vec_mul(LOAD(m->m[2]), fv);
		vfloat4 b3 = vec_mul(LOAD(m->m[3]), fv);
		for (k = 1;k < 4 && weights->influence[k];k++)
		{
			m = &boneposerelative[weights->index[k]];
			fv = vec_splats(weights->influence[k] * (1.0f / 255.0f));
			b0 = vec_add(b0, vec_mul(LOAD(m->m[0]), fv));
			b1 = vec_add(b1, vec_mul(LOAD(m->m[1]), fv));
			b2 = vec_add(b2, vec_mul(LOAD(m->m[2]), fv));
			b3 = vec_add(b3, vec_mul(LOAD(m->m[3]), fv));
		}
		m = &boneposerelative[model->num_bones + i];
		STORE(b0, m->m[0]);
		STORE(b1, m->m[1]);
		STORE(b2, m->m[2]);
		STORE(b3, m->m[3]);
	}

	// The vertices are transformed four at a time: their twelve floats are exactly three vectors, so nothing is
	// read or written past the end of the arrays, and the remaining (up to three) vertices are done with scalars

	// loads the matrices of the next four vertices, M3 (translation) is only needed for positions
#define LOAD_MATRIX(M, index) \
		const matrix4x4_t * RESTRICT M = &boneposerelative[index]; \
		const vfloat4 M##0 = LOAD(M->m[0]), M##1 = LOAD(M->m[1]), M##2 = LOAD(M->m[2])
#define LOAD_MATRICES3() \
		LOAD_MATRIX(ma, b[0]); \
		LOAD_MATRIX(mb, b[1]); \
		LOAD_MATRIX(mc, b[2]); \
		LOAD_MATRIX(md, b[3])
#define LOAD_MATRICES4() \
		LOAD_MATRICES3(); \
		const vfloat4 ma3 = LOAD(ma->m[3]), mb3 = LOAD(mb->m[3]), mc3 = LOAD(mc->m[3]), md3 = LOAD(md->m[3])

#define STORE_PACKED(out, r0, r1, r2, r3) \
		STORE(vec_perm(r0, r1, pack0), (out)); \
		STORE(vec_perm(r1, r2, pack1), (out) + 4); \
		STORE(vec_perm(r2, r3, pack2), (out) + 8)
	// i is the index of a float in the twelve loaded (in0, in1, in2 hold 0-3, 4-7, 8-11)
#define SPLAT_IN(i) SPLAT((i) < 4 ? in0 : (i) < 8 ? in1 : in2, (i) & 3)
	// rotates vertex i of the four loaded by its matrix
#define ROTATE(i, M) \
		vec_add(vec_add(vec_mul(SPLAT_IN((i)*3), M##0), vec_mul(SPLAT_IN((i)*3+1), M##1)), vec_mul(SPLAT_IN((i)*3+2), M##2))
#define TRANSFORM_POSITIONS(in, out) { \
		const vfloat4 in0 = LOAD(in), in1 = LOAD((in) + 4), in2 = LOAD((in) + 8); \
		const vfloat4 r0 = vec_add(ROTATE(0, ma), ma3); \
		const vfloat4 r1 = vec_add(ROTATE(1, mb), mb3); \
		const vfloat4 r2 = vec_add(ROTATE(2, mc), mc3); \
		const vfloat4 r3 = vec_add(ROTATE(3, md), md3); \
		STORE_PACKED(out, r0, r1, r2, r3); \
	}
#define TRANSFORM_VECTORS(in, out) { \
		const vfloat4 in0 = LOAD(in), in1 = LOAD((in) + 4), in2 = LOAD((in) + 8); \
		const vfloat4 r0 = ROTATE(0, ma); \
		const vfloat4 r1 = ROTATE(1, mb); \
		const vfloat4 r2 = ROTATE(2, mc); \
		const vfloat4 r3 = ROTATE(3, md); \
		STORE_PACKED(out, r0, r1, r2, r3); \
	}

#define LOAD_MATRIX_SCALAR() const float * RESTRICT m = &boneposerelative[*b].m[0][0]
#define TRANSFORM_POSITION_SCALAR(in, out) \
	(out)[0] = ((in)[0] * m[0] + (in)[1] * m[4] + (in)[2] * m[ 8] + m[12]); \
	(out)[1] = ((in)[0] * m[1] + (in)[1] * m[5] + (in)[2] * m[ 9] + m[13]); \
	(out)[2] = ((in)[0] * m[2] + (in)[1] * m[6] + (in)[2] * m[10] + m[14]);
#define TRANSFORM_VECTOR_SCALAR(in, out) \
	(out)[0] = ((in)[0] * m[0] + (in)[1] * m[4] + (in)[2] * m[ 8]); \
	(out)[1] = ((in)[0] * m[1] + (in)[1] * m[5] + (in)[2] * m[ 9]); \
	(out)[2] = ((in)[0] * m[2] + (in)[1] * m[6] + (in)[2] * m[10]);

	// transform vertex attributes by blended matrices
	if (vertex3f)
	{
		const float * RESTRICT v = model->surfmesh.data_vertex3f;
		const unsigned short * RESTRICT b = model->surfmesh.blends;
		// special case common combinations of attributes to avoid repeated loading of matrices
		if (normal3f)
		{
			const float * RESTRICT n = model->surfmesh.data_normal3f;
			if (svector3f && tvector3f)
			{
				const float * RESTRICT svec = model->surfmesh.data_svector3f;
				const float * RESTRICT tvec = model->surfmesh.data_tvector3f;

				for (i = 0; i < num_vertices_batched; i += 4, v += 12, n += 12, svec += 12, tvec += 12, b += 4,
						vertex3f += 12, normal3f += 12, svector3f += 12, tvector3f += 12)
				{
					LOAD_MATRICES4();
					TRANSFORM_POSITIONS(v, vertex3f);
					TRANSFORM_VECTORS(n, normal3f);
					TRANSFORM_VECTORS(svec, svector3f);
					TRANSFORM_VECTORS(tvec, tvector3f);
				}
				for (; i < num_vertices; i++, v += 3, n += 3, svec += 3, tvec += 3, b++,
						vertex3f += 3, normal3f += 3, svector3f += 3, tvector3f += 3)
				{
					LOAD_MATRIX_SCALAR();
					TRANSFORM_POSITION_SCALAR(v, vertex3f);
					TRANSFORM_VECTOR_SCALAR(n, normal3f);
					TRANSFORM_VECTOR_SCALAR(svec, svector3f);
					TRANSFORM_VECTOR_SCALAR(tvec, tvector3f);
				}

				return;
			}

			for (i = 0; i < num_vertices_batched; i += 4, v += 12, n += 12, b += 4, vertex3f += 12, normal3f += 12)
			{
				LOAD_MATRICES4();
				TRANSFORM_POSITIONS(v, vertex3f);
				TRANSFORM_VECTORS(n, normal3f);
			}
			for (; i < num_vertices; i++, v += 3, n += 3, b++, vertex3f += 3, normal3f += 3)
			{
				LOAD_MATRIX_SCALAR();
				TRANSFORM_POSITION_SCALAR(v, vertex3f);
				TRANSFORM_VECTOR_SCALAR(n, normal3f);
			}
		}
		else
		{
			for (i = 0; i < num_vertices_batched; i += 4, v += 12, b += 4, vertex3f += 12)
			{
				LOAD_MATRICES4();
				TRANSFORM_POSITIONS(v, vertex3f);
			}
			for (; i < num_vertices; i++, v += 3, b++, vertex3f += 3)
			{
				LOAD_MATRIX_SCALAR();
				TRANSFORM_POSITION_SCALAR(v, vertex3f);
			}
		}
	}

	else if (normal3f)
	{
		const float * RESTRICT n = model->surfmesh.data_normal3f;
		const unsigned short * RESTRICT b = model->surfmesh.blends;
		for (i = 0; i < num_vertices_batched; i += 4, n += 12, b += 4, normal3f += 12)
		{
			LOAD_MATRICES3();
			TRANSFORM_VECTORS(n, normal3f);
		}
		for (; i < num_vertices; i++, n += 3, b++, normal3f += 3)
		{
			LOAD_MATRIX_SCALAR();
			TRANSFORM_VECTOR_SCALAR(n, normal3f);
		}
	}

	if (svector3f)
	{
		const float * RESTRICT svec = model->surfmesh.data_svector3f;
		const unsigned short * RESTRICT b = model->surfmesh.blends;
		for (i = 0; i < num_vertices_batched; i += 4, svec += 12, b += 4, svector3f += 12)
		{
			LOAD_MATRICES3();
			TRANSFORM_VECTORS(svec, svector3f);
		}
		for (; i < num_vertices; i++, svec += 3, b++, svector3f += 3)
		{
			LOAD_MATRIX_SCALAR();
			TRANSFORM_VECTOR_SCALAR(svec, svector3f);
		}
	}

	if (tvector3f)
	{
		const float * RESTRICT tvec = model->surfmesh.data_tvector3f;
		const unsigned short * RESTRICT b = model->surfmesh.blends;
		for (i = 0; i < num_vertices_batched; i += 4, tvec += 12, b += 4, tvector3f += 12)
		{
			LOAD_MATRICES3();
			TRANSFORM_VECTORS(tvec, tvector3f);
		}
		for (; i < num_vertices; i++, tvec += 3, b++, tvector3f += 3)
		{
			LOAD_MATRIX_SCALAR();
			TRANSFORM_VECTOR_SCALAR(tvec, tvector3f);
		}
	}

#undef LOAD_MATRIX
#undef LOAD_MATRICES3
#undef LOAD_MATRICES4
#undef SPLAT_IN
#undef ROTATE
#undef STORE_PACKED
#undef TRANSFORM_POSITIONS
#undef TRANSFORM_VECTORS
#undef LOAD_MATRIX_SCALAR
#undef TRANSFORM_POSITION_SCALAR
#undef TRANSFORM_VECTOR_SCALAR
}

#endif
