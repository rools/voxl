#define _USE_MATH_DEFINES

#include <string.h>
#include <math.h>
#include <pthread.h>
#include <float.h>

#include <assert.h>

#include "internal.h"
#include "voxl.h"
#include "tribox.h"

// How many threads to use when rendering a frame.
const int RENDERING_THREADS = 8;

// The maximum level of detail level for any octree.
const int MAX_DETAIL_LEVEL = 20;

float vx_max(float a, float b, float c) {
	return a > b ? (a > c ? a : c) : (b > c ? b : c);
}

float vx_min(float a, float b, float c) {
	return a < b ? (a < c ? a : c) : (b < c ? b : c);
}

void vx_init_octree(struct vx_octree *octree) {
	octree->allocated_size = 128;
	octree->root = malloc(octree->allocated_size * sizeof(struct vx_node));
	octree->size = 1;
	octree->model_count = 0;
	octree->root->leaf_mask = 0;
	octree->root->valid_masks = 0;
}

uint32_t vx_ray_cast_iterative(struct vx_octree *octree, float *ray_origin, float *ray_direction) {
	float pos[3] = {ray_origin[0], ray_origin[1], ray_origin[2]};

	float box[3] = {0.0f, 0.0f, 0.0f};
	float planes[3];
	int ray_sign[3] = {ray_direction[0] >= 0.0f, ray_direction[1] >= 0.0f, ray_direction[2] >= 0.0f};
	float half_size = 0.5f;

	// Stacks to keep track of nodes during traversal of the octree.
	struct vx_node *node_stack[MAX_DETAIL_LEVEL];
	int octant_stack[MAX_DETAIL_LEVEL];
	int stack_pos = 0;

	node_stack[0] = octree->root;

	// What octant are we in?
	octant_stack[0] = (box[0] + half_size) < pos[0];
	octant_stack[0] |= ((box[1] + half_size) < pos[1]) << 1;
	octant_stack[0] |= ((box[2] + half_size) < pos[2]) << 2;

	// The potential X, Y and Z-planes that the ray will collide with next.
	planes[0] = box[0] + half_size * ray_sign[0] + half_size * (octant_stack[0] & 1);
	planes[1] = box[1] + half_size * ray_sign[1] + half_size * ((octant_stack[0] >> 1) & 1);
	planes[2] = box[2] + half_size * ray_sign[2] + half_size * ((octant_stack[0] >> 2) & 1);

	while (stack_pos >= 0) {
		// Have we reached a child?
		if (node_stack[stack_pos]->valid_masks & (1 << octant_stack[stack_pos])) {
			// Is this child a voxel?
			if (node_stack[stack_pos]->leaf_mask & (1 << octant_stack[stack_pos])) {
				struct vx_voxel *voxel = (struct vx_voxel *)&octree->root[node_stack[stack_pos]->child_pointers[octant_stack[stack_pos]]];

				// The direction of the directional light source.
				float light[3] = {-0.039030f, -0.866792f, 0.497139f};

				float brightness = acosf((voxel->normals[0] / 128.0f) * light[0] + (voxel->normals[1] / 128.0f) * light[1] + (voxel->normals[2] / 128.0f) * light[2]) / M_PI;

				return ((unsigned int)(voxel->colors[1] * brightness) << 16)
					| ((unsigned int)(voxel->colors[1] * brightness) << 8)
					| (unsigned int)(voxel->colors[0] * brightness);
			}

			// This was a node, push it to the stack.
			node_stack[stack_pos + 1] = &octree->root[node_stack[stack_pos]->child_pointers[octant_stack[stack_pos]]];

			box[0] += (octant_stack[stack_pos] & 1) * half_size;
			box[1] += ((octant_stack[stack_pos] >> 1) & 1) * half_size;
			box[2] += ((octant_stack[stack_pos] >> 2) & 1) * half_size;

			half_size *= 0.5f;

			++stack_pos;

			octant_stack[stack_pos] = (box[0] + half_size) < pos[0];
			octant_stack[stack_pos] |= ((box[1] + half_size) < pos[1]) << 1;
			octant_stack[stack_pos] |= ((box[2] + half_size) < pos[2]) << 2;

			planes[0] = box[0] + half_size * ray_sign[0] + half_size * (octant_stack[stack_pos] & 1);
			planes[1] = box[1] + half_size * ray_sign[1] + half_size * ((octant_stack[stack_pos] >> 1) & 1);
			planes[2] = box[2] + half_size * ray_sign[2] + half_size * ((octant_stack[stack_pos] >> 2) & 1);
			
			continue;
		}

		// Calculate intersection t values.
		float t[3];
		t[0] = (planes[0] - pos[0]) / ray_direction[0];
		t[1] = (planes[1] - pos[1]) / ray_direction[1];
		t[2] = (planes[2] - pos[2]) / ray_direction[2];

		// What plane will the ray intersect with first?
		int intersection_plane = t[0] < t[1] ? (t[0] < t[2] ? 0 : 2) : (t[1] < t[2] ? 1 : 2);

		// Advance the ray within the current box.
		pos[0] += t[intersection_plane] * ray_direction[0];
		pos[1] += t[intersection_plane] * ray_direction[1];
		pos[2] += t[intersection_plane] * ray_direction[2];

		// Check if the ray left the node.
		if (((octant_stack[stack_pos] >> intersection_plane) & 1) == ray_sign[intersection_plane]) {
			// Pop all nodes that the ray left.
			do {
				--stack_pos;
				half_size *= 2.0f;

				box[0] -= (octant_stack[stack_pos] & 1) * half_size;
				box[1] -= ((octant_stack[stack_pos] >> 1) & 1) * half_size;
				box[2] -= ((octant_stack[stack_pos] >> 2) & 1) * half_size;

			} while (stack_pos >= 0 && (((octant_stack[stack_pos] >> intersection_plane) & 1) == ray_sign[intersection_plane]));

			// Update octant.
			octant_stack[stack_pos] &= ~(1 << intersection_plane);
			octant_stack[stack_pos] |= ray_sign[intersection_plane] << intersection_plane;
			
			planes[0] = box[0] + half_size * ray_sign[0] + half_size * (octant_stack[stack_pos] & 1);
			planes[1] = box[1] + half_size * ray_sign[1] + half_size * ((octant_stack[stack_pos] >> 1) & 1);
			planes[2] = box[2] + half_size * ray_sign[2] + half_size * ((octant_stack[stack_pos] >> 2) & 1);

			continue;
		}

		// Update octant.
		octant_stack[stack_pos] &= ~(1 << intersection_plane);
		octant_stack[stack_pos] |= ray_sign[intersection_plane] << intersection_plane;

		// Move the plane that the ray hit.
		planes[intersection_plane] += half_size * (ray_sign[intersection_plane] ? 1 : -1);
	}

	// The ray didn't hit any voxel.
	return 0xFF000000;
}

uint32_t vx_ray_cast_recursive(struct vx_octree *octree, __m128 ray_origin, __m128 ray_direction) {
	__m128 rsign = _mm_cmpnlt_ps(ray_direction, _mm_setzero_ps());	
	return vx_ray_cast_helper(octree, ray_origin, ray_direction, _mm_set_ps1(0.0f), _mm_set_ps1(0.5f), octree->root /*+ 1*/, rsign);
}

uint32_t vx_ray_cast_helper(struct vx_octree *octree, __m128 pos, const __m128 rd, const __m128 box, const __m128 hsize, const struct vx_node *cd, __m128 rsign) {
	__m128 moctant = _mm_cmpnle_ps(pos, _mm_add_ps(box, hsize));

	int octant = _mm_movemask_ps(moctant) & 7;

	__m128 planes = _mm_add_ps(_mm_add_ps(_mm_and_ps(moctant, hsize), _mm_and_ps(rsign, hsize)), box);

	while (1) {
		if (cd->valid_masks & (1 << octant)) {
			// Check if this node is a leaf.
			if (cd->leaf_mask & (1 << octant)) {
				struct vx_voxel *voxel = (struct vx_voxel *)&octree->root[cd->child_pointers[octant]];
				
				float light[3] = {-0.039030f, -0.866792f, 0.497139f};
				
				float brightness = acosf((voxel->normals[0] / 128.0f) * light[0] + (voxel->normals[1] / 128.0f) * light[1] + (voxel->normals[2] / 128.0f) * light[2]) / M_PI;

				return ((unsigned int)(voxel->colors[1] * brightness) << 16)
					| ((unsigned int)(voxel->colors[1] * brightness) << 8)
					| (unsigned int)(voxel->colors[0] * brightness);
			}

			// Not a voxel, recurse
			__m128 nbox = _mm_add_ps(box, _mm_and_ps(moctant, hsize));

			int val = vx_ray_cast_helper(octree, pos, rd, nbox, _mm_mul_ps(hsize, _mm_set_ps1(0.5f)), &octree->root[cd->child_pointers[octant]], rsign);

			if (val != 0)
				return val;
		}

		// Calculate intersection t values.
		float t[4];
		_mm_store_ps(t, _mm_div_ps(_mm_sub_ps(planes, pos), rd));

		// In what direction did the ray hit the box?
		int intersection_plane = (t[0] < t[1] ? (t[0] < t[2] ? 0 : 2) : (t[1] < t[2] ? 1 : 2));

		int dir_sign = (_mm_movemask_ps(rsign) >> intersection_plane) & 1;

		// Optimization, leave this box early if there's no more children in the intersection direction.
		int m[3] = {85, 51, 15}; // Four bits each describing children in positive direction
		if ((cd->valid_masks & (m[intersection_plane] << (dir_sign * (1 << intersection_plane)))) == 0)
			return 0;

		// Advance the ray within the current box.
		pos = _mm_add_ps(pos, _mm_mul_ps(_mm_set_ps1(t[intersection_plane]), rd));

		// The ray left the box.
		if ((octant & (1 << intersection_plane)) >> intersection_plane == dir_sign)
			return 0;

		// Update octant.
		octant &= ~(1 << intersection_plane);
		octant |= dir_sign << intersection_plane;

		unsigned int nm[4] = {octant & 1 ? 0xFFFFFFFF : 0, octant & 2 ? 0xFFFFFFFF : 0, octant & 4 ? 0xFFFFFFFF : 0, 0};
		moctant = _mm_load_ps((float *)nm);

		unsigned int max = 0xFFFFFFFF;
		__m128 mmax = _mm_load_ss((float *)&max);
		
		__m128 msk[3] = {_mm_shuffle_ps(mmax, mmax, _MM_SHUFFLE(3, 3, 3, 0)), _mm_shuffle_ps(mmax, mmax, _MM_SHUFFLE(3, 3, 0, 3)), _mm_shuffle_ps(mmax, mmax, _MM_SHUFFLE(3, 0, 3, 3))};

		// Move the plane that the ray hit.
		planes = dir_sign ? _mm_add_ps(planes, _mm_and_ps(msk[intersection_plane], hsize)) : _mm_sub_ps(planes, _mm_and_ps(msk[intersection_plane], hsize));
	}
}

void vx_render_frame(struct vx_octree *octree, struct vx_camera *camera, char *buffer) {
	pthread_t threads[RENDERING_THREADS];
	struct vx_render_context context;

	context.octree = octree;
	context.camera = camera;
	context.buffer = (uint32_t *)buffer;
	context.current_row = 0;

	// Start rendering threads.
	for (int t = 0; t < RENDERING_THREADS; ++t)
		pthread_create(&threads[t], NULL, vx_render_row, &context);

	// Wait for all threads to complete.
	for (int t = 0; t < RENDERING_THREADS; ++t)
		pthread_join(threads[t], NULL);
}

void *vx_render_row(void *render_context) {
	struct vx_render_context *rc = (struct vx_render_context *)render_context;

	float cosa = cosf(rc->camera->rotation);
	float sina = sinf(rc->camera->rotation);

	float *rv = rc->camera->rotation_vector;

	// x and y factors for ray vector to get correct field of view.
	float x_factor = rc->camera->tan_fov / (rc->camera->resolution[0] / 2);
	float y_factor = (rc->camera->tan_fov / (rc->camera->resolution[1] / 2)) * ((float)rc->camera->resolution[1] / rc->camera->resolution[0]);

	float ray_origin[4];
	ray_origin[0] = rc->camera->position[0]; // - sina * 0.4f;
	ray_origin[1] = rc->camera->position[1];
	ray_origin[2] = rc->camera->position[2]; // - cosa * 0.4f;
	ray_origin[3] = 0;

	for (int32_t y = (rc->current_row)++; y < rc->camera->resolution[1]; y = (rc->current_row)++) {
		for (int x = 0; x < rc->camera->resolution[0]; ++x) {
			float ray_direction[4];

			{
				float rx = ((float)(x - rc->camera->resolution[0] / 2)) * x_factor;
				float ry = ((float)(y - rc->camera->resolution[1] / 2)) * y_factor;
				float rz = 1.0f; // This gives a rectilinear perspective.
				// float rz = 2.0f * cosf(0.5f * sqrtf(rx * rx + ry * ry)); // This gives a fisheye perspective.

				// Rotate the ray around the rotation vector.
				ray_direction[0] = rv[0] * (rv[0] * rx + rv[1] * ry + rv[2] * rz) * (1 - cosa) + rx * cosa + (- rv[2] * ry + rv[1] * rz) * sina;
				ray_direction[1] = rv[1] * (rv[0] * rx + rv[1] * ry + rv[2] * rz) * (1 - cosa) + ry * cosa + (rv[2] * rx + rv[0] * rz) * sina;
				ray_direction[2] = rv[2] * (rv[0] * rx + rv[1] * ry + rv[2] * rz) * (1 - cosa) + rz * cosa + (- rv[1] * rx + rv[0] * ry) * sina;
			}

			//uint32_t color = vx_ray_cast_iterative(rc->octree, ray_origin, ray_direction);
			uint32_t color = vx_ray_cast_recursive(rc->octree, _mm_load_ps(ray_origin), _mm_load_ps(ray_direction));

			rc->buffer[y * rc->camera->resolution[0] + x] = color;
		}
	}

	return NULL;
}

uint32_t vx_append_to_octree(struct vx_octree *octree, void *data, int size) {
	// Check if we need to allocate more memory to fit the new data.
	if ((octree->size + size) > octree->allocated_size) {
		// Allocate at least double the memory. (Is this really desirable when the octree grows very large?)
		uint32_t new_allocated_size = octree->size + size;
		if (2 * octree->allocated_size > new_allocated_size)
			new_allocated_size = 2 * octree->allocated_size;
		octree->root = realloc(octree->root, new_allocated_size * sizeof(struct vx_node));
		octree->allocated_size = new_allocated_size;
	}

	uint32_t pointer = octree->size;

	memcpy(&octree->root[octree->size], data, sizeof(struct vx_node) * size);
	octree->size += size;

	return pointer;
}

void vx_get_parent(struct vx_octree *octree, struct vx_node **parent, int *index, float x, float y, float z, int detail) {
	uint32_t node = 0;
	
	float mid_point[3] = {0.5f, 0.5f, 0.5f};
	float node_size = 0.5f;

	for (int i = detail - 1; i >= 0; --i) {
		int octant = x > mid_point[0] ? 1 : 0;
		octant |= y > mid_point[1] ? 2 : 0;
		octant |= z > mid_point[2] ? 4 : 0;
		
		node_size *= 0.5f;
		
		mid_point[0] += x > mid_point[0] ? node_size : - node_size;
		mid_point[1] += y > mid_point[1] ? node_size : - node_size;
		mid_point[2] += z > mid_point[2] ? node_size : - node_size;
		
		if (i == 0) {
			// We found the parent of the node or voxel we wanted.
			*parent = &octree->root[node];
			*index = octant;
		} else {
			// This level contains a node.
			if (octree->root[node].valid_masks & (1 << octant)) {
				node = octree->root[node].child_pointers[octant];
			} else {
				struct vx_node new_node;
				new_node.leaf_mask = 0;
				new_node.valid_masks = 0;
				
				int pointer = vx_append_to_octree(octree, &new_node, 1);
				octree->root[node].child_pointers[octant] = pointer;
				octree->root[node].valid_masks |= (1 << octant);
				node = pointer;
			}
		}
	}
}

struct vx_voxel *vx_get_octree_voxel(struct vx_octree *octree, float x, float y, float z, int detail) {
	uint32_t node = 0;
	
	float mid_point[3] = {0.5f, 0.5f, 0.5f};
	float node_size = 0.5f;

	struct vx_voxel *voxel = NULL;

	for (int i = detail - 1; i >= 0; --i) {
		int octant = x > mid_point[0] ? 1 : 0;
		octant |= y > mid_point[1] ? 2 : 0;
		octant |= z > mid_point[2] ? 4 : 0;

		node_size *= 0.5f;

		mid_point[0] += x > mid_point[0] ? node_size : - node_size;
		mid_point[1] += y > mid_point[1] ? node_size : - node_size;
		mid_point[2] += z > mid_point[2] ? node_size : - node_size;

		if (i == 0) {
			// The final level contains a voxel.
			if (octree->root[node].valid_masks & (1 << octant) && octree->root[node].leaf_mask & (1 << octant)) {
				voxel = (struct vx_voxel *)&octree->root[octree->root[node].child_pointers[octant]];
			} else {
				struct vx_voxel new_voxel;
				int pointer = vx_append_to_octree(octree, &new_voxel, 1);
				octree->root[node].valid_masks |= (1 << octant);
				octree->root[node].leaf_mask |= (1 << octant);
				octree->root[node].child_pointers[octant] = pointer;
				voxel = (struct vx_voxel *)&octree->root[pointer];
			}
		} else {
			// This level contains a node.
			if (octree->root[node].valid_masks & (1 << octant)) {
				node = octree->root[node].child_pointers[octant];
			} else {
				struct vx_node new_node;
				new_node.leaf_mask = 0;
				new_node.valid_masks = 0;

				int pointer = vx_append_to_octree(octree, &new_node, 1);
				octree->root[node].child_pointers[octant] = pointer;
				octree->root[node].valid_masks |= (1 << octant);
				node = pointer;
			}
		}
	}

	return voxel;
}

vx_error vx_poly_to_octree(const char *file_name, struct vx_octree *octree, int detail) {
	float offset[3];
	float scale;

	int vertex_count = 0;
	int face_count = 0;
	int uv_count = 0;
	int normal_count = 0;

	float **vertices;
	float **uvs;
	int **faces;
	float **normals;

	FILE *file = fopen(file_name, "r");

	if (file == NULL)
		return VX_ERROR_CANT_OPEN_FILE;

	// Start with checking the bounds of the model and size needed for arrays.
	{
		float min[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
		float max[3] = {FLT_MIN, FLT_MIN, FLT_MIN};

		while (!feof(file)) {
			char line[1024];
			fgets(line, 1024, file);

			if (!strncmp(line, "v ", 2)) {
				float x, y, z;

				sscanf(line, "v %f %f %f\n", &x, &y, &z);

				min[0] = x < min[0] ? x : min[0];
				min[1] = y < min[1] ? y : min[1];
				min[2] = z < min[2] ? z : min[2];
				max[0] = x > max[0] ? x : max[0];
				max[1] = y > max[1] ? y : max[1];
				max[2] = z > max[2] ? z : max[2];

				++vertex_count;
			} else if (!strncmp(line, "vt ", 3)) {
				++uv_count;
			} else if (!strncmp(line, "vn ", 3)) {
				++normal_count;
			} else if (!strncmp(line, "f ", 2)) {
				++face_count;
			}
		}

		offset[0] = - min[0];
		offset[1] = - min[1];
		offset[2] = - min[2];

		// Scale factor so that the model is kept within the octree.
		scale = 1.0f / vx_max(max[0] - min[0], max[1] - min[1], max[2] - min[2]);
	}

	// TODO: Free these allocations when done.
	vertices = (float **)malloc(vertex_count * sizeof(float *));
	uvs = (float **)malloc(uv_count * sizeof(float *));
	faces = (int **)malloc(face_count * sizeof(int *));
	normals = (float **)malloc(normal_count * sizeof(float *));

	vertex_count = 0;
	face_count = 0;
	uv_count = 0;
	normal_count = 0;

	rewind(file);

	// Load the actual vertices, texture normals and faces from file.
	while (!feof(file)) {
		char line[1024];
		fgets(line, 1024, file);

		if (!strncmp(line, "v ", 2)) {
			float *vertex = (float *)malloc(3 * sizeof(float));
			sscanf(line, "v %f %f %f\n", vertex, vertex + 1, vertex + 2);

			vertex[0] = (vertex[0] + offset[0]) * scale;
			vertex[1] = (vertex[1] + offset[1]) * scale;
			vertex[2] = (vertex[2] + offset[2]) * scale;

			vertices[vertex_count++] = vertex;
		} else if (!strncmp(line, "vt ", 3)) {
			float *uv = (float *)malloc(2 * sizeof(float));

			sscanf(line, "vt %f %f\n", uv, uv + 1);

			uvs[uv_count++] = uv;
		} else if (!strncmp(line, "vn ", 3)) {
			float *normal = (float *)malloc(3 * sizeof(float));

			sscanf(line, "vn %f %f %f\n", normal, normal + 1, normal + 2);

			normals[normal_count++] = normal;
		} else if (!strncmp(line, "f ", 2)) {
			int *face = (int *)malloc(9 * sizeof(int));

			sscanf(line, "f %i/%i/%i %i/%i/%i %i/%i/%i\n", face, face + 3, face + 6, face + 1, face + 4, face + 7, face + 2, face + 5, face + 8);

			for (int i = 0; i < 9; ++i)
				face[i] -= 1;

			faces[face_count++] = face;
		}
	}

	//if (vertex_count == 0 || face_count == 0 || uv_count == 0 || normal_count == 0)
	//	return VX_ERROR_POLYGON_FILE_ERROR;

	// Build the octree from the faces.
	octree->allocated_size = 4096;
	octree->root = (struct vx_node *)malloc(octree->allocated_size * sizeof(struct vx_node));
	octree->size = 1;

	octree->root->leaf_mask = 0;
	octree->root->valid_masks = 0;

	const float step = 1.0f / (1 << detail);

	// TODO: Optimize rasterization process, this is a pretty slow way of doing it.
	for (int i = 0; i < face_count; ++i) {
		int *face = faces[i];

		float *normal = normals[face[6]];
		float *triangle_vertices[3] = {vertices[face[0]], vertices[face[1]], vertices[face[2]]};

		// The bounding box of the polygon.
		float min[3], max[3];
		min[0] = vx_min(triangle_vertices[0][0], triangle_vertices[1][0], triangle_vertices[2][0]) - step;
		min[1] = vx_min(triangle_vertices[0][1], triangle_vertices[1][1], triangle_vertices[2][1]) - step;
		min[2] = vx_min(triangle_vertices[0][2], triangle_vertices[1][2], triangle_vertices[2][2]) - step;
		max[0] = vx_max(triangle_vertices[0][0], triangle_vertices[1][0], triangle_vertices[2][0]) + step;
		max[1] = vx_max(triangle_vertices[0][1], triangle_vertices[1][1], triangle_vertices[2][1]) + step;
		max[2] = vx_max(triangle_vertices[0][2], triangle_vertices[1][2], triangle_vertices[2][2]) + step;

		for (float x = min[0]; x <= max[0]; x += step) {
			for (float y = min[1]; y <= max[1]; y += step) {
				for (float z = min[2]; z <= max[2]; z += step) {
					float box_center[3] = {x, y, z};
					float box_half_size[3] = {0.5f * step, 0.5f * step, 0.5f * step};

					if (triBoxOverlap(box_center, box_half_size, triangle_vertices)) {
						struct vx_voxel *voxel = vx_get_octree_voxel(octree, x, y, z, detail);

						voxel->colors[0] = 255;
						voxel->colors[1] = 255;
						voxel->colors[2] = 255;

						voxel->normals[0] = normal[0] * 128;
						voxel->normals[1] = normal[1] * 128;
						voxel->normals[2] = normal[2] * 128;
					}
				}
			}
		}
	}

	return VX_ERROR_OK;
}

// TODO: Fix this temporary solution (eg. by switching to relative pointers).
void add_pointer(struct vx_octree *octree, uint32_t node, int amount) {
	for (int i = 0; i < 8; ++i) {
		if (octree->root[node].valid_masks & (1 << i)) {
			octree->root[node].child_pointers[i] += amount;
			if ((octree->root[node].leaf_mask & (1 << i)) == 0)
				add_pointer(octree, octree->root[node].child_pointers[i] - amount, amount);
		}
	}
}

int vx_load_model(struct vx_octree *octree, struct vx_octree *model) {
	if (octree->model_count == MODEL_LIMIT)
		return -1;

	int model_index = octree->model_count;

	// TODO: Fix better solution... This destroys the model octree.
	add_pointer(model, 0, octree->size);

	octree->models[octree->model_count++] = vx_append_to_octree(octree, model->root, model->size);

	return model_index;
}

vx_error vx_place_model(struct vx_octree *octree, int model, float x, float y, float z, int detail) {
	struct vx_node *parent;
	int index;

	if (model < 0 || model >= octree->model_count)
		return VX_ERROR_INVALID_MODEL;

	vx_get_parent(octree, &parent, &index, x, y, z, detail);

	parent->child_pointers[index] = octree->models[model];
	parent->valid_masks |= (1 << index);

	return VX_ERROR_OK;
}
