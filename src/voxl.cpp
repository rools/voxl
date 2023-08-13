#define _USE_MATH_DEFINES

#include <cstring>
#include <cmath>
#include <pthread.h>
#include <cstdlib>

#include "internal.h"
#include "voxl.h"

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
	octree->root = (vx_node *) malloc(octree->allocated_size * sizeof(struct vx_node));
	octree->size = 1;
	octree->model_count = 0;
	octree->root->leaf_mask = 0;
	octree->root->valid_masks = 0;
}

uint32_t vx_ray_cast(struct vx_octree *octree, __m128 ray_origin, __m128 ray_direction) {
	__m128 box = _mm_set_ps1(0.0f);
	__m128 ray_sign = _mm_cmpnlt_ps(ray_direction, _mm_setzero_ps());
	__m128 half_size = _mm_set_ps1(0.5f);
	__m128 moctant = _mm_cmpnle_ps(ray_origin, _mm_add_ps(box, half_size));

	// The potential X, Y and Z-planes that the ray will collide with next.
	__m128 planes = _mm_add_ps(_mm_add_ps(_mm_and_ps(moctant, half_size), _mm_and_ps(ray_sign, half_size)), box);

	// Stacks to keep track of nodes during traversal of the octree.
	struct vx_node *node_stack[MAX_DETAIL_LEVEL];
	int octant_stack[MAX_DETAIL_LEVEL];
	int stack_pos = 0;

	node_stack[0] = octree->root;

	// What octant are we in?
	octant_stack[0] = _mm_movemask_ps(moctant);

	__m128 mask[3];
	{
		unsigned int max = 0xFFFFFFFF;
		__m128 mmax = _mm_load_ss((float *)&max);
		mask[0] = _mm_shuffle_ps(mmax, mmax, _MM_SHUFFLE(3, 3, 3, 0));
		mask[1] = _mm_shuffle_ps(mmax, mmax, _MM_SHUFFLE(3, 3, 0, 3));
		mask[2] = _mm_shuffle_ps(mmax, mmax, _MM_SHUFFLE(3, 0, 3, 3));
	}

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

			box = _mm_add_ps(box, _mm_and_ps(moctant, half_size));

			half_size = _mm_mul_ps(half_size, _mm_set_ps1(0.5f));

			++stack_pos;

			moctant = _mm_cmpnle_ps(ray_origin, _mm_add_ps(box, half_size));
			octant_stack[stack_pos] = _mm_movemask_ps(moctant);

			planes = _mm_add_ps(_mm_add_ps(_mm_and_ps(moctant, half_size), _mm_and_ps(ray_sign, half_size)), box);
			
			continue;
		}

		// Calculate intersection t values.
		float t[4];
		_mm_store_ps(t, _mm_div_ps(_mm_sub_ps(planes, ray_origin), ray_direction));

		// What plane will the ray intersect with first?
		const int intersection_plane = t[0] < t[1] ? (t[0] < t[2] ? 0 : 2) : (t[1] < t[2] ? 1 : 2);
		int dir_sign = (_mm_movemask_ps(ray_sign) >> intersection_plane) & 1;

		ray_origin = _mm_add_ps(ray_origin, _mm_mul_ps(_mm_set_ps1(t[intersection_plane]), ray_direction));

		// Check if the ray left the node.
		if ((octant_stack[stack_pos] & (1 << intersection_plane)) >> intersection_plane == dir_sign) {
			// Pop all nodes that the ray left.
			do {
				--stack_pos;

				half_size = _mm_mul_ps(half_size, _mm_set_ps1(2.0f));

				unsigned int nm[4] = {octant_stack[stack_pos] & 1 ? 0xFFFFFFFF : 0, octant_stack[stack_pos] & 2 ? 0xFFFFFFFF : 0, octant_stack[stack_pos] & 4 ? 0xFFFFFFFF : 0, 0};
				moctant = _mm_load_ps((float *)nm);

				box = _mm_sub_ps(box, _mm_and_ps(moctant, half_size));
			} while (stack_pos >= 0 && (((octant_stack[stack_pos] >> intersection_plane) & 1) == dir_sign));

			// Update octant.
			octant_stack[stack_pos] &= ~(1 << intersection_plane);
			octant_stack[stack_pos] |= dir_sign << intersection_plane;

			unsigned int nm[4] = {octant_stack[stack_pos] & 1 ? 0xFFFFFFFF : 0, octant_stack[stack_pos] & 2 ? 0xFFFFFFFF : 0, octant_stack[stack_pos] & 4 ? 0xFFFFFFFF : 0, 0};
			moctant = _mm_load_ps((float *)nm);

			planes = _mm_add_ps(_mm_add_ps(_mm_and_ps(moctant, half_size), _mm_and_ps(ray_sign, half_size)), box);

			continue;
		}

		// Update octant.
		octant_stack[stack_pos] &= ~(1 << intersection_plane);
		octant_stack[stack_pos] |= dir_sign << intersection_plane;

		unsigned int nm[4] = {octant_stack[stack_pos] & 1 ? 0xFFFFFFFF : 0, octant_stack[stack_pos] & 2 ? 0xFFFFFFFF : 0, octant_stack[stack_pos] & 4 ? 0xFFFFFFFF : 0, 0};
		moctant = _mm_load_ps((float *)nm);

		// Move the plane that the ray hit.
		planes = dir_sign ? _mm_add_ps(planes, _mm_and_ps(mask[intersection_plane], half_size)) : _mm_sub_ps(planes, _mm_and_ps(mask[intersection_plane], half_size));
	}

	// The ray didn't hit any voxel.
	return 0xFF000000;
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

			uint32_t color = vx_ray_cast(rc->octree, _mm_load_ps(ray_origin), _mm_load_ps(ray_direction));

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
		octree->root = (vx_node *) realloc(octree->root, new_allocated_size * sizeof(struct vx_node));
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
