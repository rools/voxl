#include <stdio.h>
#include <string.h>
#include <float.h>

#include "internal.h"
#include "voxl.h"
#include "tribox.h"

#include <png.h>

vx_error vx_load_image(char *file_name, struct vx_image *image) {
	png_byte header[8];
	png_structp png_ptr;
	png_infop info_ptr;

	// Open file and check if it is a PNG file.
	FILE *fp = fopen(file_name, "rb");
	if (!fp)
		return VX_ERROR_CANT_OPEN_FILE;
	if (fread(header, 1, 8, fp) != 8)
		return VX_ERROR_INVALID_IMAGE;
	if (png_sig_cmp(header, 0, 8))
		return VX_ERROR_UNSUPPORTED_IMAGE;

	// Initialize PNG structures.
	png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!png_ptr)
		return VX_ERROR_INVALID_IMAGE;
	info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr)
		return VX_ERROR_INVALID_IMAGE;
	if (setjmp(png_jmpbuf(png_ptr)))
		return VX_ERROR_INVALID_IMAGE;
	png_init_io(png_ptr, fp);
	png_set_sig_bytes(png_ptr, 8);
	png_read_info(png_ptr, info_ptr);

	// Only accept RGBA images with bit depth of 8 for simplicity.
	if (png_get_color_type(png_ptr, info_ptr) != PNG_COLOR_TYPE_RGB_ALPHA || png_get_bit_depth(png_ptr, info_ptr) != 8)
		return VX_ERROR_INVALID_IMAGE;

	image->width = png_get_image_width(png_ptr, info_ptr);
	image->height = png_get_image_height(png_ptr, info_ptr);

	if (setjmp(png_jmpbuf(png_ptr)))
		return VX_ERROR_INVALID_IMAGE;

	// Read the image data.
	unsigned char **row_pointers = (unsigned char **)malloc(sizeof(png_bytep) * image->height);
	image->data = malloc(png_get_rowbytes(png_ptr, info_ptr) * image->height);
	for (int y = 0; y < image->height; y++)
		row_pointers[y] = image->data + y * png_get_rowbytes(png_ptr, info_ptr);
	png_read_image(png_ptr, (png_bytepp)row_pointers);

	free(row_pointers);

	fclose(fp);

	return VX_ERROR_OK;
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
