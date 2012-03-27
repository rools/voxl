#ifndef VOXL_H
#define VOXL_H

#include <stdio.h>
#include <stdint.h>
#include <xmmintrin.h>

#define MODEL_LIMIT 16

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923132169163975144
#endif

/**
 * Error codes
 */
typedef enum vx_error {
	VX_ERROR_OK                      = 0, // No error
	VX_ERROR_INVALID_DETAIL_LEVEL    = 1, // Unvalid detail level specifier
	VX_ERROR_TEXTURE_NOT_FOUND       = 2, // Referenced texture not found
	VX_ERROR_INVALID_OCTREE          = 3, // The octree was invalid
	VX_ERROR_CANT_OPEN_FILE          = 4, // The specified file can't be opened
	VX_ERROR_POLYGON_FILE_ERROR      = 5, // The polygon file isn't valid
	VX_ERROR_INVALID_MODEL           = 6, // The specified model is invalid
	VX_ERROR_INVALID_IMAGE           = 7, // The specified image is invalid
	VX_ERROR_UNSUPPORTED_IMAGE       = 8  // The specified image is unsupported
} vx_error;

/**
 * An octree
 */
struct vx_octree {
	struct vx_node *root;
	uint32_t size;
	uint32_t allocated_size;

	// Pointers to loaded models in the octree.
	uint32_t models[MODEL_LIMIT];

	int model_count;
};

/**
 * Camera
 */
struct vx_camera {
	float position[3];
	float direction[3];
	float rotation_vector[3];
	float rotation;

	float tan_fov;

	int resolution[2];
};

/**
 * A render context associated with a specific octree and camera.
 */

// Initialize an empty octree.
void vx_init_octree(struct vx_octree *octree);

// Generate a frame and save it to buffer.
void vx_render_frame(struct vx_octree *octree, struct vx_camera *camera, char *buffer);

// Rasterize the polygon model with the specified level of detail and save it to octree.
vx_error vx_poly_to_octree(const char *file_name, struct vx_octree *octree, int detail);

// Write the octree to the specified file.
vx_error vx_write_octree(FILE *file, struct vx_octree octree);

// Load an octree as a model to another octree and return model id.
int vx_load_model(struct vx_octree *octree, struct vx_octree *model);

// Place a loaded model in an octree.
vx_error vx_place_model(struct vx_octree *octree, int model, float x, float y, float z, int detail);

#endif
