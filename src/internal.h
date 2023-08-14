#ifndef VOXL_INTERNAL_H
#define VOXL_INTERNAL_H

#include <cstdint>

#include "voxl.h"

// An octree node.
struct vx_node {
    uint32_t child_pointers[8];
    uint8_t valid_masks;
    uint8_t leaf_mask;

    uint16_t padding;
};

// An octree leaf (a voxel).
struct vx_voxel {
    int8_t normals[3];

    uint8_t colors[4];

    // TODO: This padding causes a lot of wasted memory.
    int8_t padding;
    uint32_t padding2[7];
};

struct vx_render_context {
    struct vx_octree *octree;
    struct vx_camera *camera;

    uint32_t *buffer;

    int current_row;
};

struct vx_image {
    int width;
    int height;

    // 8 bit per channel RGBA pixel information.
    unsigned char *data;
};

// Return the largest argument.
float vx_max(float a, float b, float c);

// Return the smallest argument.
float vx_min(float a, float b, float c);

// Render the row in the context. This function is thread safe.
void *vx_render_row(void *render_context);

// Append nodes and/or voxels to the octree and return pointer to it.
uint32_t vx_append_to_octree(struct vx_octree *octree, void *data, int size);

// Returns the specified voxel, and creates it if it doesn't exist.
struct vx_voxel *vx_get_octree_voxel(struct vx_octree *octree, float x, float y, float z, int detail);

// Get the parent of the node or voxel that is or would have been at the specified position.
void vx_get_parent(
        struct vx_octree *octree, struct vx_node **parent, int *index, float x, float y, float z, int detail);

// Load an image. Only supports PNG RGBA images with a bit depth of 8 at the moment.
vx_error vx_load_image(char *file_name, struct vx_image *image);

#endif
