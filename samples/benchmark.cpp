#include <cstdio>
#include <cmath>
#include <cstring>
#include <sys/time.h>
#include <GLFW/glfw3.h>
#include <climits>
#include <cstdlib>

#include "voxl.h"

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

const float MOVEMENT_SPEED = 0.01f;

const int FRAME_COUNT = 100;

double benchmark(GLFWwindow *window, struct vx_octree *octree, int draw);

void print_help(FILE *stream, const char *name) {
	fprintf(stream, "usage: %s [--draw] obj_file\n", name);
}

double get_time() {
	struct timeval t;
	gettimeofday(&t, NULL);
	return t.tv_sec + 1.0e-6 * t.tv_usec;
}

void glfw_error_callback(int error, const char *description) {
	fputs(description, stderr);
}

// A simple benchmark routine to make quick performance tests.
double benchmark(GLFWwindow *window, struct vx_octree *octree, int draw) {
	char *buffer = (char *) malloc(WINDOW_WIDTH * WINDOW_HEIGHT * sizeof(uint32_t));
	struct vx_camera camera;
	double start_time, end_time;
	GLuint texture;

	camera.resolution[0] = WINDOW_WIDTH;
	camera.resolution[1] = WINDOW_HEIGHT;
	camera.position[1] = 0.51f;
	camera.direction[0] = 1.0f;
	camera.direction[1] = 0.0f;
	camera.direction[2] = 0.0f;
	camera.rotation_vector[0] = 0.0f;
	camera.rotation_vector[1] = 1.0f;
	camera.rotation_vector[2] = 0.0f;
	camera.rotation = 0.0f;
	camera.tan_fov = tanf(M_PI * 0.25);

	if (draw) {
		glGenTextures(1, &texture);
		glBindTexture(GL_TEXTURE_2D, texture);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	}

	start_time = get_time();

	for (int i = 0; i < FRAME_COUNT; ++i) {
		float progress = (float)i / FRAME_COUNT;

		// Rotate the camera around the center, always facing the center.
		camera.position[0] = 0.7f * cos(progress * M_PI * 2) + 0.5f;
		camera.position[2] = 0.7f * sin(progress * M_PI * 2) + 0.5f;
		camera.rotation = - progress * M_PI * 2 - 0.5 * M_PI;

		vx_render_frame(octree, &camera, buffer);

		if (draw) {
			glClear(GL_COLOR_BUFFER_BIT);

			glLoadIdentity();

			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, WINDOW_WIDTH, WINDOW_HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			glColor3f(1, 1, 1);
			glBegin(GL_QUADS);
			glTexCoord2f(0.0, 0.0);
			glVertex2f(-1, -1);
			glTexCoord2f(0.0, 1.0);
			glVertex2f(-1, 1);
			glTexCoord2f(1.0, 1.0);
			glVertex2f(1, 1);
			glTexCoord2f(1.0, 0.0);
			glVertex2f(1, -1);
			glEnd();

			glfwSwapBuffers(window);
			glfwPollEvents();
		}
	}

	end_time = get_time();

	if (draw)
		glDeleteTextures(1, &texture);

	return end_time - start_time;
}

int main (int argc, const char * argv[]) {
	int draw = 0;
	struct vx_octree octree;

	// Read command line options.
	{
		int arg = 1;

		while (arg < (argc - 1)) {
			if (strcmp(argv[arg], "--draw") == 0) {
				draw = 1;
				arg++;
			} else {
				print_help(stderr, argv[0]);
				exit(1);
			}
		}

		if (arg == argc) {
			print_help(stderr, argv[0]);
			exit(1);
		}
	}

	{
		vx_error e = vx_poly_to_octree(argv[argc - 1], &octree, 8);

		if (e == VX_ERROR_CANT_OPEN_FILE) {
			fprintf(stderr, "%s: %s: Could not open file\n", argv[0], argv[argc - 1]);
			exit(1);
		}
	}

	GLFWwindow *window = NULL;

	if (draw) {
		glfwSetErrorCallback(glfw_error_callback);

		if (!glfwInit()) {
			fprintf(stderr, "Could not initialize GLFW\n");
			exit(1);
		}

		window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "voxl benchmark", NULL, NULL);
		if (!window) {
			fprintf(stderr, "Could not open GLFW window\n");
			glfwTerminate();
			exit(1);
		}

		glfwMakeContextCurrent(window);

		int framebuffer_width, framebuffer_height;
		glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
		glViewport(0, 0, framebuffer_width, framebuffer_height);

		glEnable(GL_TEXTURE_2D);

		glMatrixMode(GL_PROJECTION);
		glOrtho(-1.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f);
		glMatrixMode(GL_MODELVIEW);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	}

	{
		double time = benchmark(window, &octree, draw);
		printf("benchmark time: %f s (%f fps)\n", time, FRAME_COUNT / time);
	}

	if (draw) {
		glfwDestroyWindow(window);
		glfwTerminate();
	}

	return 0;
}
