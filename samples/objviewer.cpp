#include <cstdio>
#include <cmath>
#include <cstring>
#include <GLFW/glfw3.h>
#include <climits>

#include "voxl.h"

const float MOVEMENT_SPEED = 0.1f;

const int DEFAULT_WINDOW_WIDTH = 800;
const int DEFAULT_WINDOW_HEIGHT = 600;
const int DEFAULT_DETAIL_LEVEL = 8;
const float DEFAULT_FOV = 90;
const int FORMATION_SIZE = 4;

const int FORMATION_LINE = 1;
const int FORMATION_PLANE = 2;
const int FORMATION_PYRAMID = 3;

void print_help(FILE *stream, const char *name) {
	fprintf(stream, "usage: %s [OPTION] obj_file\n", name);
	fprintf(stream, "  --detail       detail level (default %i)\n", DEFAULT_DETAIL_LEVEL);
	fprintf(stream, "  --width        window width (default %i)\n", DEFAULT_WINDOW_WIDTH);
	fprintf(stream, "  --height       window height (default %i)\n", DEFAULT_WINDOW_HEIGHT);
	fprintf(stream, "  --fov          camera field of view (default %.1f)\n", DEFAULT_FOV);
	fprintf(stream, "  --formation    line, plane, pyramid\n");
	fprintf(stream, "  -h, --help     display this help and exit\n");
}

void glfw_error_callback(int error, const char *description) {
	fputs(description, stderr);
}

int main (int argc, const char * argv[]) {
	int running = GL_TRUE;
	struct vx_camera camera;
	struct vx_octree octree;

	int detail_level = DEFAULT_DETAIL_LEVEL;
	int window_width = DEFAULT_WINDOW_WIDTH;
	int window_height = DEFAULT_WINDOW_HEIGHT;
	float tan_fov = tan((DEFAULT_FOV * 0.5f * M_PI) / 180.0f);
	int formation = 0;

	// Read command line options.
	{
		int arg = 1;

		while (arg < (argc - 1)) {
			if (strcmp(argv[arg], "--detail") == 0) {
				detail_level = atoi(argv[arg + 1]);
				if (detail_level < 1 || detail_level > 20) {
					fprintf(stderr, "Invalid detail level\n");
					exit(1);
				}
				arg += 2;
			} else if (strcmp(argv[arg], "--width") == 0) {
				window_width = atoi(argv[arg + 1]);
				if (window_width < 1) {
					fprintf(stderr, "Invalid window width\n");
					exit(1);
				}
				arg += 2;
			} else if (strcmp(argv[arg], "--height") == 0) {
				window_height = atoi(argv[arg + 1]);
				if (window_height < 1) {
					fprintf(stderr, "Invalid field of view\n");
					exit(1);
				}
				arg += 2;
			} else if (strcmp(argv[arg], "--fov") == 0) {
				float fov = atof(argv[arg + 1]);
				if (fov <= 0 || fov >= 180) {
					fprintf(stderr, "Invalid field of view\n");
					exit(1);
				}
				tan_fov = tan((fov * 0.5f * M_PI) / 180.0f);
				arg += 2;
			} else if (strcmp(argv[arg], "--formation") == 0) {
				++arg;
				if (strcmp(argv[arg], "line") == 0)
					formation = FORMATION_LINE;
				else if (strcmp(argv[arg], "plane") == 0)
					formation = FORMATION_PLANE;
				else if (strcmp(argv[arg], "pyramid") == 0)
					formation = FORMATION_PYRAMID;
				else {
					fprintf(stderr, "Invalid formation\n");
					exit(1);
				}
				++arg;
			} else if (strcmp(argv[arg], "--help") == 0 || strcmp(argv[arg], "-h") == 0) {
				print_help(stdout, argv[0]);
				exit(0);
				++arg;
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

	camera.resolution[0] = window_width;
	camera.resolution[1] = window_height;
	camera.direction[0] = 1.0f;
	camera.direction[1] = 0.0f;
	camera.direction[2] = 0.0f;
	camera.rotation_vector[0] = 0.0f;
	camera.rotation_vector[1] = 1.0f;
	camera.rotation_vector[2] = 0.0f;
	camera.rotation = M_PI * 0.75f;
	camera.tan_fov = tan_fov;

	vx_init_octree(&octree);

	int model;

	{
		struct vx_octree m;
		vx_error e = vx_poly_to_octree(argv[argc - 1], &m, detail_level);

		if (e == VX_ERROR_CANT_OPEN_FILE) {
			fprintf(stderr, "%s: %s: Could not open file\n", argv[0], argv[argc - 1]);
			exit(1);
		}

		model = vx_load_model(&octree, &m);
	}

	float step = 1.0f / (1 << FORMATION_SIZE);

	if (formation == FORMATION_LINE) {
		camera.position[0] = 0.05f;
		camera.position[1] = 0.50f;
		camera.position[2] = 0.6f;

		for (float x = 0.0f; x <= 1.0f; x += step) {
			vx_place_model(&octree, model, x, 0.5f, 0.5f, FORMATION_SIZE);
		}
	} else if (formation == FORMATION_PLANE) {
		camera.position[0] = 0.05f;
		camera.position[1] = 0.55f;
		camera.position[2] = 0.95f;

		for (float x = 0.0f; x <= 1.0f; x += step) {
			for (float z = 0.0f; z <= 1.0f; z += step) {
				vx_place_model(&octree, model, x, 0.5f, z, FORMATION_SIZE);
			}
		}
	} else if (formation == FORMATION_PYRAMID) {
		camera.position[0] = 0.05f;
		camera.position[1] = 0.2f;
		camera.position[2] = 0.95f;

		for (float y = 0.0f; y <= 1.0f; y += step) {
			for (float x = y; x <= (1.0f - y); x += step) {
				for (float z = y; z <= (1.0f - y); z += step) {
					vx_place_model(&octree, model, x, y, z, FORMATION_SIZE);
				}
			}
		}
	} else {
		camera.position[0] = 0.40f;
		camera.position[1] = 0.48f;
		camera.position[2] = 0.55f;

		vx_place_model(&octree, model, 0.5f, 0.5f, 0.5f, FORMATION_SIZE);
	}

	glfwSetErrorCallback(glfw_error_callback);

	if (!glfwInit()) {
		fprintf(stderr, "Could not initialize GLFW\n");
		exit(1);
	}

	GLFWwindow *window = glfwCreateWindow(window_width, window_height, "voxl obj viewer", NULL, NULL);
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

	GLuint texture;
	char *buffer = (char *) malloc(window_width * window_height * sizeof(uint32_t));

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	double lastTime = glfwGetTime();

	float totFramerate = 0;
	int frameCount = 0;

	while (running) {
		glClear(GL_COLOR_BUFFER_BIT);

		glLoadIdentity();

		vx_render_frame(&octree, &camera, buffer);

		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, window_width, window_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
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

		double time = glfwGetTime();
		float dtime = time - lastTime;
		float framerate = 1 / dtime;
		if (frameCount % 30 == 0)
			printf("%f fps\n", framerate);
		lastTime = time;

		totFramerate += framerate;
		++frameCount;

		running = !glfwGetKey(window, GLFW_KEY_ESCAPE) && !glfwWindowShouldClose(window);

		if (glfwGetKey(window, 'W')) {
			camera.position[0] += dtime * MOVEMENT_SPEED * sinf(camera.rotation );
			camera.position[2] += dtime * MOVEMENT_SPEED * cosf(camera.rotation );	
		} else if (glfwGetKey(window, 'S')) {
			camera.position[0] -= dtime * MOVEMENT_SPEED * sinf(camera.rotation );
			camera.position[2] -= dtime * MOVEMENT_SPEED * cosf(camera.rotation );
		}

		if (glfwGetKey(window, 'A')) {
			camera.position[0] -= dtime * MOVEMENT_SPEED * sinf(camera.rotation  + M_PI_2);
			camera.position[2] -= dtime * MOVEMENT_SPEED * cosf(camera.rotation  + M_PI_2);
		} else if (glfwGetKey(window, 'D')) {
			camera.position[0] += dtime * MOVEMENT_SPEED * sinf(camera.rotation  + M_PI_2);
			camera.position[2] += dtime * MOVEMENT_SPEED * cosf(camera.rotation  + M_PI_2);
		}

		if (camera.position[0] < 0.0f)
			camera.position[0] = 0.0f;
		else if (camera.position[0] > 1.0f)
			camera.position[0] = 1.0f;

		if (camera.position[2] < 0.0f)
			camera.position[2] = 0.0f;
		else if (camera.position[2] > 1.0f)
			camera.position[2] = 1.0f;

		if (glfwGetKey(window, GLFW_KEY_SPACE)) {
			camera.position[1] += dtime * MOVEMENT_SPEED;
		} else if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL)) {
			camera.position[1] -= dtime * MOVEMENT_SPEED;
		}

		if (camera.position[1] < 0.0f)
			camera.position[1] = 0.0f;
		else if (camera.position[1] > 1.0f)
			camera.position[1] = 1.0f;

		double mouse_x, mouse_y;
		glfwGetCursorPos(window, &mouse_x, &mouse_y);

		camera.rotation  = mouse_x / 90.0f;
	}

	printf("avgerage framerate: %f\n", totFramerate / frameCount);

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}
