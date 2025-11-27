#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>

#include <Eigen/Dense>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opencv2/opencv.hpp> 

#include "render.h"
#include "calculations.h"
#include "arm_attributes.h"
#include "get_path.h"

//for math constants
#define _USE_MATH_DEFINES

using namespace std;

//=========== Camera and Model Movement ============
enum class cameraMovements : char {
	CAMERA_UP,
	CAMERA_DOWN,
	CAMERA_LEFT,
	CAMERA_RIGHT,
	CAMERA_FORWARD,
	CAMERA_BACKWARD,
	MODEL_ROTATE_CW,
	MODEL_ROTATE_CCW,
	MODEL_ROTATE_FORWARD,
	MODEL_ROTATE_BACKWARD
};

glm::vec3 move_camera(cameraMovements input) {
	float speed = 0.5;

	switch (input) {
	case cameraMovements::CAMERA_DOWN:
			return glm::vec3(0.0f, -1.0f*speed, 0.0f);

		case cameraMovements::CAMERA_UP:
			return glm::vec3(0.0f, 1.0f*speed, 0.0f);

		case cameraMovements::CAMERA_RIGHT:
			return glm::vec3(1.0f*speed, 0.0f, 0.0f);
		case cameraMovements::CAMERA_LEFT:
			return glm::vec3(-1.0f*speed, 0.0f, 0.0f);

		case cameraMovements::CAMERA_FORWARD:
			return glm::vec3(0.0f, 0.0f, -1.0f*speed);

		case cameraMovements::CAMERA_BACKWARD:
			return glm::vec3(0.0f, 0.01f, 1.0f*speed);
	}
}

pair<float, glm::vec3> rotate_model(cameraMovements rotation) {
	float speed = 0.05;
	switch (rotation) {
	case cameraMovements::MODEL_ROTATE_CW:
		return { speed, {0.0f, 1.0f, 0.0f} }; //about y axis

	case cameraMovements::MODEL_ROTATE_CCW:
		return { -speed, {0.0f, 1.0f, 0.0f} }; //about y axis

	case cameraMovements::MODEL_ROTATE_FORWARD:
		return { speed, {1.0f, 0.0f, 0.0f} }; //about x axis

	case cameraMovements::MODEL_ROTATE_BACKWARD:
		return { -speed, {1.0f, 0.0f, 0.0f} }; //about x axis
	}
	return { 0.0f, { 0.0f, 0.0f, 0.0f } };
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GLFW_TRUE);
	}
}

static void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
}

void render() {
	// GLFW init for the window
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(1600, 1200, "Line Test", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	glfwSetKeyCallback(window, key_callback);

	if (!window) {
		std::cout << "Failed to create GLFW window\n";
		glfwTerminate();
		return;
	}

	//GLAD init
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD\n";
		return;
	}

	// =========== Shaders =========== 
	const char* vertexShaderSource = R"(
		#version 330 core
		layout(location = 0) in vec3 aPos;     // position
		layout(location = 1) in vec3 aColor;   // color

		//camera and model matrices
		uniform mat4 projection;
		uniform mat4 view;
		uniform mat4 model;

		out vec3 vertexColor;

		void main() {
			gl_Position = projection * view*model* vec4(aPos, 1.0);
			vertexColor = aColor; // pass the color to the fragment shader
		}
	)";

	const char* fragmentShaderSource = R"(
		#version 330 core
		out vec4 FragColor;
		in vec3 vertexColor; 

		void main() {
			FragColor = vec4(vertexColor, 1.0); // output the color
		}
		)";

	int success;
	char infoLog[512];

	// Vertex Shader
	unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
	glCompileShader(vertexShader);

	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		std::cout << "Vertex shader error:\n" << infoLog << "\n";
	}

	// Fragment Shader
	unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
	glCompileShader(fragmentShader);

	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::cout << "Fragment shader error:\n" << infoLog << "\n";
	}

	// Shader Program
	unsigned int shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);

	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		std::cout << "Shader linking error:\n" << infoLog << "\n";
	}

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

	// =============== Defining Vertices and Points ========

	// Robot arm
	vector<float> vertices;
	vector<float> points;

	for (int i = 0; i < 15; i++) {
		uniform_real_distribution<float>rand_x(-l2, l2);
		uniform_real_distribution<float>rand_yz(-l1, l3);
		float tar_x = rand_x(gen);
		float tar_y = rand_yz(gen);
		float tar_z = rand_yz(gen);

		tuple<float, float, float> angles = ik(tar_x, tar_y, tar_z);
		fk_test(get<0>(angles), get<1>(angles), get<2>(angles));

		vector<float> temp = simulate_angles(get<0>(angles), get<1>(angles), get<2>(angles));
		vertices.insert(vertices.end(), temp.begin(), temp.end());

		//target
		vector<float>pt_temp = { tar_x, tar_y, tar_z, 1.0f, 0.0f, 0.0f };
		points.insert(points.end(), pt_temp.begin(), pt_temp.end());
	}

	//axis lines
	vector<float> axis = {
		//positions									//colors
		// X axis - red
		20.0f, 0.0f,0.0f,			1.0f,0.0f,0.0f,  
		-20.0f,0.0f,0.0f,			1.0f,0.0f,0.0f, 

		// Y axis - blue
		0.0f, 20.0f,0.0f, 			0.0f,0.0f,1.0f, 
		0.0f,-20.0f,0.0f,			0.0f,0.0f,1.0f, 

		// Z axis - green
		0.0f,0.0f,20.0f,				0.0f,1.0f,0.0f,  
		0.0f,0.0f,-20.0f,			0.0f,1.0f,0.0f,  
	};	

	// lines VAO/VBO 
	GLuint VAO_lines, VBO_lines;
	glGenVertexArrays(1, &VAO_lines);
	glGenBuffers(1, &VBO_lines);
	glBindVertexArray(VAO_lines);
	glBindBuffer(GL_ARRAY_BUFFER, VBO_lines);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);

	//positions
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	//positions
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3*sizeof(float)));
	glEnableVertexAttribArray(1);

	// ============= Axis VAO/VBO  =============
	GLuint VAO_axis, VBO_axis;
	glGenVertexArrays(1, &VAO_axis);
	glGenBuffers(1, &VBO_axis);
	glBindVertexArray(VAO_axis);
	glBindBuffer(GL_ARRAY_BUFFER, VBO_axis);

	//positions
	glBufferData(GL_ARRAY_BUFFER, axis.size() * sizeof(float), axis.data(), GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	//color
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3*sizeof(float)));
	glEnableVertexAttribArray(1);

	// ==================== points VAO/VBO =============
	GLuint VAO_points, VBO_points;
	glGenVertexArrays(1, &VAO_points);
	glGenBuffers(1, &VBO_points);
	glBindVertexArray(VAO_points);
	glBindBuffer(GL_ARRAY_BUFFER, VBO_points);
	glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(float), points.data(), GL_DYNAMIC_DRAW);

	//positions
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	//color
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);


	// ================ Set camera view =================

	// Initial camera position 	
	glm::vec3 init_camPos = { 0.0f, 0.0f, 40.0f };
	glm::vec3 cameraFront = glm::vec3(0, 0, -1);

	// Initial view and position
	glm::vec3 cameraPos = init_camPos;
	glm::vec3 cameraTarget = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::mat4 view = glm::lookAt(cameraPos, cameraTarget, cameraUp);

	// Projection and Model matrices
	glm::mat4 model = glm::mat4(1.0f); // identity
	glm::mat4 projection = glm::perspective(glm::radians(45.0f), 1600.0f / 1200.0f, 0.1f, 100.0f);


	// Setting initial conditions
	glm::vec3 curr_camPos = init_camPos;

	int line_count = 0;
	int points_count = 0;
	float curr_time = 0, last_time=0, delay =0.6f;

	glEnable(GL_DEPTH_TEST);
	// ======== Render Loop ========
	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();
		curr_time = glfwGetTime();

		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::vec3 camOffsets{ 0.0f, 0.0f, 0.0f };
		float modelAngleoffsets = 0.0f;
		glm::vec3 modelRotAxis = { 0.0f, 1.0f, 0.0f };		//y by default

		// Rotate camera around the arm
		if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
			camOffsets = move_camera(cameraMovements::CAMERA_UP);
			//cout << "moving FORWARD" << endl;
		}
		if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
			camOffsets = move_camera(cameraMovements::CAMERA_DOWN);
			//cout << "moving BACKWARD" << endl;
		}
		if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
			camOffsets = move_camera(cameraMovements::CAMERA_LEFT);
			//cout << "moving LEFT" << endl;
		}
		if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
			camOffsets = move_camera(cameraMovements::CAMERA_RIGHT);
			//cout << "moving RIGHT" << endl;
		}
		if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
			camOffsets = move_camera(cameraMovements::CAMERA_FORWARD);
			//cout << "moving FORWARD" << endl;
		}
		if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
			camOffsets = move_camera(cameraMovements::CAMERA_BACKWARD);
			//cout << "moving BACKWARD\n";
		}
		if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
			modelAngleoffsets = rotate_model(cameraMovements::MODEL_ROTATE_CW).first;
			modelRotAxis = rotate_model(cameraMovements::MODEL_ROTATE_CW).second;
			//cout << "rotating CW\n";
		}

		if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
			modelAngleoffsets = rotate_model(cameraMovements::MODEL_ROTATE_CCW).first;
			modelRotAxis = rotate_model(cameraMovements::MODEL_ROTATE_CCW).second;
			//cout << "rotating CCW\n";
		}
		if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
			modelAngleoffsets = rotate_model(cameraMovements::MODEL_ROTATE_FORWARD).first;
			modelRotAxis = rotate_model(cameraMovements::MODEL_ROTATE_FORWARD).second;
			//cout << "rotating FORWARD\n";
		}

		if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
			modelAngleoffsets = rotate_model(cameraMovements::MODEL_ROTATE_BACKWARD).first;
			modelRotAxis = rotate_model(cameraMovements::MODEL_ROTATE_BACKWARD).second;
			//cout << "rotating BACKWARDS\n";
		}

		cameraPos = curr_camPos + camOffsets;

		// Reset position
		if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
			cameraTarget = { 0.0f, 0.0f, 0.0f };
			cameraPos = init_camPos;
			curr_camPos = init_camPos;
			model = glm::translate(model, { 0.0f,0.0f, 0.0f }); //reset model position
			cout << "fixing to origin\n";
		}

		
		glm::vec3 cameraTarget = cameraPos + cameraFront;
		glm::mat4 view = glm::lookAt(cameraPos, cameraTarget, cameraUp);
		curr_camPos += camOffsets;

		model = glm::rotate(model, modelAngleoffsets, modelRotAxis); // rotate the model without moving camera
			
		
		//Reset position
		if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
			cameraPos = init_camPos;
			cout << "fixing to origin...";
		}
		// Reset rotation
		if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {
			model = glm::mat4(1.0f);
			cout << "fixing viewport...";
		}

		// Upload vertices to GPU
		glBindBuffer(GL_ARRAY_BUFFER, VBO_lines);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);

		// Use shader
		glUseProgram(shaderProgram);
		glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
		glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

		//define line width and point size
		glLineWidth(10.0f);
		glPointSize(15.0f);

		glLineWidth(5.0f);
		glBindVertexArray(VAO_axis);
		glDrawArrays(GL_LINES, 0, axis.size() / 6);

		// Draw the next line
		glBindVertexArray(VAO_lines);
		glDrawArrays(GL_LINES, line_count, 2*3); // 2 vertices per line

		// Draw points
		glBindVertexArray(VAO_points);
		glDrawArrays(GL_POINTS, points_count, 1);


		if (curr_time - last_time > delay) {
			line_count += 2*3;	//draw 3 lines at a time
			if (line_count >= vertices.size() / 6) {
				line_count = 0; // Reset to the beginning
			}

			points_count++;
			if (points_count >= points.size() / 6) {
				points_count = 0; // Reset to the beginning
			}
			last_time = curr_time;
		}
		glfwSwapBuffers(window);

		//control speed of rendering
		this_thread::sleep_for(std::chrono::milliseconds(16));
	}
}