#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>

#include <glad/glad.h>
#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "render.h"
#include "calculations.h"
#include "arm_attributes.h"
#include "get_path.h"

using Eigen::MatrixXd, Eigen::Vector3d;
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

static glm::vec3 move_camera(cameraMovements input) {
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

static pair<float, glm::vec3> rotate_model(cameraMovements rotation) {
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

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GLFW_TRUE);
	}
}

static void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
}

static GLuint CreateShaderProgram() {

	int success;
	char infoLog[512];
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

	return shaderProgram;
}

glm::vec3 getCamMovement(GLFWwindow* &window) {
	glm::vec3 camOffsets{ 0.0f, 0.0f, 0.0f };

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
	return camOffsets;
}

pair<float,glm::vec3> getCamRotation(GLFWwindow*& window) {
	float modelAngleoffsets = 0.0f;
	glm::vec3 modelRotAxis = { 0.0f, 1.0f, 0.0f };		//y by default

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
	return { modelAngleoffsets, modelRotAxis };
}

//given joint angles, define arm vertices for rendering

struct ArmVertices {

	vector<float> vertices; vector<float> points;
	float x; float y; float z;
	float pitch; float roll; float yaw;

	ArmVertices(float _x, float _y, float _z, float _roll, float _pitch, float _yaw) : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw)
	{
	}

	void animateAngles() {
		float step = 0.05f;

		// compute arm joint angles and positions
		Eigen::Vector3d arm_angles = ik(x, y, z);
		float t1 = arm_angles(0); float t2 = arm_angles(1); float t3 = arm_angles(2);

		Eigen::Vector3d wrist_angles = wristIk(x, y, z, roll, pitch, yaw);
		float t4 = wrist_angles(0); float t5 = wrist_angles(1); float t6 = wrist_angles(2);

		enum class angleIndices : char {
			T1, T2, T3, T4, T5, T6
		};

		vector<angleIndices> armAngles = {
			angleIndices::T1,
			angleIndices::T2,
			angleIndices::T3,
		};

		vector<angleIndices> wristAngles = {
			angleIndices::T4,
			angleIndices::T5,
			angleIndices::T6,
		};

		float t_1 = 0, t_2 = 0, t_3 = 0, t_4 = 0, t_5 = 0, t_6 = 0;

		for (float dx = 0.0f; dx <= 1.0f; dx += step) {
			for (angleIndices t : armAngles) {
				switch (t) {
				case angleIndices::T1:
					t_1 = t_1 + dx * (t1 - t_1);
					break;

				case angleIndices::T2:
					t_2 = t_2 + dx * (t2 - t_2);
					break;
				case angleIndices::T3:
					t_3 = t_3 + dx * (t3 - t_3);
					break;
			}
				vector<float> arm_temp = defineArm(x,y,z,t_1, t_2, t_3, t_4, t_5, t_6);
				vector<float> claw_temp = defineClaw(t_1, t_2, t_3, t_4, t_5, t_6);

				vertices.insert(vertices.end(), arm_temp.begin(), arm_temp.end());
				vertices.insert(vertices.end(), claw_temp.begin(), claw_temp.end());
			}
		}

		for (float dx = 0.0f; dx <= 1.0f; dx += step) {
			for (angleIndices t : wristAngles) {
				switch (t) {
				case angleIndices::T4:
					t_4 = t_4 + dx * (t4 - t_4);
					break;
				case angleIndices::T5:
					t_5 = t_5 + dx * (t5 - t_5);
					break;
				case angleIndices::T6:
					t_6 = t_6 + dx * (t6 - t_6);
					break;
				}
				vector<float> arm_temp = defineArm(x,y,z,t_1, t_2, t_3, t_4, t_5, t_6);
				vector<float> claw_temp = defineClaw(t_1, t_2, t_3, t_4, t_5, t_6);

				vertices.insert(vertices.end(), arm_temp.begin(), arm_temp.end());
				vertices.insert(vertices.end(), claw_temp.begin(), claw_temp.end());

				Eigen::Vector3d end_eff_pos = fk(t_1, t_2, t_3, t_4, t_5, t_6).back();
				points.push_back(end_eff_pos(0)); points.push_back(end_eff_pos(1)); points.push_back(end_eff_pos(2));
				points.push_back(0.0f); points.push_back(1.0f); points.push_back(0.0f); 
			}
		}
	}

	vector<float> getVerts() const { return vertices; }
	vector<float> getPoints() const { return points; }

	vector<float> defineArm(float x, float y, float z, float t1, float t2, float t3, float t4, float t5, float t6) {
		vector<float> arm_verts;

		vector<Eigen::Vector3d> arm_pos = fk(t1, t2, t3, t4, t5, t6);
		
		Eigen::Vector3d link1 = arm_pos[0]; Eigen::Vector3d link2 = arm_pos[1]; Eigen::Vector3d link3 = arm_pos[2];
		Eigen::Vector3d link4 = arm_pos[3]; Eigen::Vector3d link5 = arm_pos[4]; Eigen::Vector3d link5_1 = arm_pos[5];
		Eigen::Vector3d link5_2 = arm_pos[6]; Eigen::Vector3d link6 = arm_pos[7];

		// line from origin to end of link 1
		arm_verts.push_back(0.0f); arm_verts.push_back(0.0f); arm_verts.push_back(0.0f);
		arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); //color
		arm_verts.push_back(link1(0)); arm_verts.push_back(link1(1)); arm_verts.push_back(link1(2));
		arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); //color

		// line from end of link1 to end of link 2
		arm_verts.push_back(link1(0)); arm_verts.push_back(link1(1)); arm_verts.push_back(link1(2));
		arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); //color
		arm_verts.push_back(link2(0)); arm_verts.push_back(link2(1)); arm_verts.push_back(link2(2));
		arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); //color

		// line from end of link2 to end of link 3
		arm_verts.push_back(link2(0)); arm_verts.push_back(link2(1)); arm_verts.push_back(link2(2));
		arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); //color
		arm_verts.push_back(link3(0)); arm_verts.push_back(link3(1)); arm_verts.push_back(link3(2));
		arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); //color

		// wrist joint 1
		arm_verts.push_back(link3(0)); arm_verts.push_back(link3(1)); arm_verts.push_back(link3(2));
		arm_verts.push_back(0.0f); arm_verts.push_back(1.0f); arm_verts.push_back(0.0f); //color
		arm_verts.push_back(link4(0)); arm_verts.push_back(link4(1)); arm_verts.push_back(link4(2));
		arm_verts.push_back(0.0f); arm_verts.push_back(1.0f); arm_verts.push_back(0.0f); //color

		// wrist joint 2
		arm_verts.push_back(link4(0)); arm_verts.push_back(link4(1)); arm_verts.push_back(link4(2));
		arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); arm_verts.push_back(0.0f); //color
		arm_verts.push_back(link5(0)); arm_verts.push_back(link5(1)); arm_verts.push_back(link5(2));
		arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); arm_verts.push_back(0.0f); //color

		// wrist joint 3
		arm_verts.push_back(link5(0)); arm_verts.push_back(link5(1)); arm_verts.push_back(link5(2));
		arm_verts.push_back(0.0f); arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); //color
		arm_verts.push_back(link5_1(0)); arm_verts.push_back(link5_1(1)); arm_verts.push_back(link5_1(2));
		arm_verts.push_back(0.0f); arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); //color

		// wrist joint 3
		arm_verts.push_back(link5_1(0)); arm_verts.push_back(link5_1(1)); arm_verts.push_back(link5_1(2));
		arm_verts.push_back(0.0f); arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); //color
		arm_verts.push_back(link5_2(0)); arm_verts.push_back(link5_2(1)); arm_verts.push_back(link5_2(2));
		arm_verts.push_back(0.0f); arm_verts.push_back(1.0f); arm_verts.push_back(1.0f); //color

		// wrist joint 3
		arm_verts.push_back(link5_2(0)); arm_verts.push_back(link5_2(1)); arm_verts.push_back(link5_2(2));
		arm_verts.push_back(0.0f); arm_verts.push_back(1.0f); arm_verts.push_back(0.5f); //color
		arm_verts.push_back(link6(0)); arm_verts.push_back(link6(1)); arm_verts.push_back(link6(2));
		arm_verts.push_back(0.0f); arm_verts.push_back(1.0f); arm_verts.push_back(0.5f); //color

		return arm_verts;
	}

	vector<float> defineClaw(float t1, float t2, float t3, float t4, float t5, float t6) {

		vector<float> claw_verts;
		vector<Eigen::Vector3d> arm_pos = fk(t1, t2, t3, t4, t5, t6);
		Eigen::Vector3d wrist_base = arm_pos.back();
		//roll about X
		MatrixXd rot4(3, 3);
		rot4 << 1, 0, 0,
			0, cos(t4), -sin(t4),
			0, sin(t4), cos(t4);

		//roll about Y
		MatrixXd rot5(3, 3);
		rot5 << cos(t5), 0, sin(t5),
			0, 1, 0,
			-sin(t5), 0, cos(t5);

		//pitch about X
		MatrixXd rot6(3, 3);
		rot6 << 1, 0, 0,
			0, cos(t6), -sin(t6),
			0, sin(t6), cos(t6);

		Eigen::Vector3d wrist_link = arm_pos.back() - arm_pos[arm_pos.size()-2];

		cout << "End eff pos " << arm_pos.back().transpose() << endl;

		Eigen::Vector3d perp_base{ 0.0f, 1.0, 0.0f };			//for the base of claw
		Eigen::Vector3d perp_pinch{ 0.0f, 1.0, 0.0f };			//for the base of claw

		//Eigen::Vector3d perp = { wrist_base(0), wrist_base(1), 0.0f };
		Eigen::Vector3d wrist_offset = perp_base.cross(wrist_link).normalized();

		//wrist_offset = rot6 *wrist_offset;

		Eigen::Vector3d claw_left{ 1.0, 0.0, 0.0f };
		Eigen::Vector3d claw_right{ 1.0, 0.0, 0.0f };

		claw_left = -perp_pinch.cross(wrist_offset).normalized() * 2.0;
		claw_right = -perp_pinch.cross(wrist_offset).normalized() * 2.0;

		Eigen::Vector3d left_wb_point = wrist_base + 0.5 * wrist_offset;
		Eigen::Vector3d right_wb_point = wrist_base - 0.5 * wrist_offset;

		Eigen::Vector3d claw_left_point = claw_left + left_wb_point;
		Eigen::Vector3d claw_right_point = claw_right + right_wb_point;

		// line from wrist to left of wrist base	
		claw_verts.push_back(wrist_base(0)); claw_verts.push_back(wrist_base(1)); claw_verts.push_back(wrist_base(2));
		claw_verts.push_back(1.0f); claw_verts.push_back(0.0f); claw_verts.push_back(0.0f);		//color
		claw_verts.push_back(left_wb_point(0)); claw_verts.push_back(left_wb_point(1)); claw_verts.push_back(left_wb_point(2));
		claw_verts.push_back(1.0f); claw_verts.push_back(0.0f); claw_verts.push_back(0.0f);		//color

		// line from wrist to right of wrist base	
		claw_verts.push_back(wrist_base(0)); claw_verts.push_back(wrist_base(1)); claw_verts.push_back(wrist_base(2));
		claw_verts.push_back(1.0f); claw_verts.push_back(0.0f); claw_verts.push_back(0.0f);		//color
		claw_verts.push_back(right_wb_point(0)); claw_verts.push_back(right_wb_point(1)); claw_verts.push_back(right_wb_point(2));
		claw_verts.push_back(1.0f); claw_verts.push_back(0.0f); claw_verts.push_back(0.0f);		//color

		// line from claw left to left of end of claw
		claw_verts.push_back(left_wb_point(0)); claw_verts.push_back(left_wb_point(1)); claw_verts.push_back(left_wb_point(2));
		claw_verts.push_back(1.0f); claw_verts.push_back(0.0f); claw_verts.push_back(0.0f);		//color
		claw_verts.push_back(claw_left_point(0)); claw_verts.push_back(claw_left_point(1)); claw_verts.push_back(claw_left_point(2));
		claw_verts.push_back(1.0f); claw_verts.push_back(0.0f); claw_verts.push_back(0.0f);		//color

		// line from claw right to right of end of claw
		claw_verts.push_back(right_wb_point(0)); claw_verts.push_back(right_wb_point(1)); claw_verts.push_back(right_wb_point(2));
		claw_verts.push_back(1.0f); claw_verts.push_back(0.0f); claw_verts.push_back(0.0f);		//color
		claw_verts.push_back(claw_right_point(0)); claw_verts.push_back(claw_right_point(1)); claw_verts.push_back(claw_right_point(2));
		claw_verts.push_back(1.0f); claw_verts.push_back(0.0f); claw_verts.push_back(0.0f);		//color

		return claw_verts;
	}
};

struct Target {
	vector<float> target_verts;
	//defult constructor

	enum class targetColors : char {
		RED,
		GREEN,
		BLUE
	};

	Target() {}

	vector<float> defineTarget(float x, float y, float z, targetColors color, float size) {
		float l = size;
		float c1, c2, c3;

		switch (color) {
		case (targetColors::RED):
				c1 = 1.0f; c2 = 0.0f; c3 = 0.0f;
				break;
		case (targetColors::GREEN):
				c1 = 0.0f; c2 = 1.0f; c3 = 0.0f;
				break;
		case (targetColors::BLUE):
			c1 = 0.0f; c2 = 0.0f; c3 = 1.0f;
			break;
		default:
			c1 = 1.0f; c2 = 1.0f; c3 = 1.0f;
		}

		target_verts = {
			x - l, y - l, z + l,   c1,c2,c3,
			x + l, y - l, z + l,   c1,c2,c3,
			x + l, y + l, z + l,   c1,c2,c3,

			x - l, y - l, z + l,   c1,c2,c3,
			x + l, y + l, z + l,   c1,c2,c3,
			x - l, y + l, z + l,   c1,c2,c3,

			// Back face (c1,c2,c3)
			x - l, y - l, z - l,   c1,c2,c3,
			x + l, y + l, z - l,   c1,c2,c3,
			x + l, y - l, z - l,   c1,c2,c3,

			x - l, y - l, z - l,   c1,c2,c3,
			x - l, y + l, z - l,   c1,c2,c3,
			x + l, y + l, z - l,   c1,c2,c3,

			// Left face (c1,c2,c3)
			x - l, y - l, z - l,   c1,c2,c3,
			x - l, y + l, z + l,   c1,c2,c3,
			x - l, y - l, z + l,   c1,c2,c3,

			x - l, y - l, z - l,   c1,c2,c3,
			x - l, y + l, z - l,   c1,c2,c3,
			x - l, y + l, z + l,   c1,c2,c3,

			// Right face (c1,c2,c3)
			x + l, y - l, z - l,   c1,c2,c3,
			x + l, y - l, z + l,   c1,c2,c3,
			x + l, y + l, z + l,   c1,c2,c3,

			x + l, y - l, z - l,   c1,c2,c3,
			x + l, y + l, z + l,   c1,c2,c3,
			x + l, y + l, z - l,   c1,c2,c3,

			// Top face (c1,c2,c3)
			x - l, y + l, z + l,   c1,c2,c3,
			x + l, y + l, z + l,   c1,c2,c3,
			x + l, y + l, z - l,   c1,c2,c3,

			x - l, y + l, z + l,   c1,c2,c3,
			x + l, y + l, z - l,   c1,c2,c3,
			x - l, y + l, z - l,   c1,c2,c3,

			// Bottom face
			x - l, y - l, z + l,   c1,c2,c3,
			x + l, y - l, z - l,   c1,c2,c3,
			x + l, y - l, z + l,   c1,c2,c3,

			x - l, y - l, z + l,   c1,c2,c3,
			x - l, y - l, z - l,   c1,c2,c3,
			x + l, y - l, z - l,   c1,c2,c3,
		};

		return target_verts;
	}
};

struct Engine {

	ArmVertices arm;
	Target target;
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
		0.0f,0.0f,20.0f,			0.0f,1.0f,0.0f,
		0.0f,0.0f,-20.0f,			0.0f,1.0f,0.0f,
	};

	vector<float> vertices; vector<float> target_verts; vector<float> points;

	// lines VAO/VBO 
	GLuint VAO_lines, VBO_lines;
	GLuint VAO_target, VBO_target;	
	GLuint VAO_axis, VBO_axis;
	GLuint VAO_points, VBO_points;

	int line_count = 0;

	Engine(float _x, float _y, float _z, float _roll, float _pitch, float _yaw) 
		: arm(_x, _y, _z, _roll, _pitch, _yaw)
	{}


	void loadRender() {

		arm.animateAngles();

		target_verts = target.defineTarget(arm.x,arm.y,arm.z, Target::targetColors::RED, 0.5);
		vertices = arm.getVerts();
		points = arm.getPoints();

		int total_floats = (int)vertices.size();
		int total_vertices = total_floats / 6; // because 6 floats per vertex

		// ==================== Lines VAO/VBO =============

		glGenVertexArrays(1, &VAO_lines);
		glGenBuffers(1, &VBO_lines);
		glBindVertexArray(VAO_lines);
		glBindBuffer(GL_ARRAY_BUFFER, VBO_lines);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);

		//positions
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		//positions
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		// ============= Axis VAO/VBO  =============
		glGenVertexArrays(1, &VAO_axis);
		glGenBuffers(1, &VBO_axis);
		glBindVertexArray(VAO_axis);
		glBindBuffer(GL_ARRAY_BUFFER, VBO_axis);

		//positions
		glBufferData(GL_ARRAY_BUFFER, axis.size() * sizeof(float), axis.data(), GL_DYNAMIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		//color
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		
		// ==================== Target VAO/VBO =============
		glGenVertexArrays(1, &VAO_target);
		glGenBuffers(1, &VBO_target);
		glBindVertexArray(VAO_target);
		glBindBuffer(GL_ARRAY_BUFFER, VBO_target);
		glBufferData(GL_ARRAY_BUFFER, target_verts.size() * sizeof(float), target_verts.data(), GL_DYNAMIC_DRAW);

		//positions
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		//color
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		// ==================== Point VAO/VBO =============
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
		
	}

	void draw(glm::mat4 view, glm::mat4 model, glm::mat4 projection, float dt, float delay) {

		// Use shader
		GLuint shaderProgram = CreateShaderProgram();
		glUseProgram(shaderProgram);
		glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
		glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

		//define line width and point size
		glLineWidth(14.0f);
		glPointSize(10.0f);	

		const int verts_num = 24;		// number of vertices per frame (10 lines, 2 vertices each)

		// ========== Draw axis ==========
		glBindVertexArray(VAO_axis);
		glDrawArrays(GL_LINES, 0, axis.size() / 6);

		// ========= Draw Target ==========
		glBindVertexArray(VAO_target);
		glDrawArrays(GL_TRIANGLES, 0, 36);

		// ========== Draw arm lines ==========
		glBindVertexArray(VAO_lines);
		glDrawArrays(GL_LINES, line_count, verts_num);

		// ========== Draw arm lines ==========
		glBindVertexArray(VAO_points);
		glDrawArrays(GL_POINTS, 0, points.size()/6);

		if (dt > delay) {
			line_count += verts_num;
			if (line_count >= vertices.size() / 6) {
				line_count = 0; // Reset frame count
				//cout << "\nCompleted animation cycle. Resetting frame count to 0.\n";
			}
		}
	
	}
};

void render() {
	float x = 20.f;
	float y = 20.0f;
	float z = -5.f;
	float roll = PI/2;
	float pitch = 0;
	float yaw = 0;

	Engine engine(x, y,z, roll, pitch, yaw); //initialize engine with target position
	// GLFW init for the window
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(1600, 1200, "Arm Simulation", NULL, NULL);
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

	//initialize the rendering engine with some points and lines
	engine.loadRender();

	// ================ Set camera view =================

	// Initial camera position 	

	glm::vec3 init_camPos = { 0.f, 0.f, 60.f};
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

	glEnable(GL_DEPTH_TEST);
	float last_time = glfwGetTime(), delay = 0.005f;

	//renderloop
	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();
		float curr_time = glfwGetTime();

		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//set camera movements and model positions
		glm::vec3 camOffsets = getCamMovement(window);
		float modelAngleoffsets = getCamRotation(window).first; 
		glm::vec3 modelRotAxis = getCamRotation(window).second;

		cameraPos = curr_camPos + camOffsets;

		//reset position
	
		if (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) {
			cameraTarget = { 0.0f, 0.0f, 0.0f };
			cameraPos = init_camPos;
			curr_camPos = init_camPos;
			model = glm::translate(model, { 0.0f,0.0f, 0.0f }); //reset model position
			cout << "fixing to origin\n";
		}
		
		glm::vec3 cameraTarget = cameraPos + cameraFront;
		glm::mat4 view = glm::lookAt(cameraPos, cameraTarget, cameraUp);
		
		curr_camPos += camOffsets;			//reset 

		model = glm::rotate(model, modelAngleoffsets, modelRotAxis); // rotate the model without moving camera
			
		// Reset rotation

		if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {
			model = glm::mat4(1.0f);
			cout << "fixing viewport...";
		}

		//draw lines and points with delay
		float dt = curr_time - last_time;

		engine.draw(view, model, projection, dt, delay);
 
		if (curr_time - last_time > delay) {
			last_time = curr_time;
		}

		glfwSwapBuffers(window);

		//control speed of rendering
		this_thread::sleep_for(std::chrono::milliseconds(16));		// ~60 FPS
	}
}