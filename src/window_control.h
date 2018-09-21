// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#include <string>
#include <sstream>
#include <iostream>

//////////////////////////////
// Basic Data Types         //
//////////////////////////////

struct float3 { float x, y, z; };
struct float2 { float x, y; };

struct rect
{
	float x, y;
	float w, h;

	// Create new rect within original boundaries with give aspect ration
	rect adjust_ratio(float2 size) const
	{
		auto H = static_cast<float>(h), W = static_cast<float>(h) * size.x / size.y;
		if (W > w) {
			auto scale = w / W;
			W *= scale;
			H *= scale;
		}

		return { x + (w - W) / 2, y + (h - H) / 2, W, H };
	}
};

//////////////////////////////
// Simple font loading code //
//////////////////////////////

#include "../third-party/stb_easy_font.h"

inline void draw_text(int x, int y, const char * text)
{
	char buffer[60000]; // ~300 chars
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_FLOAT, 16, buffer);
	glDrawArrays(GL_QUADS, 0, 4 * stb_easy_font_print((float) x, (float) (y - 7), (char *) text, nullptr, buffer, sizeof(buffer)));
	glDisableClientState(GL_VERTEX_ARRAY);
}

////////////////////////
// Image display code //
////////////////////////
class texture
{
public:
	void render(const rs2::video_frame& frame, const rect& r)
	{
		upload(frame);
		show(r.adjust_ratio({ float(width), float(height) }));
	}

	void upload(const rs2::video_frame& frame)
	{
		if (!frame) return;

		if (!gl_handle)
			glGenTextures(1, &gl_handle);
		GLenum err = glGetError();

		auto format = frame.get_profile().format();
		width = frame.get_width();
		height = frame.get_height();
		stream = frame.get_profile().stream_type();

		glBindTexture(GL_TEXTURE_2D, gl_handle);

		switch (format) {
			case RS2_FORMAT_RGB8:
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, frame.get_data());
				break;
			case RS2_FORMAT_RGBA8:
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, frame.get_data());
				break;
			case RS2_FORMAT_Y8:
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, frame.get_data());
				break;
			default:
				throw std::runtime_error("The requested format is not supported by this demo!");
		}

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	GLuint get_gl_handle() { return gl_handle; }

	void show(const rect& r) const
	{
		if (!gl_handle)
			return;

		glBindTexture(GL_TEXTURE_2D, gl_handle);
		glEnable(GL_TEXTURE_2D);
		glBegin(GL_QUAD_STRIP);
		glTexCoord2f(0.f, 1.f); glVertex2f(r.x, r.y + r.h);
		glTexCoord2f(0.f, 0.f); glVertex2f(r.x, r.y);
		glTexCoord2f(1.f, 1.f); glVertex2f(r.x + r.w, r.y + r.h);
		glTexCoord2f(1.f, 0.f); glVertex2f(r.x + r.w, r.y);
		glEnd();
		glDisable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, 0);

		draw_text((int) r.x + 15, (int) r.y + 20, rs2_stream_to_string(stream));
	}
private:
	GLuint gl_handle = 0;
	int width = 0;
	int height = 0;
	rs2_stream stream = RS2_STREAM_ANY;
};

// Struct for managing rotation of pointcloud view
struct glfw_state
{
	glfw_state() : yaw(15.0), pitch(15.0), last_x(0.0), last_y(0.0),
		ml(false), offset_x(2.f), offset_y(2.f), tex()
	{
	}
	double yaw;
	double pitch;
	double last_x;
	double last_y;
	bool ml;
	float offset_x;
	float offset_y;
	texture tex;
};


// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(float width, float height, glfw_state& app_state, rs2::points& points)
{
	if (!points)
		return;

	// OpenGL commands that prep screen for the pointcloud
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	glClear(GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, width / height, 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

	glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
	float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
	glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
	glBegin(GL_POINTS);


	/* this segment actually prints the pointcloud */
	auto vertices = points.get_vertices();              // get vertices
	auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
	for (int i = 0; i < points.size(); i++) {
		if (vertices[i].z) {
			// upload the point and texture coordinates only for points we have depth data for
			glVertex3fv(vertices[i]);
			glTexCoord2fv(tex_coords[i]);
		}
	}

	// OpenGL cleanup
	glEnd();
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}
