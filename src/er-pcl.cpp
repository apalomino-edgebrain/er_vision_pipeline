// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../../../examples/example.hpp" // Include short list of convenience functions for rendering

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

// Struct for managing rotation of pointcloud view
struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
        ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr;

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points);

bool get_texcolor(const rs2::video_frame *frame, float u, float v, uint8_t &r, uint8_t &g, uint8_t &b)
{
	auto ptr = frame;
	if (ptr == nullptr) {
		return false;
	}

	const int w = ptr->get_width(), h = ptr->get_height();
	int x = std::min(std::max(int(u*w + .5f), 0), w - 1);
	int y = std::min(std::max(int(v*h + .5f), 0), h - 1);
	int idx = x * ptr->get_bits_per_pixel() / 8 + y * ptr->get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(ptr->get_data());

	r = texture_data[idx++];
	g = texture_data[idx++];
	b = texture_data[idx];

	return true;
}

bool get_texinfrared(const rs2::video_frame *frame, float u, float v, uint8_t &a)
{
	auto ptr = frame;
	if (ptr == nullptr) {
		return false;
	}

	const int w = ptr->get_width(), h = ptr->get_height();
	int x = std::min(std::max(int(u*w + .5f), 0), w - 1);
	int y = std::min(std::max(int(v*h + .5f), 0), h - 1);
	int idx = x * ptr->get_bits_per_pixel() / 8 + y * ptr->get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(ptr->get_data());

	a = texture_data[idx];
	return true;
}

pcl_ptr points_to_pcl(rs2::pointcloud &pc, const rs2::video_frame &color_depth_map, const rs2::video_frame &color, const rs2::video_frame &infrared, const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pc.map_to(color_depth_map);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
	auto tex_coords_depth = points.get_texture_coordinates();

	unsigned int i = 0;

	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;

		get_texcolor(&color_depth_map, tex_coords_depth[i].u, tex_coords_depth[i].v, p.r, p.g, p.b);
		i++;
		ptr++;
	}

	pc.map_to(color);
	auto tex_coords_color = points.get_texture_coordinates();

	i = 0;
	ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;

		if (tex_coords_color[i].u < 0 || tex_coords_color[i].u > 1 || tex_coords_color[i].v < 0 || tex_coords_color[i].v > 1) {
		} else {
			get_texcolor(&color, tex_coords_color[i].u, tex_coords_color[i].v, p.r, p.g, p.b);
		}
		i++;
        ptr++;
    }

    return cloud;
}

float3 colors[] { { 0.8f, 0.1f, 0.3f },
                  { 0.1f, 0.9f, 0.5f },
                };

using namespace rs2;

class er_pointcloud : public options
{
public:
	er_pointcloud() : _queue(1)
	{
		rs2_error* e = nullptr;

		auto pb = std::shared_ptr<rs2_processing_block>(
			rs2_create_pointcloud(&e),
			rs2_delete_processing_block);
		_block = std::make_shared<processing_block>(pb);

		error::handle(e);

		// Redirect options API to the processing block
		options::operator=(pb);

		_block->start(_queue);
	}

	points calculate(frame depth)
	{
		_block->invoke(std::move(depth));
		rs2::frame f;
		if (!_queue.poll_for_frame(&f))
			throw std::runtime_error("Error occured during execution of the processing block! See the log for more info");
		return points(f);
	}

	void map_to(frame mapped)
	{
		_block->set_option(RS2_OPTION_TEXTURE_SOURCE, float(mapped.get_profile().unique_id()));
		_block->invoke(std::move(mapped));
	}
private:
	friend class context;

	std::shared_ptr<processing_block> _block;
	frame_queue _queue;
};


int main(int argc, char * argv[]) try
{
	rs2::config cfg;

	if (argc > 1) {
		char file_record[512];
		strcpy(file_record, argv[1]);

		cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
		cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 15);
		cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 15);

		cfg.enable_device_from_file(file_record);
	}

    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "Earth Rover PCL Pipeline");
    // Construct an object to manage view state
    state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings

	rs2::colorizer color_map;

	// Create a shared pointer to a pipeline
	auto pipe = std::make_shared<rs2::pipeline>();

	// Start streaming with default recommended configuration
	pipe->start(cfg);

	auto profile = pipe->get_active_profile();
	auto device = profile.get_device();

	rs2::frameset frames;

	rs2::playback playback = device.as<rs2::playback>();

	int current_frame = 0;
	unsigned int i = 0;

	device.as<rs2::playback>().set_real_time(false);
	std::vector<pcl_ptr> layers;

	er_pointcloud pc;

	pointcloud pc2;
	while (app) // Application still alive?
    {

		// Wait for the next set of frames from the camera

		if (device.as<rs2::playback>())
			if (pipe->poll_for_frames(&frames)) {
				layers.clear();

				auto depth = frames.get_depth_frame();
				rs2::video_frame color = frames.get_color_frame();
				rs2::video_frame infrared = frames.get_infrared_frame();

				// Find and colorize the depth data
				//rs2::video_frame color_depth_map = color_map(frames.get_depth_frame());

				// Generate the pointcloud and texture mappings
				// We want the points object to be persistent so we can display the last cloud when a frame drops

				pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

				{
					rs2::points points;
					points = pc2.calculate(depth);

					auto sp = points.get_profile().as<rs2::video_stream_profile>();
					cloud->width = sp.width();
					cloud->height = sp.height();
					cloud->is_dense = false;
					cloud->points.resize(points.size());

					pc2.map_to(infrared);

					auto ptr = points.get_vertices();
					auto tex_coords = points.get_texture_coordinates();

					i = 0;
					for (auto& p : cloud->points)
					{
						p.x = ptr->x;
						p.y = ptr->y;
						p.z = ptr->z;
						get_texinfrared(&infrared, tex_coords[i].u, tex_coords[i].v, p.a);
						p.r = p.g = p.b = p.a;

						i++;
						ptr++;
					}

				}

				{
					rs2::points points;
					points = pc.calculate(depth);

					pc.map_to(color);

					auto ptr = points.get_vertices();
					auto tex_coords = points.get_texture_coordinates();

					i = 0;
					for (auto& p : cloud->points)
					{
						if (tex_coords[i].u < 0 || tex_coords[i].u > 1 || tex_coords[i].v < 0 || tex_coords[i].v > 1) {
						} else {

							get_texcolor(&color, tex_coords[i].u, tex_coords[i].v, p.r, p.g, p.b);

							float nvdi = float(p.a - p.r) / (p.a + p.r);
							if (nvdi < - 0.25) {
								p.r = 0;
								p.g = 0;
								p.b = 0;
								//printf(" NVDI %2.2f \n ", nvdi);
							}
						}
						i++;
						ptr++;
					}

				}

				pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
				pcl::PassThrough<pcl::PointXYZRGBA> pass;
				pass.setInputCloud(cloud);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(0.0, 10.0);
				pass.filter(*cloud_filtered);
				layers.push_back(cloud_filtered);

				//layers.push_back(cloud);
			}

		draw_pointcloud(app, app_state, layers);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, state& app_state)
{
    app.on_left_mouse = [&](bool pressed)
    {
        app_state.ml = pressed;
    };

    app.on_mouse_scroll = [&](double xoffset, double yoffset)
    {
        app_state.offset_x += static_cast<float>(xoffset);
        app_state.offset_y += static_cast<float>(yoffset);
    };

    app.on_mouse_move = [&](double x, double y)
    {
        if (app_state.ml)
        {
            app_state.yaw -= (x - app_state.last_x);
            app_state.yaw = std::max(app_state.yaw, -120.0);
            app_state.yaw = std::min(app_state.yaw, +120.0);
            app_state.pitch += (y - app_state.last_y);
            app_state.pitch = std::max(app_state.pitch, -80.0);
            app_state.pitch = std::min(app_state.pitch, +80.0);
        }
        app_state.last_x = x;
        app_state.last_y = y;
    };

    app.on_key_release = [&](int key)
    {
        if (key == 32) // Escape
        {
            app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
        }
    };
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points)
{
    // OpenGL commands that prep screen for the pointcloud
    glPopMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    float width = app.width(), height = app.height();

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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
    glEnable(GL_TEXTURE_2D);

    int color = 0;

    for (auto&& pc : points)
    {
        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

        glBegin(GL_POINTS);
		glPointSize(5);

		/* this segment actually prints the pointcloud */
        for (int i = 0; i < pc->points.size(); i++)
        {
            auto&& p = pc->points[i];
            if (p.z)
            {
                // upload the point and texture coordinates only for points we have depth data for
				glColor3f(float(p.r) / 255.f, float(p.g) / 255.f, float(p.b) / 255.f);
				glVertex3f(p.x, p.y, p.z);
            }
        }

        glEnd();
    }

    // OpenGL cleanup
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}
