//#############################################################################
                                 /*-:::--.`
                            -+shmMMNmmddmmNMNmho/.
                 `yyo++osddMms:                  `/yNNy-
              yo    +mMy:                       `./dMMdyssssso-
              oy  -dMy.                     `-+ssso:.`:mMy`   .ys
                ho+MN:                  `:osso/.         oMm-   +h
                +Mmd-           `/syo-                     :MNhs`
                `NM-.hs`      :syo:                          sMh
                oMh   :ho``/yy+.                             `MM.
                hM+    `yNN/`                                 dM+
                dM/  -sy/`/ho`                                hMo
                hMo/ho.     :yy-                             dM/
            :dNM/             :yy:                         yMy
            sy`:MN.              `+ys-                     +Mm`
            oy`   :NM+                  .+ys/`           `hMd.ys
            /sssssyNMm:                   `:sys:`     `oNN+   m-
                        .sNMh+.                   `:sNMdyysssssy:
                        -odMNhs+:-.`    `.-/oydMNh+.
                            `-+shdNMMMMMMMNmdyo/.
                                    `````*/
//#############################################################################
// Point cloud processing from the Realsense
//
// Integrates the PCL library to enable a tool to perform analyse on the images
// and video
//#############################################################################

#include "er-pipeline.h"
#include "process_3d.h"
#include "window_control.h"

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, state& app_state)
{
	app.on_left_mouse = [&] (bool pressed) {
		app_state.ml = pressed;
	};

	app.on_mouse_scroll = [&] (double xoffset, double yoffset) {
		app_state.offset_x += static_cast<float>(xoffset);
		app_state.offset_y += static_cast<float>(yoffset);
	};

	app.on_mouse_move = [&] (double x, double y) {
		if (app_state.ml) {
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

	app.on_key_release = [&] (int key) {
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

	glTranslatef(0, 0, 1.5f + app_state.offset_y*0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_TEXTURE_2D);

	int color = 0;

	for (auto&& pc : points) {
		glBegin(GL_POINTS);
		glPointSize(5);

		/* this segment actually prints the pointcloud */
		for (int i = 0; i < pc->points.size(); i++) {
			auto&& p = pc->points[i];
			if (p.z) {
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

void draw_polygons(window& app, state& app_state)
{
	// OpenGL commands that prep screen for the pointcloud
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	float width = app.width(), height = app.height();

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

	//----------
	glBegin(GL_TRIANGLES);

	glColor3f(0.5f, 0.0f, 1.0f);
	glVertex3f(-1.0f, -0.5f, 4.0f);
	glVertex3f(1.0f, 0.5f, 4.0f);
	glVertex3f(0.0f, 0.5f, 4.0f);
	glEnd();
	//----------

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}

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
	ImGui_ImplGlfw_Init(app, false);

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

	bool show_app = true;

	bool show_analysis = true;

	bool show_ground = true;
	bool show_plants = true;

	bool bool_cloud_raw = false;
	bool bool_color_cluster = false;
	bool bool_distance_filter = true;
	bool bool_voxel_process = false;

	float min_nvdi = 1;
	float cur_nvdi = -1;
	float max_nvdi = -1;

	float cur_max_clip[3] = { 0, 1, 1 };
	float cur_min_clip[3] = { 0, 0, 0 };

	float min_clip[3] = { 0, 0, 50 };
	float max_clip[3] = { 0, 50, -50 };

	ImGui::SetNextWindowSize({ 300, 100 });

	er::field_map field;

	while (app) // Application still alive?
    {
		static const int flags = ImGuiWindowFlags_AlwaysAutoResize;

		ImGui_ImplGlfw_NewFrame(1);

		// Wait for the next set of frames from the camera

		if (device.as<rs2::playback>())
			if (pipe->poll_for_frames(&frames)) {
				layers.clear();

				auto depth = frames.get_depth_frame();

				uint32_t frame = frames.get_frame_number();

				rs2::video_frame color = frames.get_color_frame();
				rs2::video_frame infrared = frames.get_infrared_frame();

				// Find and colorize the depth data
				// rs2::video_frame color_depth_map = color_map(frames.get_depth_frame());

				// Generate the pointcloud and texture mappings
				// We want the points object to be persistent so we can display the last cloud when a frame drops

				//------------- Point cloud creation ------------------------
				rs2::points points;

				points = pc2.calculate(depth);

				pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
				{
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

						//-- Z Clipping max and min values--

						if (p.z > max_clip[2])
							max_clip[2] = p.z;

						if (p.z < min_clip[2])
							min_clip[2] = p.z;

						//-- Y Clipping max and min values--
						if (p.y > max_clip[1])
							max_clip[1] = p.y;

						if (p.y < min_clip[1])
							min_clip[1] = p.y;

						get_texinfrared(&infrared, tex_coords[i].u, tex_coords[i].v, p.a);
						p.r = p.g = p.b = p.a;

						i++;
						ptr++;
					}

				}

				//------------- Infrared extraction ------------------------

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

							if (show_ground && show_plants) {
							} else {
								if (show_ground) {
									if (nvdi > cur_nvdi) {
										p.r = p.g = p.b = 0;
										p.z = -10;
									}
								}

								if (show_plants) {
									if (nvdi < cur_nvdi) {
										p.r = p.g = p.b = 0;
										p.z = -10;
									}
								}
							}

							if (nvdi < min_nvdi)
								min_nvdi = nvdi;

							if (nvdi > max_nvdi)
								max_nvdi = nvdi;

						}
						i++;
						ptr++;
					}

				}


				//----------- Normal computation ----------------------

				// Create the normal estimation class, and pass the input dataset to it

				//------------- Filter limit ------------------------
				if (bool_distance_filter) {
					pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
					pcl::PassThrough<pcl::PointXYZRGBA> pass;
					pass.setInputCloud(cloud);
					pass.setFilterFieldName("z");
					pass.setFilterLimits(cur_min_clip[2], cur_max_clip[2]);

					//pass.setFilterFieldName("y");
					//pass.setFilterLimits(cur_min_clip[1], cur_max_clip[1]);

					pass.filter(*cloud_filtered);

					layers.push_back(cloud_filtered);
					/*
					pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
					ne.setInputCloud(cloud_filtered);
					pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
					ne.setSearchMethod(tree);
					pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
					ne.setRadiusSearch(0.3);

					// Compute the features
					ne.compute(*cloud_normals);
					*/
				} else
				if (bool_cloud_raw) {
					layers.push_back(cloud);
				}

				//------------- VOXEL -----------------------------
				pcl_ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGBA>);

				pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
				sor.setInputCloud(cloud);
				sor.setLeafSize(0.1f, 0.1f, 0.1f);
				sor.filter(*cloud_voxel);

				if (bool_voxel_process) {
					layers.push_back(cloud_voxel);
				}

				//------------- COLOR Cluster ------------------------

				if (bool_color_cluster && cloud->points.size() > 0) {

					pcl::search::Search <pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> >(new pcl::search::KdTree<pcl::PointXYZRGBA>);
					pcl::IndicesPtr indices(new std::vector <int>);

					pcl::PassThrough<pcl::PointXYZRGBA> pass;
					pass.setInputCloud(cloud);
					pass.setFilterFieldName("z");
					pass.setFilterLimits(0.0, 10.0);
					pass.filter(*indices);

					pcl::RegionGrowingRGB<pcl::PointXYZRGBA> reg;
					reg.setInputCloud(cloud);
					reg.setIndices(indices);
					reg.setSearchMethod(tree);
					reg.setDistanceThreshold(1);
					reg.setPointColorThreshold(6);
					reg.setRegionColorThreshold(5);
					reg.setMinClusterSize(600);

					std::vector <pcl::PointIndices> clusters;
					reg.extract(clusters);

					pcl::PointCloud <pcl::PointXYZRGBA>::Ptr colored_cloud = reg.getColoredCloudRGBA();
					if (colored_cloud)
						layers.push_back(colored_cloud);
				}


				/*
				// Normal estimation*
				pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
				ne.setInputCloud(cloud);
				pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());

				ne.setSearchMethod(tree);
				ne.setKSearch(20);

				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
				//ne.setRadiusSearch(0.3);
				ne.compute(*cloud_normals);
				*/

				//* normals should not contain the point normals + surface curvatures

				// Concatenate the XYZ and normal fields*
				//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
				//pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

			}

		//------ BASIC FLOOR DISCOVERY & MAPPING ------
		// Create floor grid
		//  Width x Height

		// Algorithm of floor removal and floor discovery
		// Split in two clusters
		//		1. Some green + and nvdi = Plant
		//		2. No color, just ground

		// Reduce floor cloud density with a voxelgrid
		//
		// Raytrace points from floor grid up to find the right position in space
		//
		// Classify between floor and plants


		draw_pointcloud(app, app_state, layers);
		//draw_polygons(app, app_state);

		{
			ImGui::Begin("app", nullptr, flags);
			ImGui::Text("Color Spaces");
			ImGui::SliderFloat("NVDI", &cur_nvdi, min_nvdi, max_nvdi);
			//ImGui::SliderFloat("HSV", &cur_nvdi, min_nvdi, max_nvdi);

			ImGui::Text("Clipping Z");
			ImGui::SliderFloat("Min Z", &cur_min_clip[2], min_clip[2], max_clip[2]);
			ImGui::SliderFloat("Max Z", &cur_max_clip[2], min_clip[2], max_clip[2]);

			ImGui::Text("Clipping Y");
			ImGui::SliderFloat("clipping Y", &cur_max_clip[1], min_clip[1], max_clip[1]);
			ImGui::End();
		}

		{
			ImGui::Begin("Analysis", &show_analysis, flags);
			ImGui::Checkbox("Show ground", &show_ground);
			ImGui::Checkbox("Show plants", &show_plants);
			ImGui::Separator();

			ImGui::Checkbox("Raw Cloud", &bool_cloud_raw);
			ImGui::Checkbox("Color cluster", &bool_color_cluster);
			ImGui::Checkbox("Voxel Process", &bool_voxel_process);
			ImGui::Checkbox("Distance Filter", &bool_distance_filter);

			ImGui::End();
		}

		ImGui::Render();
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
