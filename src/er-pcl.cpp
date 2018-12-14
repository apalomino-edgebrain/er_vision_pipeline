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

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

//-----------------------------------------------------------------------------
// UI DRAWING
//-----------------------------------------------------------------------------

#include "window_control.h"
#include "visualizer.h"

#include <pcl/kdtree/kdtree_flann.h>
#include "process_3d.h"
#include "application_state.h"

using namespace er;
using namespace rs2;

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

pcl_ptr points_to_pcl(rs2::pointcloud &pc, const rs2::video_frame &color_depth_map,
	const rs2::video_frame &color, const rs2::video_frame &infrared, const rs2::points& points)
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

	for (auto& p : cloud->points) {
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
	for (auto& p : cloud->points) {
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

int main(int argc, char * argv[]) try {

#ifdef WIN32
	HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
	DWORD dwMode = 0;
	GetConsoleMode(hOut, &dwMode);
	dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
	SetConsoleMode(hOut, dwMode);
#endif

	char file_record[512];
	bool read_file = false;

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
	cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 15);
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 15);
	bool realtime = true;

	if (argc > 1) {
		strcpy(file_record, argv[1]);

		cfg.enable_device_from_file(file_record);

		printf_h1("Loading %s ", file_record);

		app_state::get().set_current_file(file_record);
		app_state::get().invalidate_playback = false;
		read_file = true;
		realtime = false;

	} else {
		std::cout << "Not enough parameters for playback." << std::endl << std::endl;
		std::cout << "Use example: " << std::endl;
		std::cout << "  er-vision ~/data/8799ecf/180904_135414/capture.bag" << std::endl;

		// We enable recording to disk
		cfg.enable_record_to_file("disk_record.bag");
	}

	Eigen::initParallel();
	worker_t *worker = launch_visualizer();

	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "Earth Rover PCL Pipeline");

	// Construct an object to manage view state
	state app_state;

	// Register callbacks to allow manipulation of the pointcloud
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

	int current_frame = 0;
	unsigned int i = 0;

	std::vector<pcl_ptr> layers;

	rs2::pointcloud pc;
	rs2::pointcloud pc2;

	er::pipeline er_pipe = er::pipeline::get();

	if (read_file)
		er_pipe.initialize_folder(app_state::get().capture_folder);

	bool playing_state = true;

	while (app && !app_state::get().should_close_app) // Application still alive?
	{
		app_state::get().should_close_app = glfwWindowShouldClose(app);

		// Wait for the next set of frames from the camera
		if (device.as<rs2::recorder>()) {

		} else {
			rs2::playback playback = device.as<rs2::playback>();
			device.as<rs2::playback>().set_real_time(false);

			uint64_t position = playback.get_position() / 1000000;
			app_state::get().playback_position = position;
			std::chrono::milliseconds duration = std::chrono::milliseconds(playback.get_duration().count());

			app_state::get().playback_duration = duration.count();

			if (playing_state != app_state::get().playing) {
				playback.resume();
				playing_state = !playing_state;
				if (playing_state) {
					printf_h2("Playing");
					playback.resume();
				} else {
					printf_h2("Pause");
					playback.pause();
				}
			}
		}

		if (app_state::get().invalidate_playback) {
			std::cout << "Invalidate playback " << std::endl;

			pipe->stop(); // Stop the pipeline with the default configuration
			pipe = std::make_shared<rs2::pipeline>();

			rs2::config cfg; // Declare a new configuration
			cfg.enable_device_from_file(app_state::get().capture_bag);
			pipe->start(cfg); //File will be opened at this point
			device = pipe->get_active_profile().get_device();
			app_state::get().invalidate_playback = false;
		}

		pcl_ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl_ptr cloud_pipe(new pcl::PointCloud<pcl::PointXYZRGBA>);

		if (pipe->poll_for_frames(&frames)) {
			layers.clear();

			auto depth = frames.get_depth_frame();

			uint32_t frame = frames.get_frame_number();

			rs2::video_frame color = frames.get_color_frame();
			rs2::video_frame infrared = frames.get_infrared_frame();

			// Generate the pointcloud and texture mappings
			// We want the points object to be persistent so we can display the last cloud when a frame drops

			//------------- Point cloud creation ------------------------
			rs2::points points;

			points = pc2.calculate(depth);

			{
				auto sp = points.get_profile().as<rs2::video_stream_profile>();
				cloud_raw->points.clear();
				cloud_raw->width = sp.width();
				cloud_raw->height = sp.height();
				cloud_raw->is_dense = false;

				pc2.map_to(infrared);

				auto size = points.size();
				auto ptr = points.get_vertices();
				auto tex_coords = points.get_texture_coordinates();

				for (i = 0; i < size; i++) {
					pcl::PointXYZRGBA p;

					p.x = ptr->x;
					p.y = -ptr->y;
					p.z = ptr->z;

					//-- Z Clipping max and min values--

					if (p.z > app_state::get().max_clip[2])
						app_state::get().max_clip[2] = p.z;

					if (p.z < app_state::get().min_clip[2])
						app_state::get().min_clip[2] = p.z;

					//-- Y Clipping max and min values--
					if (p.y > app_state::get().max_clip[1])
						app_state::get().max_clip[1] = p.y;

					if (p.y < app_state::get().min_clip[1])
						app_state::get().min_clip[1] = p.y;

					get_texinfrared(&infrared, tex_coords[i].u, tex_coords[i].v, p.a);
					p.r = p.g = p.b = p.a;

					ptr++;
					cloud_raw->points.push_back(p);
				}
			}

			//------------- Color extraction ------------------------

			// All this section is just DEMO Code
			// We should pass the point cloud directly to the pipeline.
			{
				rs2::points points;
				points = pc.calculate(depth);

				pc.map_to(color);

				auto ptr = points.get_vertices();
				auto tex_coords = points.get_texture_coordinates();

				cloud->points.clear();
				cloud_pipe->points.clear();

				i = 0;

				for (auto& p : cloud_raw->points) {
					auto point_raw = p; // TODO: We make several copies of this point here.

					bool add_point = true;
					bool add_point_pipe = true;

					if (tex_coords[i].u < 0 || tex_coords[i].u > 1 || tex_coords[i].v < 0 || tex_coords[i].v > 1) {
						if (!app_state::get().show_ir_only_data) {
							add_point = false;
						}

						add_point_pipe = false;
					} else {
						get_texcolor(&color, tex_coords[i].u, tex_coords[i].v, p.r, p.g, p.b);
						point_raw = p;

						float nvdi = float(p.a - p.r) / (p.a + p.r);

						if (app_state::get().show_ground_raw && app_state::get().show_plants_raw) {
						} else {
							if (app_state::get().show_ground_raw) {
								if (nvdi > app_state::get().cur_nvdi) {
									p.r = p.g = p.b = 0;
									p.z = 0;
									add_point = false;
								}

								if (p.a < (app_state::get().cur_ir * 255.f)) {
									p.g = p.b = 0;
									p.r = p.a;
								}
							}

							if (app_state::get().show_plants_raw) {
								if (nvdi < app_state::get().cur_nvdi) {
									if (app_state::get().bool_tint_nvdi) {
										p.g = p.b = 0;
										p.r = p.a;
									} else {
										add_point = false;
									}
								} else {
									if (p.a < (app_state::get().cur_ir * 255.f)) {
										if (app_state::get().bool_tint_ir) {
											p.g = p.r = 0;
											p.b = p.a;
										} else {
											add_point = false;
										}
									}
								}
							}
						}

						if (nvdi < app_state::get().min_nvdi)
							app_state::get().min_nvdi = nvdi;

						if (nvdi > app_state::get().max_nvdi)
							app_state::get().max_nvdi = nvdi;

					}

					if (add_point && p.z >= 0.01f) {
						cloud->points.push_back(p);
					}

					if (add_point_pipe && p.z >= 0.01f) {
						cloud_pipe->points.push_back(point_raw);
					}

					i++;
					ptr++;
				}

				//--------------- Push cloud ----------------------------
				if (!app_state::get().bool_color_cluster) {
					er_pipe.process_frame(cloud_pipe, worker->get_data_views());
					worker->update_view = true;
				}
			}

			// rs2_frame* frame = rs2_extract_frame(frames, i, &e);
			// float dist_to_center = rs2_depth_frame_get_distance(frame, width / 2, height / 2, &e);

			//------------- Filter limit ------------------------
			if (app_state::get().bool_distance_filter) {
				pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
				pcl::PassThrough<pcl::PointXYZRGBA> pass;
				pass.setInputCloud(cloud);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(app_state::get().cur_min_clip[2], app_state::get().cur_max_clip[2]);
				pass.filter(*cloud_filtered);

				layers.push_back(cloud_filtered);
			} else
				if (app_state::get().bool_cloud_raw) {
					layers.push_back(cloud);
				}

			//------------- VOXEL -----------------------------
			pcl_ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGBA>);

			pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
			sor.setInputCloud(cloud);
			sor.setLeafSize(0.1f, 0.1f, 0.1f);
			sor.filter(*cloud_voxel);

			if (app_state::get().bool_voxel_process) {
				layers.push_back(cloud_voxel);
			}

			//------------- COLOR Cluster ------------------------

			// Creates a pretty picture set of clustered data by color.
			if (app_state::get().bool_color_cluster && cloud->points.size() > 0) {

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
				if (colored_cloud) {
					layers.push_back(colored_cloud);
					printf("TODO!");
				}
			}
		}

		draw_pointcloud(app, app_state, layers);
	}

	return EXIT_SUCCESS;
	} catch (const rs2::error & e) {
		std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
#ifdef WIN32
		MessageBoxA(NULL, ("RealSense error calling!"),
			e.what(), NULL);
#endif
		return EXIT_FAILURE;
	} catch (const std::exception & e) {
		std::cerr << e.what() << std::endl;
#ifdef WIN32
		MessageBoxA(NULL, ("RealSense error calling!"),
			e.what(), NULL);
#endif

		return EXIT_FAILURE;
	}
