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
// Plant extraction process_unit
//#############################################################################
//
// See er-pipeline.h to get more information on the functionality and structure
//

#include <limits>

#include <pcl/visualization/common/shapes.h>
#include <pcl/common/angles.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/impl/morphological_filter.hpp>

#include <vtkLineSource.h>
#include <vtkTubeFilter.h>
#include <vtkConeSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkSphereSource.h>
#include <vtkDiskSource.h>
#include <vtkPlaneSource.h>
#include <vtkCubeSource.h>

#ifdef USE_IMGUI
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#endif

#define MAX_COLORS 15
static const float colors[3 * MAX_COLORS] = {
	255,255,255,  // WHITE
	255, 0, 0, // RED
	0, 255, 0, // Lime
	0, 0, 255, // Blue
	255, 255, 0, // Yellow
	0, 255, 255, // Cyan
	255, 0, 255, // Magenta
	192, 192, 192, // Silver
	128, 128, 128, // Gray
	128, 0, 0, // Maroon
	128, 128, 0, // Olive
	0, 128, 0, // Green
	128, 0, 128, // Purple
	0, 128, 128, // Teal
	0, 0, 128 // Navy
};

using namespace pcl;

#include "../er-pipeline.h"
#include "../application_state.h"

#include "plant_segmentation.h"

using namespace er;

//------ Definition of a detected plant ------

plant_definition::plant_definition()
{
//	printf("+ Plant processed\n");
}

//------ Plant discovery & segmentation ------

plants_segmentation_filter::plants_segmentation_filter() : tex_id(0)
{
	size_w = app_state::get().floor_project_x;
	size_h = app_state::get().floor_project_y;

	if (size_w == 0) {
		size_w = FLOOR_SIZE_W;
		app_state::get().floor_project_x = size_w;
	}

	if (size_h == 0) {
		size_h = FLOOR_SIZE_H;
		app_state::get().floor_project_y = size_h;
	}

	uint32_t len = size_w * size_h;
	floor_projection = new float[len];

	// Raw texture to render in OpenGL for debugging purposes
	tex_floor_rgba = new uint32_t[len];

	memset(tex_floor_rgba, 0x11, sizeof(uint32_t) * len);

	// This is our cloud filtering to be reused after processing
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud_filtered = cloud;

	// Temporal cloud to store our voxel data
	pcl_ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud_voxel = cloud_temp;
}

plants_segmentation_filter::~plants_segmentation_filter()
{
	delete floor_projection;
}

bool plants_segmentation_filter::process()
{
	if (cloud_in->points.size() == 0) {
		printf("! plants_segmentation_filter::process() No point cloud\n");
		return true;
	}

	printf(" plants_segmentation_filter::process() \n");
	cloud_out->clear();
	cloud_voxel->clear();
	cloud_filtered->clear();

	/*
	template <typename PointT> void
		pcl::applyMorphologicalOperator(const typename pcl::PointCloud<PointT>::ConstPtr &cloud_in,
			float resolution, const int morphological_operator,
			pcl::PointCloud<PointT> &cloud_out)
	*/

	if (app_state::get().use_octreepoint_voxel) {
		float leaf = app_state::get().leaf_X;
		pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGBA>  octree(leaf);
		octree.setInputCloud(cloud_in);
		octree.defineBoundingBox();
		octree.addPointsFromInputCloud();
		pcl::octree::OctreePointCloud<pcl::PointXYZRGBA>::AlignedPointTVector centroids;
		octree.getVoxelCentroids(centroids);

		cloud_voxel->points.assign(centroids.begin(), centroids.end());
		cloud_voxel->width = uint32_t(centroids.size());
		cloud_voxel->height = 1;
		cloud_voxel->is_dense = true;
	} else
		if (app_state::get().use_voxel_grid) {
			pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
			sor.setInputCloud(cloud_in);
			sor.setLeafSize(app_state::get().leaf_X,
				app_state::get().leaf_Y, app_state::get().leaf_Z);
			sor.filter(*cloud_voxel);

			if (cloud_voxel->points.size() == 0)
				return true;

			std::cout << "points in total Cloud : " << cloud_voxel->points.size() << std::endl;
			// get the cluster centroids
		} else {
			printf(" No cloud to process!");
			return true;
		}

	if (app_state::get().use_erosion) {
		pcl::applyMorphologicalOperator<pcl::PointXYZRGBA>(cloud_in,
			app_state::get().erosion_resolution, MORPH_ERODE, *cloud_out);
	}

	uint32_t len = size_w * size_h;
	memset(tex_floor_rgba, 0x0, sizeof(uint32_t) * len);
	memset(floor_projection, 0x0, sizeof(float) * len);

	for (size_t i = 0; i < cloud_in->points.size(); i++) {
		float x = cloud_in->points[i].x * 0.5f;
		float y = cloud_in->points[i].y;
		float z = (cloud_in->points[i].z);

		int pos_x = size_w / 2 - x * size_w;
		int pos_y = size_h - z * size_h;

		int pos = pos_x + pos_y * size_w;
		if (pos > 0 && pos < len) {
			floor_projection[pos] += y;
		}
	}

	// Find the moving average and find the points where we have the maximum
	// Run a correlation filtering on the image.

	int pos = 0;
	for (uint32_t y = 0; y < size_h; y++) {
		for (uint32_t x = 0; x < size_w; x++) {
			if (floor_projection[pos + x] > 0) {
				tex_floor_rgba[pos + x] = 0xFFFFFFFF;
			}
		}

		pos += size_w;
	}

	for (size_t j = 0; j < MAX_PLANTS; j++) {
		plants_seg[j].visible = false;
	}

	if (app_state::get().show_euclidian_cluster) {
		pcl::copyPointCloud(*cloud_voxel, *cloud_filtered);

		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
		tree->setInputCloud(cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
		ec.setClusterTolerance(app_state::get().cluster_tolerance);

		ec.setMinClusterSize(app_state::get().min_cluster_points);
		ec.setMaxClusterSize(app_state::get().max_cluster_points);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_filtered);
		ec.extract(cluster_indices);

		plants_mutex.lock();
		plants.clear();

		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
				cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			printf("%d: Found cluster %d \n", j, cloud_cluster->width);

			if (j < MAX_PLANTS) {
				// TODO: Replace this function with computeNDCentroid
				CentroidPoint<pcl::PointXYZRGBA> centroid;
				for (int t = 0; t < cloud_cluster->width; t++) {
					centroid.add(cloud_cluster->points[t]);

					// Get a color from our fancy color table
					int c = (j % MAX_COLORS) * 3;
					cloud_cluster->points[t].r = colors[c + 0];
					cloud_cluster->points[t].g = colors[c + 1];
					cloud_cluster->points[t].b = colors[c + 2];
				}

				pcl::PointXYZRGBA c1;
				centroid.get(c1);

				view.point_scale = 0.05f;
				plant_definition p;
				p.view_x = - c1.x;
				p.view_y = 0.7 - c1.z / 2;
				p.view_radius = 0;
				plants.push_back(p);

				plants_seg[j].invalidate_cloud(cloud_cluster);
				plants_seg[j].visible = true;
			}

			j++;
		}

		plants_mutex.unlock();
	}

	view.f_external_render = [&] (void *viewer_ptr) {
		if (app_state::get().show_euclidian_cluster) {
			// Pass the views to the subrenderer
			if (view.sub_views.size() == 0) {
				for (size_t t = 0; t < MAX_PLANTS; t++)
					view.sub_views.push_back(&plants_seg[t]);
			}
			return;
		}

		if (app_state::get().show_voxel_data) {
			pcl::copyPointCloud(*cloud_voxel, *cloud_out);
		}

#ifdef USE_PCL_1_8_0
		// Kmeans clustering gives us pretty bad results.
		// We might be able to improve the results at some point

		if (!app_state::get().show_kmeans_cluster)
			return;

		char text[512];

		Eigen::RowVector3d pos;
		pcl::Kmeans real(static_cast<int> (cloud_out->points.size()), 3);

		real.setClusterSize(app_state::get().cluster_size); //it is important that you set this term appropriately for your application
		for (size_t i = 0; i < cloud_out->points.size(); i++) {
			std::vector<float> data(3);
			data[0] = cloud_out->points[i].x;
			data[1] = cloud_out->points[i].y;
			data[2] = cloud_out->points[i].z;
			real.addDataPoint(data);
		}

		real.kMeans();

		pcl::Kmeans::Centroids centroids = real.get_centroids();

		std::cout << "centroid count: " << centroids.size() << std::endl;

		plants_mutex.lock();
		plants.clear();
		for (int i = 0; i < centroids.size(); i++) {
			pos << centroids[i][0], centroids[i][1], centroids[i][2];

			sprintf(text, "(%2.2f, %2.2f, %2.2f)", pos.x(), pos.y(), pos.z());
			view.render_point(viewer_ptr, pos,
				Eigen::Vector3d { 0, 0, 1 }, text);

			view.point_scale = 0.05f;

			plant_definition p;
			p.view_x = pos.x();
			p.view_y = pos.z() / 2;
			p.view_radius = 0;
			plants.push_back(p);
		}
		plants_mutex.unlock();

		Eigen::RowVector3d min_;
		min_ << min_pt.x, min_pt.y, min_pt.z;
		sprintf(text, "MIN (%2.2f, %2.2f, %2.2f)", min_.x(), min_.y(), min_.z());
		view.render_point(viewer_ptr, min_,
			Eigen::Vector3d { 1, 0, 1 }, text);

		Eigen::RowVector3d max_;
		max_ << max_pt.x, max_pt.y, max_pt.z;
		sprintf(text, "MAX (%2.2f, %2.2f, %2.2f)", max_.x(), max_.y(), max_.z());
		view.render_point(viewer_ptr, max_,
			Eigen::Vector3d { 1, 0, 1 }, text);
#endif
	};

	return true;
}

void plants_segmentation_filter::invalidate_view()
{
	view.point_scale = er::app_state::get().scale_voxel_grid;
	er::process_unit::invalidate_view();
}

#define BED_SIZE 3

void plants_segmentation_filter::render_ui()
{
#ifdef USE_IMGUI
	static const int flags = ImGuiWindowFlags_AlwaysAutoResize;

	if (app_state::get().show_image_view) {
#ifdef WIN32
       // It seems that we have an issue in Linux to use raw GL.
       // A possible option is that we haven't initialized the GL functions
       // which are normally wrappers and dynamically loaded.

		if (tex_id == 0) {
			glGenTextures(1, &tex_id);
		}

		glBindTexture(GL_TEXTURE_2D, tex_id);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

		glTexImage2D(GL_TEXTURE_2D,     // Type of texture
			0,							// Pyramid level (for mip-mapping) - 0 is the top level
			GL_RGBA,					// Internal colour format to convert to
			size_w, size_h,				// texture size
			0,							// Border width in pixels (can either be 1 or 0)
			GL_RGBA,					// Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
			GL_UNSIGNED_BYTE,			// Image data type
			tex_floor_rgba);			// The actual image data itself

		ImGui::Begin("2D View", &app_state::get().show_image_view, flags);
		ImGui::TextWrapped("Show 2.5d image");

		ImGui::Image((ImTextureID) tex_id, ImVec2(size_w / 2, size_h / 2));

		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();

			ImGuiIO& io = ImGui::GetIO();

			ImVec2 pos = ImGui::GetCursorScreenPos();

			float region_sz = 32.0f;
			float region_x = io.MousePos.x - pos.x - region_sz * 0.5f; if (region_x < 0.0f) region_x = 0.0f; else if (region_x > size_w - region_sz) region_x = size_w - region_sz;
			float region_y = io.MousePos.y - pos.y - region_sz * 0.5f; if (region_y < 0.0f) region_y = 0.0f; else if (region_y > size_h - region_sz) region_y = size_h - region_sz;
			float zoom = 4.0f;
			ImGui::Text("Min: (%.2f, %.2f)", region_x, region_y);
			ImGui::Text("Max: (%.2f, %.2f)", region_x + region_sz, region_y + region_sz);
			ImVec2 uv0 = ImVec2((region_x) / size_w, (region_y) / size_h);
			ImVec2 uv1 = ImVec2((region_x + region_sz) / size_w, (region_y + region_sz) / size_h);
			ImGui::Image((ImTextureID) tex_id, ImVec2(region_sz * zoom, region_sz * zoom), uv0, uv1, ImColor(255, 255, 255, 255), ImColor(255, 255, 255, 128));
			ImGui::EndTooltip();
		}
		ImGui::End();
#endif
	}

	if (app_state::get().show_voxel_view) {
		ImGui::Begin("Voxel View", &app_state::get().show_voxel_view, flags);
		ImGui::SliderFloat("Point Scale", &app_state::get().scale_voxel_grid, 0.001f, 0.005f);

		ImGui::Checkbox("Show Voxel data", &app_state::get().show_voxel_data);
		ImGui::Separator();

		ImGui::Checkbox("Show Euclidian cluster", &app_state::get().show_euclidian_cluster);
		ImGui::SliderFloat("Cluster Tolerance in m", &app_state::get().cluster_tolerance, 0.01f, 1.0f);
		ImGui::SliderInt("Min cluster points", &app_state::get().min_cluster_points, 25, 10000);
		ImGui::SliderInt("Max cluster points", &app_state::get().max_cluster_points, 25, 10000);
		ImGui::Separator();

		ImGui::Checkbox("Show Kmeans clusters", &app_state::get().show_kmeans_cluster);
		ImGui::SliderInt("Cluster size", &app_state::get().cluster_size, 1, 64);

		if (ImGui::Checkbox("Use Erosion", &app_state::get().use_erosion)) {
			//app_state::get().use_octreepoint_voxel = false;
			//app_state::get().use_voxel_grid = false;
		}

		ImGui::Text("Erosion Value");
		ImGui::SliderFloat("Erosion", &app_state::get().erosion_resolution, 0.01f, 1);

		ImGui::Separator();
		if (ImGui::Checkbox("Octree Voxel", &app_state::get().use_octreepoint_voxel)) {
			app_state::get().use_octreepoint_voxel = true;
			app_state::get().use_voxel_grid = false;
		}

		ImGui::SameLine(120);
		if (ImGui::Checkbox("Voxel Grid", &app_state::get().use_voxel_grid)) {
			app_state::get().use_octreepoint_voxel = false;
			app_state::get().use_voxel_grid = true;
		}

		ImGui::Text("Leaf Size");
		ImGui::SliderFloat("X#leaf", &app_state::get().leaf_X, 0.001f, 0.5f);
		if (!app_state::get().use_octreepoint_voxel) {
			ImGui::SliderFloat("Y#leaf", &app_state::get().leaf_Y, 0.001f, 0.5f);
			ImGui::SliderFloat("Z#leaf", &app_state::get().leaf_Z, 0.001f, 0.5f);
		}

		ImGui::End();
	}

	if (!er::app_state::get().show_plant_ui_window)
		return;

	ImGui::SetNextWindowSize(ImVec2(350, 560), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Plant Evaluation", &er::app_state::get().show_plant_ui_window)) {
		ImGui::End();
		return;
	}

	ImDrawList* draw_list = ImGui::GetWindowDrawList();

	static float sz = 36.0f;
	static float thickness = 4.0f;
	static ImVec4 col = ImVec4(1.0f, 1.0f, 0.4f, 1.0f);
	const ImU32 col32 = ImColor(col);

	{
		static bool adding_line = false;
		ImGui::Text("Plant canvas");

		ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
		ImVec2 canvas_size = ImGui::GetContentRegionAvail();
		if (canvas_size.x < 50.0f) canvas_size.x = 50.0f;
		if (canvas_size.y < 50.0f) canvas_size.y = 50.0f;
		draw_list->AddRectFilledMultiColor(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(50, 50, 50, 255), IM_COL32(50, 50, 60, 255), IM_COL32(60, 60, 70, 255), IM_COL32(50, 50, 60, 255));
		draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(255, 255, 255, 255));

		ImGui::InvisibleButton("canvas", canvas_size);
		ImVec2 mouse_pos_in_canvas = ImVec2(ImGui::GetIO().MousePos.x - canvas_pos.x, ImGui::GetIO().MousePos.y - canvas_pos.y);

		draw_list->PushClipRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), true);      // clip lines within the canvas (if we resize it, etc.)

		plants_mutex.lock();
		for (auto &plant : plants) {
			float pos_x = canvas_pos.x + sz*0.5f + (plant.view_x * canvas_size.x / 2) + canvas_size.x / 2;
			float pos_y = canvas_pos.y + sz*0.5f + (plant.view_y * canvas_size.y);
			float pos_radius = sz*0.5f + plant.view_radius;
			draw_list->AddCircle(ImVec2(pos_x, pos_y), pos_radius, col32, 20, thickness);
		}
		plants_mutex.unlock();

		draw_list->PopClipRect();
	}
	ImGui::End();
#endif
}
