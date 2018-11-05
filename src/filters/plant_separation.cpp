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

#include <vtkLineSource.h>
#include <vtkTubeFilter.h>
#include <vtkConeSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkSphereSource.h>
#include <vtkDiskSource.h>
#include <vtkPlaneSource.h>
#include <vtkCubeSource.h>

using namespace pcl;

#include "../er-pipeline.h"
#include "../application_state.h"

#include "plant_separation.h"

using namespace er;


plant_processed::plant_processed()
{
//	printf("+ Plant processed\n");
}

//------ Plant discovery & segmentation ------

bool plants_separation_filter::process()
{
	cloud_out->clear();

	/*
	//------------------------------------------------------------------------
	// Find Oriented BBX
	// http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud_in, centroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*cloud_in, centroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
	eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

	// move the points to the that reference frame
	Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
	p2w.block<3, 3>(0, 0) = eigDx.transpose();
	p2w.block<3, 1>(0, 3) = -1.f * (p2w.block<3, 3>(0, 0) * centroid.head<3>());
	pcl::PointCloud<pcl::PointXYZRGBA> cPoints;
	pcl::transformPointCloud(*cloud_in, cPoints, p2w);

	pcl::getMinMax3D(cPoints, min_pt, max_pt);
	const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

	// final transform
	const Eigen::Quaternionf rotation(eigDx);
	const Eigen::Vector3f translation = eigDx*mean_diag + centroid.head<3>();

	double width = max_pt.x - min_pt.x;
	double height = max_pt.y - min_pt.y;
	double depth = max_pt.z - min_pt.z;

	// coefficients = [Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth]

	vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
	t->Identity();
	t->Translate(translation.x(), translation.y(), translation.z());

	Eigen::AngleAxisf a(rotation);
	t->RotateWXYZ(pcl::rad2deg(a.angle()), a.axis()[0], a.axis()[1], a.axis()[2]);

	vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
	cube->SetXLength(width);
	cube->SetYLength(height);
	cube->SetZLength(depth);

	vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	tf->SetTransform(t);
	tf->SetInputConnection(cube->GetOutputPort());
	tf->Update();

	vtkSmartPointer<vtkDataSet> dataset = tf->GetOutput();

	// End Oriented BBX
	//------------------------------------------------------------------------
	*/

	if (app_state::get().use_octreepoint_voxel) {
		float leaf = app_state::get().leaf_X;
		pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGBA>  octree(leaf);
		octree.setInputCloud(cloud_in);
		octree.defineBoundingBox();
		octree.addPointsFromInputCloud();
		pcl::octree::OctreePointCloud<pcl::PointXYZRGBA>::AlignedPointTVector centroids;
		octree.getVoxelCentroids(centroids);

		cloud_out->points.assign(centroids.begin(), centroids.end());
		cloud_out->width = uint32_t(centroids.size());
		cloud_out->height = 1;
		cloud_out->is_dense = true;
	} else
	if (app_state::get().use_voxel_grid) {
		pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(cloud_in);
		sor.setLeafSize(app_state::get().leaf_X,
			app_state::get().leaf_Y, app_state::get().leaf_Z);
		sor.filter(*cloud_out);

		if (cloud_out->points.size() == 0)
			return true;

		std::cout << "points in total Cloud : " << cloud_out->points.size() << std::endl;
		// get the cluster centroids
	} else {
		return true;
	}


	if (view != nullptr)
		view->f_external_render = [&] (void *viewer_ptr) {
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
			view->render_point(viewer_ptr, pos,
				Eigen::Vector3d { 0, 0, 1 }, text);

			view->point_scale = 0.05f;

			plant_processed p;
			p.view_x = pos.x();
			p.view_y = pos.z() / 2;
			p.view_radius = 0;
			plants.push_back(p);
		}
		plants_mutex.unlock();

		Eigen::RowVector3d min_;
		min_ << min_pt.x, min_pt.y, min_pt.z;
		sprintf(text, "MIN (%2.2f, %2.2f, %2.2f)", min_.x(), min_.y(), min_.z());
		view->render_point(viewer_ptr, min_,
			Eigen::Vector3d { 1, 0, 1 }, text);

		Eigen::RowVector3d max_;
		max_ << max_pt.x, max_pt.y, max_pt.z;
		sprintf(text, "MAX (%2.2f, %2.2f, %2.2f)", max_.x(), max_.y(), max_.z());
		view->render_point(viewer_ptr, max_,
			Eigen::Vector3d { 1, 0, 1 }, text);

	};

	return true;
}

void plants_separation_filter::invalidate_view()
{
	if (view != nullptr)
		view->point_scale = er::app_state::get().scale_voxel_grid;
	er::process_unit::invalidate_view();
}


#define BED_SIZE 3

void plants_separation_filter::render_ui()
{
#ifdef USE_IMGUI
	static const int flags = ImGuiWindowFlags_AlwaysAutoResize;

	if (app_state::get().show_voxel_view) {
		ImGui::Begin("Voxel View", &app_state::get().show_voxel_view, flags);

		ImGui::SliderFloat("Point Scale", &app_state::get().scale_voxel_grid, 0.001f, 0.005f);

		ImGui::Checkbox("Show cluster centres", &app_state::get().show_kmeans_cluster);
		ImGui::SliderInt("Cluster size", &app_state::get().cluster_size, 1, 64);

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
		ImGui::SliderFloat("X#leaf", &app_state::get().leaf_X, 0.01f, 1);
		if (!app_state::get().use_octreepoint_voxel) {
			ImGui::SliderFloat("Y#leaf", &app_state::get().leaf_Y, 0.01f, 1);
			ImGui::SliderFloat("Z#leaf", &app_state::get().leaf_Z, 0.01f, 1);
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

		//if (ImGui::Button("Clear"))
		//	points.clear();
	}
	ImGui::End();
#endif
}
