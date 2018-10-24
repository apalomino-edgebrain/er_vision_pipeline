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
// Ground extraction process_unit
//#############################################################################
//
// See er-pipeline.h to get more information on the functionality and structure
//

#include <limits>

#include "../er-pipeline.h"
#include "../application_state.h"
#include <igl/fit_plane.h>

#include "ground_filter.h"

using namespace er;

#define NVDI -0.345f
#define IR (0.361f * 255.f)
#define MAX_LUMINOSITY (150.0f*3.0f)

#define MAX_Z 3

//------ BASIC FLOOR DISCOVERY & MAPPING ------
// Create floor grid
//  Width x Height

// Algorithm of floor removal and floor discovery
// Split in two clusters
//		1. Some green + and nvdi = Plant
//		2. No color, just ground
//
// Find the plane defined by the ground plane
//
// Adjust the plane to fit the current view using the Minimun coefficients
// Find the angle X to align the plane on that axis
// Find the angle Y to align the plane on that axis
//
// Create a transformation matrix to fit orientate the plane
// Transfor every position to fit the new axis base.

// Next steps:
//	Reduce floor cloud density with a voxelgrid
//  Raytrace points from floor grid up to find the right position in space
//  Classify between floor and plants

/*
void find_boundaries(Eigen::MatrixXd &V,
	Eigen::Vector3d &top_left,
	Eigen::Vector3d &top_right,
	Eigen::Vector3d &bottom_left,
	Eigen::Vector3d &bottom_right)
{
	Eigen::Vector3d bbx_m;
	Eigen::Vector3d bbx_M;

	bbx_m = V.colwise().minCoeff();
	bbx_M = V.colwise().maxCoeff();

	bottom_left = { bbx_M(0), bbx_M(1), bbx_M(2) };
	for (int r = 0; r < V.rows(); r++) {
		Eigen::Vector3d v = V.row(r);

		if (v.y() < bottom_left.y()) {
			bottom_left = v;
		} else
		if (v.y() == bottom_left.y()) {
			if (v.x() < bottom_left.x()) {
				bottom_left = v;
			}
		}
	}
}
*/

bool ground_filter::process()
{
	if (cloud_out == nullptr) {
		pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		cloud_out = cloud;
	} else {
		// TODO: Do not clear and reuse points
		cloud_out->clear();
	}

	Eigen::RowVector3d cp;

	for (auto& p : cloud_in->points) {
		bool add_point = true;

		if (p.r + p.g + p.b > MAX_LUMINOSITY) {
			add_point = false;
		} else
		if (p.a > IR) {
			add_point = false;
		} else {
			float nvdi = float(p.a - p.r) / (p.a + p.r);

			if (nvdi > NVDI) {
				add_point = false;
			}
		}

		if (add_point && p.z >= 0.01f)
			cloud_out->points.push_back(p);
	}

	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(cloud_out);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, MAX_Z);
	pass.filter(*cloud_out);

#define Z_POS 2.5

	if (view != nullptr) {
		view->invalidate_cloud(cloud_out);
		if (view->V.size() > 0) {
			Eigen::Vector3d bbx_m;
			Eigen::Vector3d bbx_M;

			Eigen::RowVector3d N;
			igl::fit_plane(view->V, N, plane_centre);

			ground_plane = plane::fromPointNormal(plane_centre, N);

			//-------------------------------------------------------------
			// https://en.wikipedia.org/wiki/Direction_cosine
			// cos A = ax/|a|;   	cos B = ay/|a|;   	cos Y = az/|a|

			plane new_plane = plane::fromPointNormal(plane_centre, N);

			float dot = std::sqrt(ground_plane.a * ground_plane.a +
				ground_plane.b * ground_plane.b +
				ground_plane.c * ground_plane.c);

			float cosa = std::acos(ground_plane.a / dot);
			cosa = - (cosa - M_PI / 2.0f);

			float cosb = std::acos(ground_plane.b / dot);
			if (cosb > M_PI / 2.0f)
				cosb = M_PI - cosb;

			float cosy = std::acos(ground_plane.c / dot);
			if (cosy > M_PI / 2.0f)
				cosy = M_PI - cosb;

			if (er::app_state::get().bool_debug_verbose) {
				std::cout << "-------------" << std::endl;
				std::cout << "cosa = " << cosa * (360 / (2 * M_PI)) << std::endl;
				std::cout << "cosb = " << cosb * (360 / (2 * M_PI)) << std::endl;
				std::cout << "cosy = " << cosy * (360 / (2 * M_PI)) << std::endl;
			}

			if (er::app_state::get().bool_override_rotation) {
				cosa = er::app_state::get().rot_x;
				cosb = er::app_state::get().rot_y;
				cosy = er::app_state::get().rot_z;
			} else {
				er::app_state::get().rot_x = cosa;
				er::app_state::get().rot_y = cosb;
				er::app_state::get().rot_z = cosy;
			}

			Tform3 X_Rot = Eigen::Affine3d(Eigen::AngleAxisd(cosa, Eigen::Vector3d::UnitZ()));
			Tform3 Y_Rot = Eigen::Affine3d(Eigen::AngleAxisd(cosb, Eigen::Vector3d::UnitX()));
			Tform3 Z_Rot = Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));

			Tform3 T = Tform3::Identity();
			Tform3 T2 = Tform3::Identity();

			if (er::app_state::get().bool_traslate) {
				T.translate(Vec3(-plane_centre(0), -plane_centre(1), -plane_centre(2)));
				T2.translate(Vec3(0, 0, plane_centre(2) / 2));
			}

			//-------------------------------------------------------------

			// TODO: https://stackoverflow.com/questions/38841606/shorter-way-to-apply-transform-to-matrix-containing-vectors-in-eigen
			// Find how to do this properly

			if (er::app_state::get().ground_alignment) {

				Tform3 F = T2 * Y_Rot * X_Rot * Z_Rot * T;
				for (int r = 0; r < view->V.rows(); r++) {
					Eigen::Vector3d v = view->V.row(r);
					v = F * v;
					view->V.row(r) = v;
				}

				bbx_m = view->V.colwise().minCoeff();
				bbx_M = view->V.colwise().maxCoeff();

			} else {

			}

			view->bbx_m = bbx_m = view->V.colwise().minCoeff();
			view->bbx_M = bbx_M = view->V.colwise().maxCoeff();

			//-----------------------------------------------------------------

			V_box.resize(8, 3);
			V_box <<
				bbx_m(0), bbx_m(1), bbx_m(2),
				bbx_M(0), bbx_m(1), bbx_m(2),
				bbx_M(0), bbx_M(1), bbx_m(2),
				bbx_m(0), bbx_M(1), bbx_m(2),
				bbx_m(0), bbx_m(1), bbx_M(2),
				bbx_M(0), bbx_m(1), bbx_M(2),
				bbx_M(0), bbx_M(1), bbx_M(2),
				bbx_m(0), bbx_M(1), bbx_M(2);

			view->visible = er::app_state::get().show_ground;

			view->f_external_render = [&] (void *viewer_ptr) {
				char text[256];

				if (er::app_state::get().show_ground_plane) {
					view->render_point(viewer_ptr, plane_centre,
						Eigen::Vector3d { 1, 1, 1 }, "Ground Centre");

					//---------------------------------------------------------
					// Intersection with ground at 0 position to create a vector
					Eigen::RowVector3d pos_intersection;

					double z_pos = ground_plane.get_z(0, 0);
					pos_intersection << 0, 0, z_pos;

					sprintf(text, "Intersection [%2.2f]", z_pos);
					view->render_point(viewer_ptr, pos_intersection,
						Eigen::Vector3d { 0, 1, 0 }, text);

					//---------------------------------------------------------

					Eigen::RowVector3d pos;
					double y_pos = ground_plane.get_y(0, Z_POS);
					pos << 0, y_pos, Z_POS;

					double angleInRadians = std::atan2(y_pos, Z_POS - z_pos);
					double angleInDegrees = (angleInRadians / M_PI) * 180.0;

					sprintf(text, "Angle [%2.2f]", angleInDegrees);
					view->render_point(viewer_ptr, pos,
						Eigen::Vector3d { 1, 1, 0 }, text);

					view->render_plane(viewer_ptr, ground_plane,
						view->bbx_m, view->bbx_M);

				}
			};
		}
	}
	return true;
}

void ground_filter::invalidate_view()
{
	if (!visible)
		return;

	if (cloud_out == nullptr || view == nullptr)
		return;
}
