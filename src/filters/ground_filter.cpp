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

// Reduce floor cloud density with a voxelgrid
// Raytrace points from floor grid up to find the right position in space
// Classify between floor and plants

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

		float nvdi = float(p.a - p.r) / (p.a + p.r);

		if (nvdi > NVDI) {
			add_point = false;
		}

		if (add_point && p.z >= 0.01f)
			cloud_out->points.push_back(p);
	}

	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(cloud_out);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, MAX_Z);
	pass.filter(*cloud_out);

	if (view != nullptr) {
		view->invalidate_cloud(cloud_out);
		if (view->V.size() > 0) {
			auto bbx_m = view->V.colwise().minCoeff();
			auto bbx_M = view->V.colwise().maxCoeff();

			//---------- Center in plane --------------------------------------
			Tform3 T;
			T = Tform3::Identity();
			T.translate(Vec3(-(bbx_m(0) + bbx_M(0)) / 2,
				-(bbx_m(1)),
				-(bbx_m(2)) / 2));

// TODO: https://stackoverflow.com/questions/38841606/shorter-way-to-apply-transform-to-matrix-containing-vectors-in-eigen
// Find how to do this properly

			for (int r = 0; r < view->V.rows(); r++) {
				Eigen::Vector3d v = view->V.row(r);
				v = T * v;
				view->V.row(r) = v;
			}

			view->bbx_m = view->V.colwise().minCoeff();
			view->bbx_M = view->V.colwise().maxCoeff();

			#define Z_TEST 2.5
			double y_pos = ground_plane.get_y(0, Z_TEST);
			double angleInRadians = std::atan2(y_pos, Z_TEST);

			Eigen::RowVector3d N;
			igl::fit_plane(view->V, N, plane_centre);

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

			// Plane
			// Ax + By + Cz = D;
			ground_plane = plane::fromPointNormal(plane_centre, N);

			view->f_external_render = [&] (void *viewer_ptr) {
				if (er::app_state::get().show_ground_plane) {
					view->render_point(viewer_ptr, plane_centre,
						Eigen::Vector3d { 1, 1,1 }, "Ground Centre");

					view->render_plane(viewer_ptr, ground_plane,
						view->bbx_m, view->bbx_M);

					Eigen::RowVector3d pos;

					#define Z_POS 2.5
					double y_pos = ground_plane.get_y(0, Z_POS);
					pos << 0, y_pos, Z_POS;

					double angleInRadians = std::atan2(y_pos, Z_POS);
					double angleInDegrees = (angleInRadians / M_PI) * 180.0;

					char text[256];
					sprintf(text, "Angle [%2.2f]", angleInDegrees);

					view->render_point(viewer_ptr, pos,
						Eigen::Vector3d{ 1, 1, 0 }, text);
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
