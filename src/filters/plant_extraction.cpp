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

#include "../er-pipeline.h"
#include "../application_state.h"
#include <igl/fit_plane.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include "ground_filter.h"
#include "plant_extraction.h"

using namespace er;

#define NVDI -0.345f
#define IR (0.361f * 255.f)
#define MAX_LUMINOSITY (150.0f*3.0f)
#define MIN_Z 0.2f

void plants_filter::input_pcl(pcl_ptr cloud)
{
	er::process_unit::input_pcl(cloud);
}

//------ Plant discovery & Extraction ------
// After we align the floor

bool plants_filter::process()
{
	printf(" Plants_filter: %p\n", this);
	if (cloud_out == nullptr) {
		pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		cloud_out = cloud;
	} else {
		// TODO: Do not clear and reuse points
		cloud_out->clear();
	}

	Eigen::Transform<double, 3, Eigen::Affine>
		ground_transform = grnd_filter->ground_transform;

/*  In order to change keep orientation working we would have to render

	pcl_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_in, *transformed_cloud,
		ground_transform.matrix());
*/
	for (auto& p : cloud_in->points) {
		bool add_point = true;

		if (p.r + p.g + p.b < MAX_LUMINOSITY) {
			add_point = false;
		} else
		if (p.a < IR) {
			add_point = false;
		} else {
			float nvdi = float(p.a - p.r) / (p.a + p.r);
			if (nvdi < NVDI) {
				add_point = false;
			}
		}

		if (add_point && p.z >= MIN_Z)
			cloud_out->points.push_back(p);
	}

	if (view != nullptr) {
		view->invalidate_cloud(cloud_out);

		if (view->V.size() > 0) {
			if (er::app_state::get().ground_alignment) {
				for (int r = 0; r < view->V.rows(); r++) {
					Eigen::Vector3d v = view->V.row(r);
					v = ground_transform * v;
					view->V.row(r) = v;
				}
			}
		}
	}
	return true;
}

void plants_filter::set_ground_filter(ground_filter *grnd_filter_)
{
	grnd_filter = grnd_filter_;
}

void plants_filter::invalidate_view()
{

}
