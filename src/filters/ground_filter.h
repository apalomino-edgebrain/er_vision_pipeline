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
// Filter to extract the ground and detect the orientation of the floor
//#############################################################################

#ifndef ground_filter_H_
#define ground_filter_H_

#include "../er-pipeline.h"
#include "../algebra/plane.h"

namespace er {
	class ground_filter : public process_unit
	{
	public:
		pcl_ptr cloud_render;

		plane ground_plane;
		Eigen::RowVector3d plane_centre;
		Eigen::MatrixXd V_box;

		Eigen::Vector3d top_left = { 0, 0, 0 };
		Eigen::Vector3d top_right = { 0, 0, 0 };
		Eigen::Vector3d bottom_left = { 0, 0, 0 };
		Eigen::Vector3d bottom_right = { 0, 0, 0 };

		// Here we define the current frame ground transformation for
		// other processes to align their views after their operations.

		bool is_ground_transform;
		Eigen::Transform<double, 3, Eigen::Affine> ground_transform;

		ground_filter() {};
		~ground_filter() {};
		bool process() override;
		void invalidate_view() override;
	};
}

#endif