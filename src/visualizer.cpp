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
// Thread to render using libIGL
//#############################################################################

#include "visualizer.h"
#include "visualizer_ui.h"

// Static opengl window where we render our results on libigl
igl::opengl::glfw::Viewer viewer;

// Viewer thread, we try to lock as less as we can.
// This thread contains our UI and displays and should only be locked to update
// the render objects.

er::worker_t viewer_thread;

//#############################################################################
// Pushed point cloud from the realsense Thread
//#############################################################################

er::frame_data::frame_data()
{
	idx = 0;
	initialized = false;
	invalidate = false;
	cloud = pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
}

void er::frame_data::invalidate_cloud(pcl_ptr cloud_)
{
	boost::lock_guard<frame_data> guard(*this);
	invalidate = true;
	pcl::copyPointCloud(*cloud_, *cloud);
}

void er::worker_t::setup()
{
	bthread = new boost::thread(boost::bind(&er::worker_t::start, this));
}

void er::worker_t::add_axis()
{
	Eigen::MatrixXd V_axis(4, 3);
	V_axis <<
		0, 0, 0,
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	Eigen::MatrixXi E_axis(3, 2);
	E_axis <<
		0, 1,
		0, 2,
		0, 3;

	Eigen::VectorXd radius(V_axis.rows());
	radius.setConstant(er::app_state::get().point_scale * 10 * viewer.core.camera_base_zoom);

	// Plot the corners of the bounding box as points
	viewer.data().add_points(V_axis, Eigen::RowVector3d(1, 0, 0), radius);

	// Plot labels with the coordinates of bounding box vertices
	Eigen::Vector3d x(1, 0, 0);
	viewer.data().add_label(x, "x");

	Eigen::Vector3d y(0, 1, 0);
	viewer.data().add_label(y, "y");

	Eigen::Vector3d z(0, 0, 1);
	viewer.data().add_label(z, "z");

	Eigen::Vector3d centre(0, 0, 0);
	viewer.data().add_label(centre, "centre");
}

void er::worker_t::compute_cloud()
{
	if (!initialized)
		return;

	if (!data.invalidate)
		return;

	boost::lock_guard<frame_data> guard(data);
	size_t size = data.cloud->points.size();

	printf("Compute cloud %zd\n", size); \

	V.resize(size, 3);
	C.resize(size, 3);

	int i = 0;
	for (auto& p : data.cloud->points) {

		//printf(" %2.2f, %2.2f, %2.2f ", p.x, p.y, p.z);
		V(i, 0) = p.x;
		V(i, 1) = -p.y;
		V(i, 2) = p.z;

		C(i, 0) = p.r / 255.f;
		C(i, 1) = p.g / 255.f;
		C(i, 2) = p.b / 255.f;
		i++;
	}

	Eigen::VectorXd radius(size);
	radius.setConstant(er::app_state::get().point_scale * viewer.core.camera_base_zoom);

	viewer.data().set_points(V, C, radius);
}

void er::worker_t::start()
{
	printf("Start thread\n");

	// Load an example mesh in OFF format
	// Just to check that we are have a functional libIGL.

	// TODO: This thread will crash if the window is not created on time.
	boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

	//------------------------------------------------------------
	igl::readOFF("S:/libigl/tutorial/shared/grid.off", V, F);

	Eigen::VectorXd radius(V.rows());
	radius.setConstant(0.01 * viewer.core.camera_base_zoom);

	Eigen::VectorXd Z = V.col(2);
	igl::jet(Z, true, C);

	viewer.data().set_points(V, C, radius);

	initialize_visualizer_ui(viewer);

	add_axis();
	viewer.append_mesh();

	viewer.callback_init = [&] (auto viewer_) {
		printf("Setting up camera!\n");

		// Position the camera in our reference frame
		viewer.core.camera_eye = Eigen::Vector3f( 0, 0, -2.5f);

		return false;
	};

	// Thread were we check to see if we have to invalidate the pointcloud
	viewer.callback_post_draw = [&] (auto viewer) {
		initialized = true;

		// Position the camera in our reference frame
		compute_cloud();
		return false;
	};

	viewer.launch();
}

// Lock the data access when we push the cloud.
// On this thread we will copy as fast as we can the pcl frame
// and return.
void er::worker_t::push_cloud(pcl_ptr cloud)
{
	data.invalidate_cloud(cloud);
}

er::worker_t *launch_visualizer()
{
	viewer_thread.setup();
	return &viewer_thread;
}
