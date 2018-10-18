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

#include <igl/fit_plane.h>

#include "algebra/plane.h"

// Static opengl window where we render our results on libigl
igl::opengl::glfw::Viewer viewer;

// Viewer thread, we try to lock as less as we can.
// This thread contains our UI and displays and should only be locked to update
// the render objects.

er::worker_t viewer_thread;

//#############################################################################
// Utils
//#############################################################################

// Renders a plane in space
void render_plane(igl::opengl::glfw::Viewer *viewer,
	plane &p, Eigen::Vector3d &m, Eigen::Vector3d M)
{
	Eigen::MatrixXd V(4, 3);
	V <<
		m(0), p.get_y(m(0), m(2)), m(2),
		M(0), p.get_y(M(0), m(2)), m(2),
		m(0), p.get_y(m(0), M(2)), M(2),
		M(0), p.get_y(M(0), M(2)), M(2);

	Eigen::MatrixXi F(2, 3);
	F <<
		0, 1, 2,
		1, 2, 3;

	Eigen::MatrixXd C;
	C.resize(4, 3);

	Eigen::VectorXd Z = V.col(2);
	igl::jet(Z, true, C);

	viewer->data().set_mesh(V, F);
	viewer->data().set_colors(C);

	viewer->append_mesh();
}

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

void er::frame_data::calculate_view()
{
	std::lock_guard<std::mutex> lock(mtx);
	size_t size = cloud->points.size();

	V.resize(size, 3);
	C.resize(size, 3);

	int i = 0;
	for (auto& p : cloud->points) {

		//printf(" %2.2f, %2.2f, %2.2f ", p.x, p.y, p.z);
		V(i, 0) = p.x;
		V(i, 1) = -p.y;
		V(i, 2) = p.z;

		C(i, 0) = p.r / 255.f;
		C(i, 1) = p.g / 255.f;
		C(i, 2) = p.b / 255.f;
		i++;
	}

	radius.resize(size);
	radius.setConstant(er::app_state::get().point_scale);

	bbx_m = V.colwise().minCoeff();
	bbx_M = V.colwise().maxCoeff();
}

void er::frame_data::render(void *viewer_ptr)
{
	std::lock_guard<std::mutex> lock(mtx);
	igl::opengl::glfw::Viewer *viewer = (igl::opengl::glfw::Viewer *) viewer_ptr;

	viewer->data().set_points(V, C, radius);
	id_mesh = viewer->data().id;
	viewer->append_mesh();

	//-------------------------------------------------------------------------
	// Find the bounding box
	if (er::app_state::get().show_bbx) {

		// Corners of the bounding box
		Eigen::MatrixXd V_box(8, 3);
		V_box <<
			bbx_m(0), bbx_m(1), bbx_m(2),
			bbx_M(0), bbx_m(1), bbx_m(2),
			bbx_M(0), bbx_M(1), bbx_m(2),
			bbx_m(0), bbx_M(1), bbx_m(2),
			bbx_m(0), bbx_m(1), bbx_M(2),
			bbx_M(0), bbx_m(1), bbx_M(2),
			bbx_M(0), bbx_M(1), bbx_M(2),
			bbx_m(0), bbx_M(1), bbx_M(2);

		// Edges of the bounding box
		Eigen::MatrixXi E_box(12, 2);
		E_box << 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 7, 3;

		Eigen::VectorXd radius_(V_box.rows());
		radius.setConstant(er::app_state::get().point_scale);

		// Plot labels with the coordinates of bounding box vertices
		std::stringstream l1;
		l1 << bbx_m(0) << ", " << bbx_m(1) << ", " << bbx_m(2);
		viewer->data().add_label(bbx_m, l1.str());
		std::stringstream l2;
		l2 << bbx_M(0) << ", " << bbx_M(1) << ", " << bbx_M(2);
		viewer->data().add_label(bbx_M, l2.str());
	}

	//-------------------------------------------------------------------------
	// Plane fitting
	if (er::app_state::get().show_ground_plane) {
		Eigen::Vector3d m = V.colwise().minCoeff();
		Eigen::Vector3d M = V.colwise().maxCoeff();

		Eigen::RowVector3d N;
		Eigen::RowVector3d cp;

		igl::fit_plane(V, N, cp);

		// Plane
		// Ax + By + Cz = D;

		float constantD = -N.dot(cp);
		viewer->data().add_label(cp, "Ground Centre");

		plane p = plane::fromPointNormal(cp, N);
		render_plane(viewer, p, m, M);

		Eigen::MatrixXd V_axis(4, 3);
	}
}

void er::frame_data::invalidate_cloud(pcl_ptr cloud_)
{
	invalidate = true;
	pcl::copyPointCloud(*cloud_, *cloud);

	size_t size = cloud->points.size();
	if (size == 0)
		return;

	calculate_view();
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
	E_axis << 0, 1, 0, 2, 0, 3;

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

	viewer.append_mesh();
	axis_index = viewer.selected_data_index;
}

void er::worker_t::test_cube()
{
	Eigen::MatrixXd V(8, 3);
	V <<
		0.0, 0.0, 0.0,
		0.0, 0.0, 1.0,
		0.0, 1.0, 0.0,
		0.0, 1.0, 1.0,
		1.0, 0.0, 0.0,
		1.0, 0.0, 1.0,
		1.0, 1.0, 0.0,
		1.0, 1.0, 1.0;
	Eigen::MatrixXi F(12, 3);
	F <<
		0, 6, 4,
		0, 2, 6,
		0, 3, 2,
		0, 1, 3,
		2, 7, 6,
		2, 3, 7,
		4, 6, 7,
		4, 7, 5,
		0, 4, 5,
		0, 5, 1,
		1, 5, 7,
		1, 7, 3;

	viewer.data().set_mesh(V, F);
	viewer.append_mesh();
}

void er::worker_t::compute_cloud()
{
	if (!initialized)
		return;

	if (!update_view)
		return;

	while (viewer.selected_data_index != axis_index) {
		viewer.erase_mesh(viewer.selected_data_index);
	}

	viewer.data().clear();

	for (auto &data : data_views) {
		data->render((void *) &viewer);
	}

	update_view = false;
}

void er::worker_t::start()
{
	printf("Start thread\n");

	// Load an example mesh in OFF format
	// Just to check that we are have a functional libIGL.

	// TODO: This thread will crash if the window is not created on time.
	boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

	//------------------------------------------------------------
	// Just a basic testing Grid
	//
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::MatrixXd C;

	igl::readOFF("S:/libigl/tutorial/shared/grid.off", V, F);

	Eigen::VectorXd radius(V.rows());
	radius.setConstant(0.01 * viewer.core.camera_base_zoom);

	Eigen::VectorXd Z = V.col(2);
	igl::jet(Z, true, C);

	viewer.data().set_points(V, C, radius);
	//------------------------------------------------------------

	initialize_visualizer_ui(viewer);

	add_axis();

	viewer.callback_init = [&] (auto viewer_) {
		printf("Setting up camera!\n");

		// Position the camera in our reference frame
		viewer.core.camera_eye = Eigen::Vector3f(0, 0, -2.5f);
		viewer.core.is_animating = true;
		return false;
	};

	viewer.callback_pre_draw = [&] (auto viewer_) {
		return false;
	};

	// Thread were we check to see if we have to invalidate the pointcloud
	viewer.callback_post_draw = [&] (auto viewer_) {
		initialized = true;
		// Position the camera in our reference frame
		compute_cloud();
		return false;
	};

	viewer.launch();
}

std::vector<er::frame_data *> &er::worker_t::get_data_views()
{
	return data_views;
}

er::worker_t *launch_visualizer()
{
	viewer_thread.setup();
	return &viewer_thread;
}
