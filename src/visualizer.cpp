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

//#############################################################################
// Pushed point cloud from the realsense Thread
//#############################################################################

er::frame_data::frame_data() : f_external_render { nullptr }, initialized { false },
visible { true }, idx { 0 }, invalidate { false }
{
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
		V(i, 1) = p.y;
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
	if (!visible)
		return;

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

	if (f_external_render != nullptr) {
		f_external_render(viewer_ptr);
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

void er::frame_data::render_text(void *viewer_ptr, Eigen::RowVector3d &cp,
	const char *text)
{
	if (viewer_ptr == nullptr)
		return;

	igl::opengl::glfw::Viewer *viewer = (igl::opengl::glfw::Viewer *) viewer_ptr;
	viewer->data().add_label(cp, text);
}

// Helper to render a plane
void er::frame_data::render_point(void *viewer_ptr,
	Eigen::RowVector3d &cp,
	Eigen::Vector3d &color,
	const char *text)
{
	if (viewer_ptr == nullptr)
		return;

	igl::opengl::glfw::Viewer *viewer = (igl::opengl::glfw::Viewer *) viewer_ptr;

	if (text != nullptr)
		viewer->data().add_label(cp, text);

	Eigen::MatrixXd V(1, 3);
	V << cp[0], cp[1], cp[2];

	Eigen::MatrixXd C(1, 3);
	C << color[0], color[1], color[2];

	Eigen::VectorXd radius(1);
	radius.setConstant(er::app_state::get().point_scale * 10.5);

	viewer->data().set_points(V, C, radius);
	viewer->append_mesh();
}

// Helper to render a plane
void er::frame_data::render_plane(void *viewer_ptr,
	plane &p,
	Eigen::Vector3d &m,
	Eigen::Vector3d &M)
{
	if (viewer_ptr == nullptr)
		return;

	igl::opengl::glfw::Viewer *viewer = (igl::opengl::glfw::Viewer *) viewer_ptr;
	Eigen::MatrixXd V(4, 3);
	V <<
		m(0), p.get_y(m(0), m(2)), m(2),
		M(0), p.get_y(M(0), m(2)), m(2),
		m(0), p.get_y(m(0), M(2)), M(2),
		M(0), p.get_y(M(0), M(2)), M(2);

	if (er::app_state::get().bool_debug_verbose) {
		std::cout << "-- Render Plane --" << std::endl;
		std::cout << V << std::endl;
	}

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

//----------------------------------------------------------------------------

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
	radius.setConstant(er::app_state::get().point_scale * 2.5 * viewer.core.camera_base_zoom);

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

#include <Eigen/Geometry>

//-----------------------------------------------------------------------------
// Random code testing
//-----------------------------------------------------------------------------

void testing()
{
	Eigen::MatrixXd V_test;
	V_test.resize(2, 3);
	V_test << 0, 0, 0,
		0, 1, 0;

	using namespace std;
	using namespace Eigen;

	cout << V_test << endl;

	Tform3 T;
	T = Tform3::Identity();
	cout << "T = Tform3::Identity();" << endl;
	cout << "T=[" << endl << T.affine().matrix() << endl << "];" << endl;
	cout << "T.translate(Vec3(1,2,3));" << endl;
	T.translate(Vec3(1, 2, 3));
	cout << "T=[" << endl << T.affine().matrix() << endl << "];" << endl;

	for (int r = 0; r < V_test.rows(); r++) {
		Vector3d v = V_test.row(r);
		v = T * v;
		V_test.row(r) = v;
	}

	cout << V_test << endl;
}

//-------------------------------------------------------------------------

void er::worker_t::start()
{
	//testing();

	printf("Start thread\n");

	// Load an example mesh in OFF format
	// Just to check that we are have a functional libIGL.

	// TODO: This thread will crash if the window is not created on time.
	boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

	//------------------------------------------------------------
	// Just a basic testing Grid
	//
	{
		Eigen::MatrixXd V, C;
		Eigen::MatrixXi F;

		igl::readOBJ("S:/er_vision_pipeline/assets/floor.obj", V, F);

		Eigen::VectorXd radius(V.rows());
		radius.setConstant(0.01 * viewer.core.camera_base_zoom);

		//Eigen::VectorXd Z = V.col(2);
		//igl::jet(Z, true, C);

		viewer.data().set_mesh(V, F);
		//viewer.data().set_colors(C);
		viewer.append_mesh();
	}

	// Loads a plane grid for testing purposes
	/*
	{
		Eigen::MatrixXd V, C;
		Eigen::MatrixXi F;

		igl::readOFF("S:/er_vision_pipeline/assets/grid.off", V, F);

		Eigen::VectorXd radius(V.rows());
		radius.setConstant(0.01 * viewer.core.camera_base_zoom);

		Eigen::VectorXd Z = V.col(2);
		igl::jet(Z, true, C);

		viewer.data().set_points(V, C, radius);
	}

	*/
	//------------------------------------------------------------

	initialize_visualizer_ui(viewer);

	add_axis();

	viewer.callback_init = [&] (auto viewer_) {
		printf("Setting up camera!\n");

		// Position the camera in our reference frame
		viewer.core.camera_eye = Eigen::Vector3f(0, 1, -0.5f);
		viewer.core.is_animating = true;

		// Load camera position

		viewer.core.camera_translation = app_state::get().load_vec3f("camera_translation");

		Eigen::Vector4f v4 = app_state::get().load_vec4f("trackball_angle");
		viewer.core.trackball_angle = Eigen::Quaternionf( v4.w(), v4.x(), v4.y(), v4.z() );

		viewer.core.camera_zoom = app_state::get().config["camera_zoom"];
		if (viewer.core.camera_zoom == 0) {
			printf(" No camera info, we should default to normal one");
		}

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

	viewer.core.viewport << 0, 0, 1800, 924;

	viewer.launch();
	printf("Closing down");
	er::app_state::get().should_close_app = true;
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
