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

class FrameData : public boost::basic_lockable_adapter<boost::mutex>
{
public:
	bool initialized;
	uint32_t time_t;

	boost::mutex mtx;

	volatile uint8_t idx;
	volatile bool invalidate;

	pcl_ptr cloud;

	FrameData()
	{
		idx = 0;
		initialized = false;
		invalidate = true;
		cloud = pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	}

	void invalidate_cloud(pcl_ptr cloud_)
	{
		boost::lock_guard<FrameData> guard(*this);
		invalidate = true;
		pcl::copyPointCloud(*cloud_, *cloud);
	}
};

FrameData data;

// Static opengl window where we render our results on libigl
igl::opengl::glfw::Viewer viewer;

// Viewer thread, we try to lock as less as we can.
// This thread contains our UI and displays and should only be locked to update
// the render objects.

er::worker_t viewer_thread;

// Lock the data access when we push the cloud.
// On this thread we will copy as fast as we can the pcl frame
// and return.
void push_cloud(pcl_ptr cloud)
{
	data.invalidate_cloud(cloud);
}


void er::worker_t::setup(size_t n)
{
	n_ = static_cast<int>(n);
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

	// Plot the edges of the bounding box
	/*
	for (unsigned i = 0; i<E_axis.rows(); ++i)
		viewer.data().add_edges
		(
			V_axis.row(E_axis(i, 0)),
			V_axis.row(E_axis(i, 1)),
			Eigen::RowVector3d(1, 0, 0)
		);
	*/

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

void er::worker_t::compute_cloud(igl::opengl::glfw::Viewer &viewer)
{
	if (!data.invalidate)
		return;

	boost::lock_guard<FrameData> guard(data);
	size_t size = data.cloud->points.size();

	printf("Compute cloud %zd\n", size); \

		V.resize(size, 3);
	C.resize(size, 3);

	int i = 0;
	for (auto& p : data.cloud->points) {

		//printf(" %2.2f, %2.2f, %2.2f ", p.x, p.y, p.z);
		V(i, 0) = p.x;
		V(i, 1) = p.y;
		V(i, 2) = p.z;

		C(i, 0) = p.r / 255.f;
		C(i, 1) = p.g / 255.f;
		C(i, 2) = p.b / 255.f;
		i++;
	}

	Eigen::VectorXd radius(size);
	radius.setConstant(er::app_state::get().point_scale * viewer.core.camera_base_zoom);

	viewer.data().set_points(V, C, radius);

	add_axis();
}

void er::worker_t::start(int n)
{
	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> dist(0, 1);

	// Load a mesh in OFF format
	igl::readOFF("S:/libigl/tutorial/shared/bunny.off", V, F);

	printf("Start thread %d", n_);

	Eigen::VectorXd radius(V.rows());
	radius.setConstant(0.01 * viewer.core.camera_base_zoom);

	Eigen::VectorXd Z = V.col(2);
	igl::jet(Z, true, C);

	for (int i = 0; i < C.rows(); i++) {
		C(i, 0) = 0; // dist(e2);
		C(i, 1) = 0;
		C(i, 2) = 1;
	}

	viewer.data().set_points(V, C, radius);

	printf("- BIND -\n");

	viewer.callback_post_draw = [&] (auto viewer) {
		compute_cloud(viewer);
		return false;
	};

	initialize_visualizer_ui(viewer);

	viewer.launch();
}

void er::worker_t::operator()()
{
	start(n_);
}

void launch_visualizer()
{
	viewer_thread.setup(0);
	viewer_thread.bthread = new boost::thread(viewer_thread);
	//viewer_thread.bthread->join();
}
