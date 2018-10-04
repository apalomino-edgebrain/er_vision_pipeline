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

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/lockable_adapter.hpp>
#include <mutex>
#include <iostream>
#include <random>

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/jet.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include "er-pipeline.h"

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

// Lock the data access when we push the cloud.
// On this thread we will copy as fast as we can the pcl frame
// and return.
void push_cloud(pcl_ptr cloud)
{
	data.invalidate_cloud(cloud);
}

class worker_t
{
private:
	int    n_;
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::MatrixXd C;

public:
	void setup(size_t n)
	{
		n_ = static_cast<int>(n);
	}

	void compute_cloud()
	{
		if (!data.invalidate)
			return;

		boost::lock_guard<FrameData> guard(data);

		Eigen::MatrixXd V;
		Eigen::MatrixXd C;
	}

	void start(int n)
	{
		std::random_device rd;
		std::mt19937 e2(rd());
		std::uniform_real_distribution<> dist(0, 1);

		igl::opengl::glfw::Viewer viewer;

		// Load a mesh in OFF format
		igl::readOFF("S:/libigl/tutorial/shared/bunny.off", V, F);

		printf("Start thread %d", n_);

		Eigen::VectorXd radius(V.rows());
		radius.setConstant(0.01 * viewer.core.camera_base_zoom);

		Eigen::VectorXd Z = V.col(2);
		igl::jet(Z, true, C);

		for (int i = 0; i < C.rows(); i ++) {
			C(i, 0) = 0; // dist(e2);
			C(i, 1) = 0;
			C(i, 2) = 1;
		}

		viewer.data().set_points(V, C, radius);

		viewer.launch();
		compute_cloud();
	}

	void operator()()
	{
		start(n_);
	}

	boost::thread *bthread;
};

worker_t viewer_thread;

void launch_visualizer()
{
	viewer_thread.setup(0);
	viewer_thread.bthread = new boost::thread(viewer_thread);
}
