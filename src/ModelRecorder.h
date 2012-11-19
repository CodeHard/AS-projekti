/*
Copyright (c) 2012 Tommi Tikkanen, Eero Laukkanen

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once
#include <pcl/point_cloud.h>
#include <boost/function.hpp>
#include <boost/signals2/signal.hpp>
#include "Filter.h"
#include "Register.h"
#include "Segment.h"
#include "OpenNICamera.h"
#include "SimulatorCamera.h"
#include "DataTypes.h"

class boost::signals2::connection;

namespace askinect
{

template<typename CameraT, typename PointT>
class ModelRecorder
{
private:
	Register<PointT> reg;
	Filter<PointT> filter;
	Segment<PointT> seg;
	ModelData<PointT> modelData;
	boost::signals2::signal<void (const typename pcl::PointCloud<PointT>::ConstPtr &)> singleCloudSignal;
	boost::signals2::signal<void (typename askinect::ModelData<PointT>)> modelDataSignal;

public:
	CameraT camera;

	ModelRecorder()
	{
		boost::function<void (const typename pcl::PointCloud<PointT>::ConstPtr &)> f =
			boost::bind (&ModelRecorder::capture_cb_, this, _1);
		camera.registerCallback(f);
	}

	~ModelRecorder() {}

	template<typename T>
	boost::signals2::connection registerSingleCloudCallback(const boost::function<T> &callback) {
		return singleCloudSignal.connect(callback);
	}

	template<typename T>
	boost::signals2::connection registerModelDataCallback(const boost::function<T> &callback) {
		return modelDataSignal.connect(callback);
	}


	void capture_cb_(const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
	{
		pcl::PointCloud<PointT> cloudCopy;
		pcl::copyPointCloud(*cloud, cloudCopy);

		// all data is collected to this struct and then sent to ObjectScanner -> GUI via signals
		modelData = ModelData<PointT>();

		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud, modelData.rawCloud, indices);

		/*auto registered = reg.registerNew(*cloud);
		if (registered.size() > 0)
		{
			std::cout << "updating model..." << std::endl;
			auto filteredModel = filter.updateModel(registered);
			singleCloudSignal(boost::make_shared<const pcl::PointCloud<PointT> >(filteredModel));
		}
		else
		{
			// registration failed
		}
		/*auto segmented = segment(filteredModel);
		multiCloudSignal(segmented);*/
		seg.segment(modelData);

		modelDataSignal(modelData);

	}

	void start()
	{
		camera.start();
	}
};

}
