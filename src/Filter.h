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
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

namespace askinect
{

template<typename T>
class Filter
{
private:
	typename pcl::PointCloud<T>::Ptr model;

public:
	Filter() : model(new pcl::PointCloud<T>) {}
	~Filter() {}

	const pcl::PointCloud<T> updateModel(const pcl::PointCloud<T> &new_cloud)
	{
		*model += new_cloud;

		// down sample
		pcl::VoxelGrid<T> sor;
		pcl::PointCloud<T> result;
		sor.setInputCloud (model);
		sor.setLeafSize (0.01f, 0.01f, 0.01f);
		sor.filter (result);

		*model = result;

		return *model;
	}
};

}