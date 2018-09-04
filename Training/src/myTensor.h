//Tensor Class
#ifndef Q_MOC_RUN
#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/common.h>
#include <pcl/common/norms.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_representation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/octree/octree.h>

#include <pcl/visualization/boost.h>
#include <pcl/visualization/point_picking_event.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/math/special_functions/round.hpp>
#endif

#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetSet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/Coordinates.h"
#include "libqhullcpp/QhullPointSet.h"
#include "libqhullcpp/QhullPoints.h"
#include "libqhullcpp/QhullRidge.h"
#include "libqhull/poly.h"


#include <cstdio>   /* for printf() of help message */
#include <ostream>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <iostream>
#include <cctype>
#include <sys/time.h>
#include <iomanip>
#include <random>  /* Needs c++11 */

#include <cstdlib>
#include <ctime>


#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPLYReader.h>
#include <vtkRenderWindow.h>
#include <vtkVersion.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>


#define DIM 3

#ifdef  KDTREE
#include <pcl-1.8/pcl/kdtree/kdtree.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#endif


using orgQhull::Qhull;
using orgQhull::QhullError;
using orgQhull::QhullFacet;
using orgQhull::QhullFacetSet;
using orgQhull::QhullFacetList;
using orgQhull::QhullFacetSetIterator;
using orgQhull::QhullQh;
using orgQhull::QhullPointSet;
using orgQhull::QhullPoint;
using orgQhull::QhullPoints;
using orgQhull::QhullVertex;
using orgQhull::QhullVertexSet;
using orgQhull::QhullVertexList;
using orgQhull::QhullRidge;
using orgQhull::Coordinates;
using orgQhull::QhullFacetListIterator;
using orgQhull::QhullVertexListIterator;

struct ibsPointType{
    PCL_ADD_POINT4D;
    int parent_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(ibsPointType,
                                  (float, x,x)
                                  (float, y,y)
                                  (float, z,z)
                                  (int, parent_id, parent_id))


struct PointWithWeights{
    PCL_ADD_POINT4D;
    float w1;
    float w2;
    float w3;
    float w4;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointWithWeights,
                                  (float, x,x)
                                  (float, y,y)
                                  (float, z,z)
                                  (float, w1, w1)
                                  (float, w2, w2)
                                  (float, w3, w3)
                                  (float, w4, w4))

typedef pcl::PointCloud<ibsPointType> PointCloudIbs;


struct pseudo_id{
    int parent_id;
    int region_id;
};
struct PointWithVector{
  PCL_ADD_POINT4D;
  float v1;
  float v2;
  float v3;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointWithVector,
    (float, x,x)
    (float, y,y)
    (float, z,z)
    (float, v1, v1)
    (float, v2, v2)
    (float, v3, v3))

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<PointT> PointCloudT;
inline bool sortRegion(const pseudo_id &one, const pseudo_id &two){return one.region_id<two.region_id;}
inline bool uniqueRegion(const pseudo_id &one, const pseudo_id &two){return one.region_id==two.region_id;}
class Tensor
{
public:
  pcl::PointXYZ minObj,minScene,maxObj,maxScene,bestPlace,sampledPoint;
  float sphere_rad;
	Coordinates cloud_coord;
  PointCloud::Ptr sceneCloud,queryObjectCloud,bigCloud,auxCloud,ibs;
  PointCloudIbs::Ptr ibs_parents;
  std::vector<std::vector<float> > distan;
  PointCloud maxmin;

  Tensor(bool plotdebug, float oSize, float sSize);
  void qhull_allDistances();    
  void copyCoordinates();
	void prepareClouds();
	static void extractCloud(std::vector<int> indices,PointCloud::Ptr inCloud, PointCloud::Ptr outCloud);
  static bool getNNormals(const char * file,pcl::PointCloud<pcl::Normal>::Ptr normals, PointCloud::Ptr cloud);
};
