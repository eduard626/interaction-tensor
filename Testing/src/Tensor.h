//Tensor Class

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/norms.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/random_sample.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/console/time.h>


#include <boost/iterator/counting_iterator.hpp>

#include <cstdio>   /* for printf() of help message */
#include <ostream>
#include <stdexcept>
#include <vector>
#include <random>
#include <algorithm>
#include <fstream>
#include <string>
#include <iostream>
#include <cctype>
#include <sys/time.h>
#include <iomanip>
//#include <omp.h>

#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>

#define DIM 3

#define OBJECT 0
#define SCENE 1
#define IBSS 2
#define RESULT 3
#define HEATMAP 4
#define RNDSAMPLE 5
#define NORMALS 6
#define ALL 7

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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;


class Tensor
{
  public:
    bool scaleScene,scaleQuery,converged;
    pcl::PointXYZ minObj,minScene,maxObj,maxScene,refPointIBS,refPointOBJ;
    float originalSize,desiredSize,sampleSize,scaleScnFactor,tree_resolution,sceneSize,agg_th,maxDiag,pred_t;
    int n_descriptor_id,randomSampleSize,referencePointId,nOrientations,metric,refPointIBSId,refPointOBJId,pointCloudType,maxDiagId,localResponse,plotDebug;
    PointCloud::Ptr sceneCloud,queryObjectCloud,bigCloud,targetCloud,auxCloud,randomSampleCloud,cellCentres, spinCloud;
    PointCloudC::Ptr sampleColor,goodColor,saveCloudC;
    PointCloud::Ptr largeDescriptorCloud,largeData,largeIds;
    std::vector<int> randomSampleIdx,randomSampleIdx_original;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::RandomSample<pcl::PointXYZ> randomSample;
    std::string scn_name,method_name,file_name,scene_type,n_descriptor,data_path;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> largeVectors,largeDescriptor;
    std::vector<float> large_mags,large_lengths;
    std::vector<int> ppCentroid,startppCentroid;
    std::vector<std::string> affordance_name,ob_names;
    std::vector<PointCloud::Ptr> object_clouds;
    std::vector<pcl::PolygonMesh> object_meshes;
    Eigen::VectorXf detection_thresholds;
    Eigen::MatrixXf data_counts,alternative_data_counts;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octreeCentres,octree;
    vtkSmartPointer<vtkPolyData> polydata1 ;
    PointCloud::Ptr original_points,toObject;


    Tensor(int argC,char **argV);
    int loadData(std::string affordance, std::string scene_file, std::string obj_file);
    void printUsage();
    void init();
    void printObjInfo();
    void sampleCloud();
    void plot(int id,int r,int g, int b);
    void plot(PointCloud::Ptr cloud,char const *name,int r,int g, int b,int pointSize);
    void plot(pcl::PolygonMesh &object,char const *name,int r,int g, int b);
    void plot(PointCloudC::Ptr cloud,char const *name,int pointSize);
    bool GetPointNormals(const char * name,pcl::PointCloud<pcl::Normal>::Ptr normals, PointCloud::Ptr cloud);
    std::string saveClouds(PointCloudC::Ptr points,PointCloudC::Ptr data);
    float getTemplateData(std::string file_txt);
    std::vector<float> getTemplateData(std::string file_txt, std::string &scene_name_file);
    std::string exec(const char* cmd);  
    static void extractCloud(std::vector<int> indices,PointCloud::Ptr inCloud, PointCloud::Ptr outCloud);
    static void translateCloud(PointCloud::Ptr in,PointCloud::Ptr out, pcl::PointXYZ translation);
    static void translateCloud(PointCloud::Ptr in,PointCloud::Ptr out, pcl::PointXYZ translation, pcl::PointXYZ reference);
    static void rotateCloud(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, float angle, char axis,bool origin);
    static void rotateCloud(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, float angle, char axis,pcl::PointXYZ pivot);
    static void Tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters);
    static void extractRepeatedIds(std::vector<int> indices, PointCloud::Ptr inCloud, PointCloud::Ptr outCloud);
    static PointCloud::Ptr copyAsPointCloud(Eigen::MatrixXf *matrixData);
};
