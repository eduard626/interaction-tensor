#ifndef MYGUI_H
#define MYGUI_H

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
#include "myTensor.h"
#include "popup.h"
#include "savedialog.h"
#include <QMainWindow>

#include <QFileDialog>
#include <boost/filesystem.hpp>
// Point Cloud Library
#include <pcl/point_cloud.h>





#ifndef Q_MOC_RUN
#include <pcl/filters/filter.h>
#endif


#include <Eigen/Dense>

struct sampledPoints
{
    int id;
    int counter;
};
struct pointWrap
{
    int id;
    double mag;
};
inline bool sortPoints(const PointWithWeights &one, const PointWithWeights &two){return one.w2<two.w2;}
inline bool sortPointsProb(const sampledPoints &one, const sampledPoints &two){return one.counter<two.counter;}
inline bool sortWithWrap(const pointWrap &one, const pointWrap &two){return one.mag<two.mag;}
inline bool IsZero (const sampledPoints &s) { return s.counter==0; }


struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


inline double uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    void pp_callback(const pcl::visualization::PointPickingEvent &event, void *args);
    ~MainWindow();

public slots:
    void
    saveFileButtonPressed();

    void
    loadFileButtonPressed();

    void
    scaleButtonPressed();

    void
    resetButtonPressed();

    void
    computeIBSPressed2();

    void
    translateButtonPressed();

    

    void
    affBoxChanged(const QString &text);


    void
    nnButtonPressed();

    void
    okButtonPressed();

    void
    rotateButtonPressed();

    void
    bottomBoxButtonPressed();

    void
    topBoxButtonPressed();

    void
    denseButtonPressed();


protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    PointCloudT::Ptr cloud_scene,cloud_object,cloud_ibs,cloud_all,cloud_result,tmp;
    pcl::PointCloud<pcl::Normal>::Ptr normalsCloud;
    pcl::IndicesPtr indxs;
    pcl::PolygonMesh::Ptr mesh_obj;
    PointT p_ob,p_scene,box1,box2;
    std::vector<int> goodNN;
    int n_orientations_descriptor,ppTranslation,cloudsBoxIdx;
    double scale;
    bool normals,pointObject, pointScene;
    struct callback_args cb_args;
    Eigen::Vector4f sCentroid,oCentroid,iCentroid,centroidObjInit,centroidObjFinal;
    float ob_angle, targetNormal,sizeThreshold;
    Tensor *aTensor;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> descriptor,vectors;
    SaveDialog *saveWin;
    std::string affordance,scene_name,obj_name,scene_file,obj_file;
        
private slots:
    void on_actionLoad_triggered();

    void on_actionSave_triggered();

    void on_actionHow_to_triggered();

private:
    Ui::MainWindow *ui;
    int showClosest(pcl::PointXYZ point_scene,PointCloudT::Ptr cloud,int r, int g, int b, char const *id);
    int showClosest(pcl::PointXYZ point_scene,PointCloudT::Ptr cloud,int r, int g, int b, char const *id,float normal);
    bool createSpin(pcl::PointCloud<PointWithVector>::Ptr sample, PointCloudT::Ptr full_ibs,std::string pathh,int orientations=8,bool uniform=false);
    bool getAggloRepresentation(std::vector<float> &mags, std::string pathh,bool uniform=false);
    void getSpinMatrix(pcl::PointCloud<PointWithVector>::Ptr sample, int orientations, pcl::PointCloud<pcl::PointXYZ>::Ptr full);
    void uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> & cloud_out);
    void randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f& p);
    void randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p);
    PointCloud::Ptr densecloud(std::string input, int SAMPLE_POINTS, float leaf_size);
    std::vector<int> sampleWithProbability(std::vector<float> *probabilities, int sampleSize, std::vector<sampledPoints> *aux);
    void display(char const *id);
    void rotateCloud(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out, float angle, char axis,bool origin);
    void rotateCloud(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, float angle, char axis,pcl::PointXYZ pivot);
    void translateCloud(PointCloudT::Ptr in, PointCloudT::Ptr out, pcl::PointXYZ translation);
    void translateCloud(PointCloud::Ptr in, PointCloud::Ptr out, pcl::PointXYZ translation,pcl::PointXYZ reference);
    std::string exec(const char* cmd);
};

#endif // MAINWINDOW_H
