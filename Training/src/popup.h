#ifndef POPUP_H
#define POPUP_H

#include <QDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#ifndef Q_MOC_RUN
#include <pcl/filters/filter.h>
#endif
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <vtkRenderWindow.h>

#include <iostream>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class PopUp;
}

class PopUp : public QDialog
{
    Q_OBJECT
    
public:
    explicit PopUp(QWidget *parent = 0);
    ~PopUp();

    void setCloud(PointCloudT::Ptr in, int id);

    void loadCloud();

    
private:
    Ui::PopUp *ui;

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    PointCloudC::Ptr cloud,scene;

};

#endif // POPUP_H
