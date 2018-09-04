#ifndef SAVEDIALOG_H
#define SAVEDIALOG_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <vtkRenderWindow.h>

#include <iostream>

#include <QDialog>
#include <QMessageBox>
#include <QFileDialog>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<PointT> PointCloudT;


namespace Ui {
class SaveDialog;
}

class SaveDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit SaveDialog(QWidget *parent = 0);
    void setCloud(PointCloudT::Ptr in, int id);
    void setCloud(pcl::PolygonMesh::Ptr mesh);
    void saveFile(std::string default_name, PointCloudT::Ptr cloud);
    bool prep;
    ~SaveDialog();

public slots:
    void
    cancelButtonPressed();
    void
    okButtonPressed();
    
protected:
    PointCloudT::Ptr scene,obj,ibs,all;
    pcl::PolygonMesh::Ptr theMesh;
    int cloud_counter;
private:
    Ui::SaveDialog *ui;
};

#endif // SAVEDIALOG_H
