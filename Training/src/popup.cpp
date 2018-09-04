#include "popup.h"
#include "../build/ui_popup.h"
PopUp::PopUp(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PopUp)
{
    ui->setupUi(this);
    viewer_.reset(new pcl::visualization::PCLVisualizer ("viewer",false));
    viewer_->setBackgroundColor(0.1,0.1,0.1);
    ui->qvtkWidgetP->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui->qvtkWidgetP->GetInteractor (), ui->qvtkWidgetP->GetRenderWindow ());
    viewer_->resetCamera ();
    viewer_->addCoordinateSystem(1.0,"id",0);
    cloud.reset(new PointCloudC);
    scene.reset(new PointCloudC);
    ui->qvtkWidgetP->update ();
}

PopUp::~PopUp()
{
    delete ui;
}

void PopUp::setCloud(PointCloudT::Ptr in, int id)
{

    switch(id)
    {
        case 0:pcl::copyPointCloud (*in, *cloud);
               break;
        case 1:pcl::copyPointCloud(*in,*scene);
    }

}

void PopUp::loadCloud()
{
    std::srand(std::time(0)); // use current time as seed for random generator
    int rand_R,rand_G,rand_B;
    if(!scene->empty())
    {
        rand_R=255;
        rand_G=0;
        rand_B=0;
        pcl::visualization::PointCloudColorHandlerCustom<PointC> c2_handler(scene,rand_R,rand_G,rand_B);
        viewer_->addPointCloud(scene,c2_handler,"scene");
    }
    rand_R=0;
    rand_G=255;
    rand_B=0;
    pcl::visualization::PointCloudColorHandlerCustom<PointC> c2_handler(cloud,rand_R,rand_G,rand_B);
    viewer_->addPointCloud(cloud,c2_handler,"ibs");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ibs");
    ui->qvtkWidgetP->update ();
}
