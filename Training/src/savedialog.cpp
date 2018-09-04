#include "savedialog.h"
#include "../build/ui_savedialog.h"

SaveDialog::SaveDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SaveDialog)
{
    ui->setupUi(this);
    cloud_counter=0;
    ui->cloudBox->setEnabled(true);
    prep=false;
    connect (ui->okButton, SIGNAL(clicked ()), this, SLOT(okButtonPressed ()));
    connect (ui->cancelButton, SIGNAL(clicked ()), this, SLOT(cancelButtonPressed ()));
}

void SaveDialog::setCloud(PointCloudT::Ptr in, int id){
    switch(id){
    //scene
    case 0:scene=in;
        ui->cloudBox->addItem("Scene");
        cloud_counter++;
        break;
        //object
    case 1:obj=in;
        ui->cloudBox->addItem("Object");
        cloud_counter++;
        break;
        //Tensor
    case 2:ibs=in;
        ui->cloudBox->addItem("Tensor");
        cloud_counter++;
        break;
    default: std::cout<<"Not handled"<<std::endl;
    }
    if(cloud_counter==2)
        ui->cloudBox->addItem("ALL");
}
void SaveDialog::setCloud(pcl::PolygonMesh::Ptr mesh){
    theMesh=mesh;
    prep=true;
}

void SaveDialog::saveFile(std::string default_name, PointCloudT::Ptr cloud)
{
    default_name="./"+default_name;
    QString fileName=QFileDialog::getSaveFileName(
                this,
                tr("Save pointcloud as"),
                default_name.c_str(),
                tr ("Point cloud data (*.pcd)"));
    if(fileName.isEmpty())
    {
        return;
    }
    pcl::io::savePCDFile(fileName.toStdString().c_str(),*cloud);
    std::cout<<"Saved as: "+fileName.toStdString()<<std::endl;
}

void SaveDialog::okButtonPressed(){
    time_t now = time(0);
    std::stringstream ss;
    ss << now;
    std::string selected_cloud=ui->cloudBox->currentText().toStdString();
    if(selected_cloud.compare("Tensor")==0)
    {
        std::string file_name="Tensor_"+ss.str()+".pcd";
        saveFile(file_name,ibs);
    }
    if(selected_cloud.compare("Object")==0)
    {
        std::string file_name="Obj_"+ss.str()+".pcd";
        saveFile(file_name,obj);
    }
    if(selected_cloud.compare("Scene")==0)
    {
        std::string file_name="Scn_"+ss.str()+".pcd";
        saveFile(file_name,scene);
    }
    if(selected_cloud.compare("ALL")==0)
    {
        all.reset(new PointCloudT);
        pcl::copyPointCloud(*scene,*all);
        cout<<"save all: scene: "<<scene->size()<<" obj: "<<obj->size()<<std::endl;
        if(!obj->empty())
            *all+=*obj;
        if(!ibs->empty())
            *all+=*ibs;
        std::string file_name="All_clouds_"+ss.str()+".pcd";
        saveFile(file_name,all);
    }

}

void SaveDialog::cancelButtonPressed()
{
    this->close();
}

SaveDialog::~SaveDialog()
{
    delete ui;
}
