#include "mainwindow.h"
#include "popup.h"
#include "savedialog.h"
#include "../build/ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Tensor GUI");
    cloud_scene.reset(new PointCloudT);
    cloud_object.reset(new PointCloudT);
    cloud_ibs.reset(new PointCloudT);
    n_orientations_descriptor=8;
    viewer_.reset(new pcl::visualization::PCLVisualizer ("viewer",false));
    viewer_->setBackgroundColor(0.1,0.1,0.1);
    ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->xBox->setMinimum(-5.0);
    ui->xBox->setSingleStep(0.1);
    ui->yBox->setMinimum(-5.0);
    ui->yBox->setSingleStep(0.1);
    ui->zBox->setMinimum(-5.0);
    ui->zBox->setSingleStep(0.1);
    ui->scaleBox->setMinimum(0.01);
    ui->scaleBox->setMaximum(2);
    ui->scaleBox->setSingleStep(0.05);
    ui->radBox->setMinimum(1.0);
    ui->radBox->setMaximum(5.0);
    ui->radBox->setSingleStep(0.1);
    ui->nnButton->setDisabled(true);
    p_scene.x=-100;


    PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer_);
    viewer_->registerPointPickingCallback (&MainWindow::pp_callback,*this ,(void*)&cb_args);

    ui->translateButton->setDisabled(true);

    ui->scaleButton->setDisabled(true);
    //ui->saveButton->setDisabled(true);
    ui->ibsButton->setDisabled(true);
    //ui->saveAllButton->setDisabled(true);
    ui->resetButton->setDisabled(true);
    ui->actionSave->setDisabled(true);

    ui->tabs->setCurrentIndex(0);

    ui->bottomBoxButton->setDisabled(true);
    pointObject=false;
    pointScene=false;
    ppTranslation=0;
    ob_angle=0;
    ui->okButton->setDisabled(true);

    ui->affBox->addItem("-select-");
    ui->affBox->addItem("Hang");
    ui->affBox->addItem("Place");
    ui->affBox->addItem("Fill");
    ui->affBox->addItem("Sit");
    ui->affBox->addItem("Ride");

    connect (ui->resetButton, SIGNAL(clicked()),  this, SLOT(resetButtonPressed()));
    connect (ui->translateButton, SIGNAL(clicked()), this, SLOT(translateButtonPressed()));
    connect (ui->ibsButton, SIGNAL(clicked()), this, SLOT(computeIBSPressed2()));
    connect (ui->scaleButton, SIGNAL(clicked()), this, SLOT(scaleButtonPressed()));
    connect (ui->nnButton, SIGNAL(clicked()), this, SLOT(nnButtonPressed()));
    connect (ui->okButton, SIGNAL(clicked()), this, SLOT(okButtonPressed()));
    connect (ui->rotateButton, SIGNAL(clicked()), this, SLOT(rotateButtonPressed()));
    connect (ui->bottomBoxButton,SIGNAL(clicked()),this,SLOT(bottomBoxButtonPressed()));
    connect (ui->topBoxButton,SIGNAL(clicked()),this,SLOT(topBoxButtonPressed()));
    connect(ui->affBox,SIGNAL(currentIndexChanged(const QString&)),this,SLOT(affBoxChanged(const QString&)));
    connect (ui->denseButton,SIGNAL(clicked()),this,SLOT(denseButtonPressed()));

    mesh_obj.reset(new pcl::PolygonMesh);
    
    viewer_->resetCamera ();
    viewer_->addCoordinateSystem(0.05,"Sys",0);

    ui->cloudsBox->setDuplicatesEnabled(false);
    cloudsBoxIdx=-1;
    //viewer_->addCoordinateSystem(0.1,"id",0);
    ui->qvtkWidget->update ();
}

void MainWindow::affBoxChanged(const QString &text)
{
    std::string currAff=text.toStdString();
    int return_status;
    //pointScene=false;
    QString filename;
    int index_pscene;
    if(currAff.compare("Hang")==0)
    {

        filename="scene-hanging-rack.pcd";
        // One of the hanging "hooks" on the right
        index_pscene=267588;
    }
    if(currAff.compare("Fill")==0)
    {
        filename="scene-sink.pcd";
        //Midle point in the faucet pipe  (where the water comes out)
        index_pscene=140383;
    }
    if(currAff.compare("Place")==0)
    {
        filename="scene-wood-table.pcd";
        // Middle point on top of the table
        index_pscene=105894;
    }
    if(currAff.compare("Sit")==0)
    {
        filename="scene-stool.pcd";
        //Middle point on top of the ottoman chair
        index_pscene=242171;
    }
    if(currAff.compare("Ride")==0)
    {
        filename="scene-motorbike.pcd";
        //Middle point on the bike saddle
        index_pscene=281399;
    }

    ui->AffordanceBox->setText(text);
    PointCloudT::Ptr cloud_tmp (new PointCloudT);
    filename="../data/"+filename;
    return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
    if (return_status != 0)
    {
        PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
        return;
    }
    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    if (cloud_tmp->is_dense)
    {
        if(filename.toStdString().find("scene")!=std::string::npos)
        {
            pcl::copyPointCloud (*cloud_tmp, *cloud_scene);
            PCL_INFO("Scene\n");
            pcl::compute3DCentroid(*cloud_scene,sCentroid);
            display("scene");
            if(!pointScene)
            {
                ui->cloudsBox->addItem("Scene");
                cloudsBoxIdx+=1;
                ui->cloudsBox->setCurrentIndex(cloudsBoxIdx);
            }
            size_t sc_pos=filename.toStdString().find("scene");
            size_t dot_pos=filename.toStdString().find(".");
            scene_name=filename.toStdString().substr(sc_pos,dot_pos-sc_pos);
            scene_file=filename.toStdString();
            normals=false;
        }
    }
    else
    {
        PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
        std::vector<int> vec;
        pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_tmp, vec);
        if(filename.toStdString().find("scene")!=std::string::npos)
        {
            pcl::copyPointCloud (*cloud_tmp, *cloud_scene);
            PCL_INFO("Scene\n");
            pcl::compute3DCentroid(*cloud_scene,sCentroid);
            display("scene");
            if(!pointScene)
            {
                ui->cloudsBox->addItem("Scene");
                cloudsBoxIdx+=1;
                ui->cloudsBox->setCurrentIndex(cloudsBoxIdx);
            }
            size_t sc_pos=filename.toStdString().find("scene");
            size_t dot_pos=filename.toStdString().find(".");
            scene_name=filename.toStdString().substr(sc_pos,dot_pos-sc_pos);
            scene_file=filename.toStdString();
            normals=false;
        }
    }
    if(cloud_scene->size()>0 || cloud_object->size()>0)
        ui->translateButton->setEnabled(true);
    p_scene=cloud_scene->at(index_pscene);
    PointCloudT::Ptr test_point(new PointCloudT);
    test_point->push_back(p_scene);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> Red (test_point, 0, 0, 255);
    viewer_->removePointCloud("clicked_points");
    viewer_->addPointCloud(test_point, Red, "clicked_points");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    pointScene=true;
    viewer_->resetCamera ();
    viewer_->addCoordinateSystem(0.1,"Sys",0);
    ui->qvtkWidget->update ();
}

void MainWindow::nnButtonPressed(){


    float targetAngle=ui->normalBox->value()*M_PI/180;
    float threshold=10*M_PI/180;
    Eigen::Vector3f axis=Eigen::Vector3f::UnitY();
    if(!normals)
    {
        std::cout<<"Normals...";
        normalsCloud.reset(new pcl::PointCloud<pcl::Normal>);
        Tensor a(false,0,0);
        cloud_scene->clear();
        a.getNNormals(scene_file.c_str(),normalsCloud,cloud_scene);
        normals=true;
        std::cout<<"ok"<<std::endl;
    }
    PointT clicked=cb_args.clicked_points_3d->at(0);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_scene);
    std::vector<int> NNidx;
    std::vector<float> NNdist;
    PointCloudC::Ptr goodPoints;
    std::cout<<"Getting NN in radius ";
    if(kdtree.radiusSearch(clicked,.1,NNidx,NNdist)>0)
    {
        goodPoints.reset(new PointCloudC);
        //Look in those neighbours for points with target normals.
        std::cout<<"done"<<std::endl;

        for(int i=0;i<NNidx.size();i++)
        {
            pcl::Normal aNormal=normalsCloud->at(NNidx.at(i));
            Eigen::Vector3f normalVector(aNormal.normal_x,aNormal.normal_y,aNormal.normal_z);
            float angle=normalVector.dot(axis)/(normalVector.norm()*axis.norm());
            if(targetAngle>0)
            {
                if( fabs(std::acos(angle)) <targetAngle && fabs(std::acos(angle))>(targetAngle-threshold))
                {
                    PointC p(255,255,0);
                    p.x=cloud_scene->at(NNidx.at(i)).x;
                    p.y=cloud_scene->at(NNidx.at(i)).y;
                    p.z=cloud_scene->at(NNidx.at(i)).z;
                    goodPoints->push_back(p);
                    goodNN.push_back(NNidx.at(i));
                }
            }
            else
            {
                if( fabs(std::acos(angle)) <targetAngle+threshold)
                {
                    PointC p(255,255,0);
                    p.x=cloud_scene->at(NNidx.at(i)).x;
                    p.y=cloud_scene->at(NNidx.at(i)).y;
                    p.z=cloud_scene->at(NNidx.at(i)).z;
                    goodPoints->push_back(p);
                    goodNN.push_back(NNidx.at(i));
                }
            }

        }

    }
    if(!viewer_->contains("neighbourhood"))
        viewer_->addPointCloud(goodPoints,"neighbourhood",0);
    else
        viewer_->updatePointCloud(goodPoints,"neighbourhood");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,"neighbourhood");
    indxs.reset(new std::vector<int>(goodNN));
    std::cout<<"Found "<<goodNN.size()<<" NN with Normal at "<<targetAngle<<std::endl;
    viewer_->removePointCloud("clicked_points",0);
    //pointScene=true;
    ui->qvtkWidget->update();
    if(goodPoints->size()==0)
        QMessageBox::information(this,tr("Confirmation Message"),tr("No NN found"));
    //std::cout<<"Object point: "<<p_ob<<std::endl;

}

void MainWindow::loadFileButtonPressed()
{
    QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "./", tr ("Point cloud data (*.pcd *.ply *.obj)"));

    //PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
    //std::cout<<"File chosen: "<<filename.toStdString()<<" "<<mesh_obj->cloud.width<< std::endl;

    PointCloudT::Ptr cloud_tmp (new PointCloudT);

    if (filename.isEmpty ())
        return;

    int return_status;
    if (filename.endsWith (".pcd", Qt::CaseInsensitive))
        return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
    else
        if (filename.endsWith(".obj",Qt::CaseInsensitive))
        {
            //This is only for processing shuda's code output. Needs work.
            //Look for the same file with pcd or ply
            PCL_INFO("Loading obj mesh...");
            return_status=pcl::io::loadOBJFile(filename.toStdString(),*mesh_obj);
            std::cout<<"Mesh faces: "<<mesh_obj->polygons.size()<<" points: "<<mesh_obj->cloud.width<<std::endl;
            PCL_INFO("Loaded\n");
            filename.replace(".obj",".pcd");
            int this_return=pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
            if (this_return==0)
            {
                std::cout<<"Found pcd sibling "<<filename.toStdString()<<std::endl;
                std::cout<<"Cloud points: "<<cloud_tmp->size()<<std::endl;
            }
            else
            {
                filename.replace(".pcd",".ply");
                this_return=pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);
                if (this_return==0)
                {
                    std::cout<<"Found ply sibling "<<filename.toStdString()<<std::endl;
                    std::cout<<"Cloud points: "<<cloud_tmp->size()<<std::endl;
                }
                else
                {
                    std::cout<<"Did not found pcd nor ply sibling"<<std::endl;
                    return_status=this_return;
                }
            }
        }
        else
            return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

    if (return_status != 0)
    {
        PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
        return;
    }

    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    if (cloud_tmp->is_dense)
    {
        if(filename.toStdString().find("scene")!=std::string::npos)
        {
            pcl::copyPointCloud (*cloud_tmp, *cloud_scene);
            PCL_INFO("Scene\n");
            pcl::compute3DCentroid(*cloud_scene,sCentroid);
            display("scene");
            ui->cloudsBox->addItem("Scene");
            cloudsBoxIdx+=1;
            ui->cloudsBox->setCurrentIndex(cloudsBoxIdx);
            size_t sc_pos=filename.toStdString().find("scene");
            size_t dot_pos=filename.toStdString().find(".");
            scene_name=filename.toStdString().substr(sc_pos,dot_pos-sc_pos);
            scene_file=filename.toStdString();
            normals=false;
        }
        else
        {
            if(filename.toStdString().find("Tensor")!=std::string::npos)
            {
                pcl::copyPointCloud (*cloud_tmp, *cloud_ibs);
                PCL_INFO("Tensor\n");
                pcl::compute3DCentroid(*cloud_ibs,iCentroid);
                display("Tensor");
                ui->cloudsBox->addItem("Tensor");
                cloudsBoxIdx+=1;
                ui->cloudsBox->setCurrentIndex(cloudsBoxIdx);
            }
            else
            {
                pcl::copyPointCloud (*cloud_tmp, *cloud_object);
                PCL_INFO("Object ");
                pcl::compute3DCentroid(*cloud_object,oCentroid);
                pcl::compute3DCentroid(*cloud_object,centroidObjInit);
                Eigen::Vector4f minP,maxP;
                pcl::getMinMax3D(*cloud_object,minP,maxP);
                box1.x=(minP[0]+maxP[0])/2;
                box1.y=(minP[1]+maxP[1])/2;
                box1.z=(minP[2]+maxP[2])/2;
                PointCloudC::Ptr aPoint(new PointCloudC);
                pcl::PointXYZRGB po(0,0,255);
                po.x=oCentroid[0];
                po.y=oCentroid[1];
                po.z=oCentroid[2];
                aPoint->push_back(po);
                viewer_->addPointCloud(aPoint,"Point",0);
                display("object");
                ui->cloudsBox->addItem("Object");
                cloudsBoxIdx+=1;
                ui->cloudsBox->setCurrentIndex(cloudsBoxIdx);
                std::size_t slash=filename.toStdString().rfind("/");
                if(slash!=std::string::npos)
                {
                    std::size_t point_pos=filename.toStdString().find(".");
                    obj_name=filename.toStdString().substr(slash+1,point_pos-slash-1);
                }
                else
                {
                    std::size_t point_pos=filename.toStdString().find(".");
                    obj_name=filename.toStdString().substr(0,point_pos);
                }
                obj_file=filename.toStdString();
            }
        }
    }
    else
    {
        PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
        std::vector<int> vec;

        pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_tmp, vec);
        if(filename.toStdString().find("scene")!=std::string::npos)
        {
            pcl::copyPointCloud (*cloud_tmp, *cloud_scene);
            PCL_INFO("Scene\n");
            pcl::compute3DCentroid(*cloud_scene,sCentroid);
            display("scene");
            ui->cloudsBox->addItem("Scene");
            cloudsBoxIdx+=1;
            ui->cloudsBox->setCurrentIndex(cloudsBoxIdx);
            size_t sc_pos=filename.toStdString().find("scene");
            size_t dot_pos=filename.toStdString().find(".");
            scene_name=filename.toStdString().substr(sc_pos,dot_pos-sc_pos);
            scene_file=filename.toStdString();
            normals=false;
        }
        else
        {
            if(filename.toStdString().find("Tensor")!=std::string::npos)
            {
                pcl::copyPointCloud (*cloud_tmp, *cloud_ibs);
                PCL_INFO("Tensor\n");
                pcl::compute3DCentroid(*cloud_ibs,iCentroid);
                display("Tensor");
                ui->cloudsBox->addItem("Tensor");
                cloudsBoxIdx+=1;
                ui->cloudsBox->setCurrentIndex(cloudsBoxIdx);
            }
            else
            {
                pcl::copyPointCloud (*cloud_tmp, *cloud_object);
                PCL_INFO("Object ");
                pcl::compute3DCentroid(*cloud_object,oCentroid);
                pcl::compute3DCentroid(*cloud_object,centroidObjInit);
                Eigen::Vector4f minP,maxP;;
                pcl::getMinMax3D(*cloud_object,minP,maxP);
                box1.x=(minP[0]+maxP[0])/2;
                box1.y=(minP[1]+maxP[1])/2;
                box1.z=(minP[2]+maxP[2])/2;
                PointCloudC::Ptr aPoint(new PointCloudC);
                pcl::PointXYZRGB po(0,0,255);
                po.x=oCentroid[0];
                po.y=oCentroid[1];
                po.z=oCentroid[2];
                aPoint->push_back(po);
                viewer_->addPointCloud(aPoint,"Point",0);
                display("object");
                ui->cloudsBox->addItem("Object");
                cloudsBoxIdx+=1;
                ui->cloudsBox->setCurrentIndex(cloudsBoxIdx);
                std::size_t slash=filename.toStdString().rfind("/");
                if(slash!=std::string::npos)
                {
                    std::size_t point_pos=filename.toStdString().find(".");
                    obj_name=filename.toStdString().substr(slash+1,point_pos-slash-1);
                }
                else
                {
                    std::size_t point_pos=filename.toStdString().find(".");
                    obj_name=filename.toStdString().substr(0,point_pos);
                }
                obj_file=filename.toStdString();
            }
        }
    }

    if(viewer_->contains("object"))
    {
        ui->bottomBoxButton->setEnabled(true);
    }
    if(viewer_->contains("object")&&viewer_->contains("scene"))
    {

        ui->ibsButton->setEnabled(true);
    }
    if(viewer_->contains("scene"))
    {
        ui->tensorTab->setEnabled(true);
    }
    if(cloud_scene->size()>0 || cloud_object->size()>0)
        ui->translateButton->setEnabled(true);
    viewer_->resetCamera ();
    viewer_->addCoordinateSystem(0.05,"Sys",0);
    ui->qvtkWidget->update ();
    std::cout<<obj_name<<std::endl;
}

void MainWindow::resetButtonPressed()
{
    viewer_->removeAllPointClouds(0);
    ui->cloudsBox->clear();
    viewer_->resetCamera ();
    ui->qvtkWidget->update ();
    ui->translateButton->setDisabled(true);
    ui->resetButton->setDisabled(true);
    //ui->saveAllButton->setDisabled(true);
    ui->scaleButton->setDisabled(true);
    ui->ibsButton->setDisabled(true);
    pointScene=false;
    if(cloud_scene)
        cloud_scene->clear();
    if(cloud_object)
        cloud_object->clear();
    if(cloud_ibs)
        cloud_object->clear();
    if(cloud_all)
        cloud_all->clear();
    if(cloud_result)
        cloud_result->clear();
    //if(ui->tensorTab->isEnabled())
    //  ui->tensorTab->setDisabled(true);
    cb_args.clicked_points_3d->clear();
}

void MainWindow::okButtonPressed()
{
    if(viewer_->contains("object"))
    {
        pointObject=true;
        p_ob.x=cb_args.clicked_points_3d->at(0).x;
        p_ob.y=cb_args.clicked_points_3d->at(0).y;
        p_ob.z=cb_args.clicked_points_3d->at(0).z;
        std::cout<<"Point in object saved"<<std::endl;
    }
    else
    {

        if(cb_args.clicked_points_3d->size()==1)
        {
            if(goodNN.size()>0)
            {
                PointT t=cb_args.clicked_points_3d->at(0);
                pointScene=true;
                targetNormal=ui->normalBox->value()*M_PI/180;
                std::cout<<"NN size"<<std::endl;
                goodNN.clear();
                viewer_->removePointCloud("neighbourhood",0);
                //pointScene=true;

            }
            else
            {
                pointScene=true;
                p_scene=cb_args.clicked_points_3d->at(0);
            }

            //
        }
        else
        {
            if(cb_args.clicked_points_3d->size()>0)
            {
                for(int i=1;i<cb_args.clicked_points_3d->size();i++)
                {
                    PointT t=cb_args.clicked_points_3d->at(i);
                    std::cout<<t<<std::endl;
                }
            }
        }
        ui->okButton->setDisabled(true);
    }
    ui->qvtkWidget->update();
}

void MainWindow::pp_callback(const pcl::visualization::PointPickingEvent &event, void *args){

    struct callback_args* data = (struct callback_args *)args;
    std::cout<<"Index: "<<event.getPointIndex()<<std::endl;
    if (event.getPointIndex () == -1)
        return;
    int tab=ui->tabs->currentIndex();
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
    switch(tab)
    {
    case 0: //first tab - Compute Tensor
        if(!viewer_->contains("object"))    //only scene has been loaded
        {
            data->clicked_points_3d->clear();
            data->clicked_points_3d->push_back(current_point);

        }
        else
        {
            std::cout<<" in object "<<data->clicked_points_3d->size()<<std::endl;
            if(data->clicked_points_3d->empty())
            {
                data->clicked_points_3d->push_back(current_point);
            }
            else
            {
                data->clicked_points_3d->clear();
                data->clicked_points_3d->push_back(current_point);
            }
        }
        if(data->clicked_points_3d->size()==1 && !pointScene)
            ui->nnButton->setEnabled(true);
        if(goodNN.size()>0)
        {
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            std::cout<<"Looking in the "<<goodNN.size()<<" NN"<<std::endl;
            kdtree.setInputCloud(cloud_scene,indxs);
            std::vector<int> NNidx(1);
            std::vector<float> NNdist(1);
            if(kdtree.nearestKSearch(current_point,1,NNidx,NNdist)>0)
            {
                data->clicked_points_3d->clear();
                data->clicked_points_3d->push_back(cloud_scene->at(NNidx.at(0)));
                ui->translateButton->setEnabled(true);
                //pointScene=true;
                p_scene.x=cloud_scene->at(NNidx.at(0)).x;
                p_scene.y=cloud_scene->at(NNidx.at(0)).y;
                p_scene.z=cloud_scene->at(NNidx.at(0)).z;
                //interactionPoints.push_back(cloud_scene->at(NNidx.at(0)));
            }
        }
        ui->okButton->setEnabled(true);
        break;
    default:
        std::cout<<"Not handled"<<std::endl;
        break;
    }
    data->viewerPtr->removePointCloud("clicked_points");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> Red (data->clicked_points_3d, 0, 0, 255);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, Red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout<<"Clicked points "<<data->clicked_points_3d->size()<<std::endl;
    ui->qvtkWidget->update();
}

void MainWindow::saveFileButtonPressed()
{
    int tab=ui->tabs->currentIndex();
    std::cout<<tab<<std::endl;
    saveWin = new SaveDialog(this);
    switch(tab){

    case 0:
        if(!cloud_scene->empty())
            saveWin->setCloud(cloud_scene,0);
        if(!cloud_object->empty())
            saveWin->setCloud(cloud_object,1);
        if(!cloud_ibs->empty())
            saveWin->setCloud(cloud_ibs,2);
        break;

    default: std::cout<<"Not handled"<<std::endl;
        break;
    }
    saveWin->show();
}

void MainWindow::scaleButtonPressed()
{
    std::string active_cloud=ui->cloudsBox->currentText().toStdString();
    std::cout<<"Scale "<<active_cloud.c_str()<<" \n";
    double scale=(double)ui->scaleBox->value();
    Eigen::Affine3f transform=Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(0,Eigen::Vector3f::UnitX()));
    transform.translation()<<0,0,0;
    transform.scale(scale);
    if(active_cloud.compare("Scene")==0)
    {
        pcl::transformPointCloud (*cloud_scene, *cloud_scene, transform);
        pcl::compute3DCentroid(*cloud_scene,sCentroid);
        display("scene");
    }
    if(active_cloud.compare("Object")==0)
    {
        pcl::transformPointCloud (*cloud_object, *cloud_object, transform);
        pcl::compute3DCentroid(*cloud_object,oCentroid);
        display("object");
    }
    if(active_cloud.compare("Tensor")==0)
    {
        pcl::transformPointCloud (*cloud_ibs, *cloud_ibs, transform);
        pcl::compute3DCentroid(*cloud_ibs,iCentroid);
        display("Tensor");
    }
    ui->scaleBox->setValue(0.00);
    ui->qvtkWidget->update();

}


void MainWindow::computeIBSPressed2()
{
    affordance=ui->AffordanceBox->text().toStdString();
    if(affordance.empty())
    {
        QMessageBox::information(this,tr("Confirmation Message"),tr("No affordance name"));
    }
    else
    {
        std::cout<<"Creating Tensor object...";
        //last two are dummy parameters here, they are object and scene size (diagonal)
        // but these are set later.
        aTensor = new Tensor(false,0.15,10);
        std::cout<<"done.\n";
        //Manually setting important stuff
        aTensor->sceneCloud=cloud_scene;
        aTensor->queryObjectCloud=cloud_object;
        aTensor->auxCloud=aTensor->sceneCloud;
        pcl::getMinMax3D(*aTensor->queryObjectCloud,aTensor->minObj,aTensor->maxObj);
        aTensor->sampledPoint=p_scene;
        std::cout<<"Preparing clouds...";
        aTensor->prepareClouds();   //Copies 3d coordinates from pointcloud to qhull format
        std::cout<<"done\n";
        aTensor->sphere_rad=(float)ui->radBox->value(); //Sphere to clip the voronoi diagram
        float RAD=0.5*(float)ui->radBox->value()*pcl::L2_Norm(aTensor->minObj.getArray3fMap(),aTensor->maxObj.getArray3fMap(),DIM); //Internally, clipping sphere rad is computed like this
        cout<<"RAD: "<<RAD<<std::endl;

        std::cout<<"Scene point: "<<p_scene<<std::endl;
        std::cout<<"\nQhull callback ----- \n";
        //compute voronoi diagram
        aTensor->qhull_allDistances();
        pcl::copyPointCloud(*aTensor->ibs,*cloud_ibs);
        std::cout<<"IBS size "<<aTensor->ibs->size()<<std::endl;
        display("Tensor");
        //Needed to recover transformation applied to original object
        pcl::compute3DCentroid(*cloud_object,centroidObjFinal);
        Eigen::Vector4f minP,maxP;
        pcl::getMinMax3D(*cloud_object,minP,maxP);
        box2.x=(minP[0]+maxP[0])/2;
        box2.y=(minP[1]+maxP[1])/2;
        box2.z=(minP[2]+maxP[2])/2;

        //These two lines to avoid GUI become unresponsive, they don't seem to work tho
        ui->qvtkWidget->update ();
        qApp->processEvents();

        //Extrat part of the scene within the RAD used for voronoi digram clipping
        // only this smaller pointcloud will be used to recover provenance vectors
        // this could also be done inside the qhull callback but at the time
        // it was easier to do it here.
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSCN, kdtreeSearchRad;
        kdtreeSearchRad.setInputCloud(cloud_scene);
        std::vector<int> NNidRad;
        std::vector<float> NNdistRad;
        pcl::PointXYZ min,max;
        pcl::getMinMax3D(*cloud_object,min,max);
        RAD=0.5*pcl::L2_Norm(min.getArray3fMap(),max.getArray3fMap(),3);
        sizeThreshold=RAD;
        kdtreeSearchRad.radiusSearch(p_scene,RAD,NNidRad,NNdistRad);
        PointCloud::Ptr smallScene(new PointCloud);
        Tensor::extractCloud(NNidRad,cloud_scene,smallScene);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> Yellow (smallScene, 255, 255, 0);
        viewer_->addPointCloud(smallScene, Yellow, "rad");
        ui->qvtkWidget->update();

        // Using the smaller scene search the 1-NN for every point
        // in the ibs, also compute a "smoother field" but considering
        // 5-NN
        kdtreeSCN.setInputCloud(smallScene);
        std::vector<int> NNidSC(1);
        std::vector<float> NNdistSC(1);
        std::cout<<"Searching for Tensor vectors"<<std::endl;
        pcl::PointCloud<pcl::PointNormal>::Ptr field(new pcl::PointCloud<pcl::PointNormal>);
        float maxV,minV,sum=0,minS,maxS;
        double sumSmooth=0;
        int knn=5;
        std::vector<std::vector<int> > allNeighbors(cloud_ibs->size(),std::vector<int>(knn,1));
        pcl::PointCloud<pcl::Normal>::Ptr smoothField(new pcl::PointCloud<pcl::Normal>);
        std::vector<int> bad_ids;
        // Try to ingnore "bad vectors" which would be something very large (silly)
        int somethingSilly=1000;
        bool bad_flag=false;
        if(affordance.compare("Place")==0)
            somethingSilly=0;
        std::cout<<"Checking for normals_y greater than "<<somethingSilly<<std::endl;
        for(int i=0;i<cloud_ibs->size();i++)
        {
            PointCloud::Ptr disp_n(new PointCloud);
            PointCloudC::Ptr disp_centre(new PointCloudC);
            pcl::PointXYZ centre=cloud_ibs->at(i);
            if( kdtreeSCN.nearestKSearch(centre,knn,NNidSC,NNdistSC)>0 )
            {
                Eigen::Vector3f resultant(0,0,0);
                Eigen::Vector3f scaled_v;
                for(int j=0;j<knn;j++)
                {
                    allNeighbors.at(i).at(j)=NNidSC.at(j);
                    pcl::PointXYZ sc=smallScene->at(NNidSC.at(j));
                    Eigen::Vector3f component(sc.x-centre.x,sc.y-centre.y,sc.z-centre.z);
                    if(component[2]>somethingSilly && j==0)
                    {
                        bad_ids.push_back(i);
                        bad_flag=true;
                    }
                    resultant+=component;
                    disp_n->push_back(sc);
                    if(j==0)
                    {
                        // first NN is used for provenance vectors
                        pcl::PointXYZRGB dp(0,255,0);
                        dp.x=centre.x;
                        dp.y=centre.y;
                        dp.z=centre.z;
                        disp_centre->push_back(dp);
                        pcl::PointNormal pn;
                        pn.x=centre.x;
                        pn.y=centre.y;
                        pn.z=centre.z;
                        pcl::PointXYZ scp=smallScene->at(NNidSC.at(j));
                        Eigen::Vector3f nVector(scp.x-centre.x,scp.y-centre.y,scp.z-centre.z);
                        scaled_v=nVector;
                        if(i==0)
                        {
                            minV=maxV=nVector.norm();
                        }
                        else
                        {
                            if(minV>nVector.norm())
                                minV=nVector.norm();
                            if(maxV<nVector.norm())
                                maxV=nVector.norm();
                        }
                        sum+=nVector.norm();
                        //nVector.normalize();
                        pn.normal_x=nVector[0];
                        pn.normal_y=nVector[1];
                        pn.normal_z=nVector[2];
                        field->push_back(pn);
                    }
                }
                //normalize "smmother" provenance vector
                scaled_v=scaled_v.norm()*resultant.normalized();
                smoothField->push_back(pcl::Normal(scaled_v[0],scaled_v[1],scaled_v[2]));
                if(i==0)
                {
                    minS=maxS=resultant.norm();
                }
                else
                {
                    if(minS>resultant.norm())
                        minS=resultant.norm();
                    if(maxS<resultant.norm())
                        maxS=resultant.norm();
                }
                sumSmooth+=scaled_v.norm();
            }
            bad_flag=false;
        }

        std::cout<<"Tensor before filtering "<<cloud_ibs->size();
        //If there were some "bad" provenance vectors remove them
        if(bad_ids.size()>0)
        {
            pcl::ExtractIndices<pcl::Normal> extractN;
            pcl::ExtractIndices<pcl::PointNormal> extractF;
            pcl::ExtractIndices<pcl::PointXYZ> extractP;
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
            inliers->indices=bad_ids;
            // Extract the inliers
            extractN.setInputCloud (smoothField);
            extractF.setInputCloud(field);
            extractP.setInputCloud(cloud_ibs);


            extractN.setIndices (inliers);
            extractF.setIndices(inliers);
            extractP.setIndices(inliers);


            extractN.setNegative (true);
            extractF.setNegative(true);
            extractP.setNegative(true);

            extractN.filter (*smoothField);
            extractF.filter (*field);
            extractP.filter (*cloud_ibs);
        }
        //This is a cleaner tensor/ibs, which does not have those bad prov vectors
        PointCloud::Ptr copyIBS(new PointCloud);
        if(bad_ids.size()<1)
            pcl::copyPoint(*cloud_ibs,*copyIBS);
        else
        {
            int aux_id=0;
            int bad=bad_ids.at(aux_id);
            for(int i=0;i<cloud_ibs->size();i++)
            {
                if(i==bad)
                {
                    aux_id+=1;
                    if(aux_id<bad_ids.size())
                        bad=bad_ids.at(aux_id);
                    continue;
                }
                else
                {
                    copyIBS->push_back(cloud_ibs->at(i));
                }
            }
        }
        // Print out some data about the tensor and filtering/cleaning
        std::cout<<"Tensor after filtering "<<copyIBS->size()<<std::endl;
        std::cout<<"Min: "<<minV<<" Max: "<<maxV<<std::endl;
        std::cout<<"Sum: "<<sum<<std::endl;
        std::cout<<"======================"<<std::endl;
        std::cout<<"MinS: "<<minS<<" MaxS: "<<maxS<<std::endl;
        std::cout<<"SumSmooth: "<<sumSmooth<<std::endl;
        pcl::PointCloud<pcl::Normal>::Ptr normals_backup(new pcl::PointCloud<pcl::Normal>);
        float max_out=100,min_out=1;
        //Init probabilities for sampling
        std::vector<float> probs(field->size());

        // Sample size to take from tensor
        int sampleSize=512;

        // Sampled data
        std::vector<sampledPoints> points(field->size());

        // Save the original provenance vectors bacause we are going to normalize
        pcl::copyPointCloud(*field,*normals_backup);

        //The mapping or normalization options
        int myMap=2;  //0-> [0-1], 1->[min_out,max_out], 2->no mapping

        // Newer min/max after filtering needs to be recomputed
        float nMax=0,nMin=1000;
        // Map every vector in the tensor
        for(int i=0;i<field->size();i++)
        {
            Eigen::Vector3f oldNormal(field->at(i).normal_x,field->at(i).normal_y,field->at(i).normal_z);
            float mapped_mag=(oldNormal.norm() - minV) * (max_out- min_out) / (maxV - minV) + min_out;
            float map_prob=(oldNormal.norm() - minV) * (1- 0) / (maxV - minV) + 0;
            // Probability is inverse of magnitude
            // Longer provenance vectors -> lower prob of being sampled
            // Smaller provenance vectors are associated to regions where objects are closer together
            probs.at(i)=1-map_prob;
            points.at(i).id=i;
            points.at(i).counter=0;
            Eigen::Vector3f oldNormalSmooth(smoothField->at(i).normal_x,smoothField->at(i).normal_y,smoothField->at(i).normal_z);
            float mapped_mag2=(oldNormalSmooth.norm() - minV) * (max_out- min_out) / (maxV - minV) + min_out;
            Eigen::Vector3f newNormal,newNormalSmooth;
            if(myMap==1)
            {
                newNormal=(1/mapped_mag)*oldNormal.normalized();
                newNormalSmooth=(1/mapped_mag2)*oldNormalSmooth.normalized();
            }
            if(myMap==0)
            {
                newNormal=probs.at(i)*oldNormal.normalized();
                newNormalSmooth=probs.at(i)*oldNormalSmooth.normalized();
            }
            if(myMap==2)
            {
                newNormal=oldNormal;
                newNormalSmooth=oldNormalSmooth;
            }
            field->at(i).normal_x=newNormal[0];
            field->at(i).normal_y=newNormal[1];
            field->at(i).normal_z=newNormal[2];

            //Check/save new max/min in tensor field
            float mag=newNormal.norm();
            if(mag<nMin)    nMin=mag;
            if(mag>nMax)    nMax=mag;

            smoothField->at(i).normal_x=newNormalSmooth[0];
            smoothField->at(i).normal_y=newNormalSmooth[1];
            smoothField->at(i).normal_z=newNormalSmooth[2];
        }
        // Sample with prob: Probability inversly proportional to provenance vector lenght
        // These are affordance keypoints
        std::vector<int> keypoint_ids=sampleWithProbability(&probs,sampleSize,&points);

        // Aux containers for uniform sampling
        std::vector<int> keypoints_uniform,aux_ids;
        aux_ids.reserve(field->size());
        int n(0);
        std::generate_n(std::back_inserter(aux_ids), field->size(), [n]()mutable { return n++; });
        std::srand (unsigned(std::time(0)));

        // By now aux_ids is filled in ascending order [0-field-size)
        // Shuffle them ramdomly and get the sample
        // These are affordance keypoints aswell
        std::random_shuffle ( aux_ids.begin(), aux_ids.end() );
        keypoints_uniform.assign(aux_ids.begin(),aux_ids.begin()+512);
        std::cout<<"Got "<<keypoint_ids.size()<<" keypoints"<<std::endl;

        // Container to sort affordance keypoint
        std::vector<pointWrap> sortablePoints(keypoint_ids.size());
        std::vector<pointWrap> sortableUnidorm(keypoints_uniform.size());

        // Fill the sortable containers
        for(int i=0;i<keypoint_ids.size();i++)
        {
            pointWrap pW;
            pW.id=keypoint_ids.at(i);
            Eigen::Vector3f vec(normals_backup->at(keypoint_ids.at(i)).normal_x,normals_backup->at(keypoint_ids.at(i)).normal_y,normals_backup->at(keypoint_ids.at(i)).normal_z);
            pW.mag=vec.norm();
            sortablePoints.at(i)=pW;
            pointWrap pwU;
            pwU.id=keypoints_uniform.at(i);
            Eigen::Vector3f vecU(normals_backup->at(keypoints_uniform.at(i)).normal_x,normals_backup->at(keypoints_uniform.at(i)).normal_y,normals_backup->at(keypoints_uniform.at(i)).normal_z);
            pwU.mag=vecU.norm();
            sortableUnidorm.at(i)=pwU;
        }

        // Actual sort according to provenance vector lengh
        // Small (high weight) vectors come first
        std::cout<<"sorting new sample...";
        std::sort(sortablePoints.begin(),sortablePoints.end(),sortWithWrap);
        std::cout<<"done"<<std::endl;
        std::cout<<"sorting uniform sample...";
        std::sort(sortableUnidorm.begin(),sortableUnidorm.end(),sortWithWrap);
        std::cout<<"done"<<std::endl;

        //Sample points sorted according to weight are copied to pointcloud format and saved
        pcl::PointCloud<PointWithVector>::Ptr new_sampleCloud2(new pcl::PointCloud<PointWithVector>);
        pcl::PointCloud<PointWithVector>::Ptr new_sampleCloudU(new pcl::PointCloud<PointWithVector>);

        //Save mags in sampled mapped in 0-1 based on full tensor mags
        std::vector<float> mags_c(sampleSize);
        std::vector<float>mags_cU(sampleSize);
        pcl::PointCloud<pcl::Normal>::Ptr provenanceToPlot(new pcl::PointCloud<pcl::Normal>);
        PointCloudT::Ptr provenanceVectorsAnchor(new PointCloudT);
        std::cout<<"extracting new sample...";
        for(int i=0;i<sampleSize;i++)
        {
            PointWithVector pv,pvU;
            pv.x=cloud_ibs->at(sortablePoints.at(i).id).x;
            pv.y=cloud_ibs->at(sortablePoints.at(i).id).y;
            pv.z=cloud_ibs->at(sortablePoints.at(i).id).z;
            pv.v1=normals_backup->at(sortablePoints.at(i).id).normal_x;
            pv.v2=normals_backup->at(sortablePoints.at(i).id).normal_y;
            pv.v3=normals_backup->at(sortablePoints.at(i).id).normal_z;
            new_sampleCloud2->push_back(pv);
            //pcl::Normal n(pv.v1,pv.v2,pv.v3);
            //normals_check->push_back(n);
            Eigen::Vector3f n(pv.v1,pv.v2,pv.v3);
            //Save mags in sampled mapped in 0-1 based on full tensor mags
            mags_c.at(i)=1 - ((n.norm()-nMin)*(1-0)/(nMax-nMin)+0 );

            pvU.x=cloud_ibs->at(sortableUnidorm.at(i).id).x;
            pvU.y=cloud_ibs->at(sortableUnidorm.at(i).id).y;
            pvU.z=cloud_ibs->at(sortableUnidorm.at(i).id).z;
            pcl::PointXYZ provenanceAnchor(pvU.x,pvU.y,pvU.z);
            provenanceVectorsAnchor->push_back(provenanceAnchor);
            pvU.v1=normals_backup->at(sortableUnidorm.at(i).id).normal_x;
            pvU.v2=normals_backup->at(sortableUnidorm.at(i).id).normal_y;
            pvU.v3=normals_backup->at(sortableUnidorm.at(i).id).normal_z;
            new_sampleCloudU->push_back(pvU);
            provenanceToPlot->push_back(normals_backup->at(sortableUnidorm.at(i).id));
            Eigen::Vector3f nU(pvU.v1,pvU.v2,pvU.v3);
            //Save mags in sampled mapped in 0-1 based on full tensor mags
            mags_cU.at(i)=1-( (nU.norm()-nMin)*(1-0)/(nMax-nMin)+0 );
        }
        std::cout<<"done"<<std::endl;

        // Some file names to save data
        // Some of the data saved in this file is only kept
        // to not break previous code but for most recent
        // version is not used.
        //Check a path exists
        std::string aff_path=affordance+"/";
        if (!boost::filesystem::exists( affordance.c_str() ) )
        {
            std::cout << "New affordance? Creating dir -> " <<aff_path<< std::endl;
            std::string command="mkdir "+aff_path;
            command=exec(command.c_str());
        }
        else
        {
            std::cout << "Found affordance dir -> " <<aff_path<< std::endl;
        }

        std::string new_ibs_field=aff_path+affordance+"_"+obj_name+"_field.pcd";
        std::string new_ibs_sample=aff_path+"ibs_sample_512_"+affordance+"_"+obj_name+"_better.pcd";
        std::string new_ibs_sampleU=aff_path+"ibs_sample_512_"+affordance+"_"+obj_name+"_betterUniform.pcd";

        // Some data for info file
        // Closest point in Tensor to scene and in object to scene
        // were used previously to estimate the pose of Tensor/Object
        // relative to scene. These are still computed and save but no longer
        // used, the new pose is computed using center of bounding boxes.
        std::cout<<"Getting closest point in Tensor to scene"<<std::endl;
        int one = showClosest(p_scene,cloud_ibs,255,0,0,"ClosestTensor");
        std::cout<<"Getting closest from object to scene"<<std::endl;
        int two=showClosest(p_scene,cloud_object,0,255,0,"ClosestOBJ");
        int three=showClosest(p_scene,cloud_scene,0,255,0,"ClosestSCN");
        Eigen::Vector3f toIBS,toObject;
        toIBS[0]=cloud_ibs->at(one).x-p_scene.x;
        toIBS[1]=cloud_ibs->at(one).y-p_scene.y;
        toIBS[2]=cloud_ibs->at(one).z-p_scene.z;
        toObject[0]=cloud_object->at(two).x-p_scene.x;
        toObject[1]=cloud_object->at(two).y-p_scene.y;
        toObject[2]=cloud_object->at(two).z-p_scene.z;

        // Info file name
        std::string file_name=aff_path+"ibs_full_"+affordance+"_"+obj_name+".txt";
        std::ofstream output_file(file_name.c_str());
        int clusters=1;
        float size=0;

        // Previously some clustering was performed over IBS points to estimate
        // clusters, only one (referencing=single) of this clusters was then used to estimate poses but
        // we dropped that. Data is still saved.
        std::string referencing="Single";

        //Start saving to file
        if(output_file.is_open())
        {
            output_file<<"Scene name:"<<scene_name<<"\n";
            output_file<<"Object name:"<<obj_name<<"\n";
            output_file<<"Clusters:"<<clusters<<"\n";
            for(int i=0;i<clusters;i++)
                output_file<<cloud_ibs->at(one).x<<","<<cloud_ibs->at(one).y<<","<<cloud_ibs->at(one).z<<"\n";
            output_file<<"Distance threshold:"<<size<<"\n";
            output_file<<"Reference:"<<referencing<<"\n";
            output_file<<one<<":"<<cloud_ibs->at(one).x<<","<<cloud_ibs->at(one).y<<","<<cloud_ibs->at(one).z<<"\n";
            output_file<<"ScenePoint\n";
            output_file<<three<<":"<<cloud_scene->at(three).x<<","<<cloud_scene->at(three).y<<","<<cloud_scene->at(three).z<<"\n";
            output_file<<"IbsPointVector\n";
            output_file<<one<<":"<<toIBS[0]<<","<<toIBS[1]<<","<<toIBS[2]<<"\n";
            output_file<<"ObjPointVector\n";
            output_file<<two<<":"<<toObject[0]<<","<<toObject[1]<<","<<toObject[2]<<"\n";
            output_file<<"Object Transformation\n";
            output_file<<centroidObjFinal[0]-centroidObjInit[0]<<","<<centroidObjFinal[1]-centroidObjInit[1]<<","<<centroidObjFinal[2]-centroidObjInit[2]<<"\n";
            output_file<<ob_angle<<"\n";
            output_file<<"Object Transformation Box\n";
            output_file<<box2.x-box1.x<<","<<box2.y-box1.y<<","<<box2.z-box1.z<<"\n";
            output_file<<"SceneToBoxCentroid\n";
            output_file<<box2.x-p_scene.x<<","<<box2.y-p_scene.y<<","<<box2.z-p_scene.z<<"\n";
        }
        else
        {
            std::cout<<"Problem with data file "<<std::endl;
        }
        output_file.close();
        // Scene-to-IBS and Scene-to-object are saved in affordance keypoints file
        // As commented earlier it was used to align pointclouds
        // at test time. No longer used but still kept in files.
        PointWithVector secondtolast,last;
        secondtolast.x=toIBS[0];
        secondtolast.y=toIBS[1];
        secondtolast.z=toIBS[2];
        secondtolast.v1=one;
        secondtolast.v2=secondtolast.v3=0;
        new_sampleCloud2->push_back(secondtolast);
        last.x=toObject[0];
        last.y=toObject[1];
        last.z=toObject[2];
        last.v1=two;
        last.v2=last.v3=0;
        new_sampleCloud2->push_back(last);
        new_sampleCloudU->push_back(secondtolast);
        new_sampleCloudU->push_back(last);
        
        // Save everything
        pcl::io::savePCDFileASCII(new_ibs_field.c_str(),*field);
        pcl::io::savePCDFile(new_ibs_sample.c_str(),*new_sampleCloud2);
        pcl::io::savePCDFile(new_ibs_sampleU.c_str(),*new_sampleCloudU);
        std::cout<<"Done and saved as "<<new_ibs_field<<std::endl;
        std::string smoother_field=aff_path+affordance+"_"+obj_name+"_smoothfield.pcd";
        std::string clean_ibs=aff_path+"ibs_full_"+affordance+"_"+obj_name+"_clean.pcd";
        std::string full_ibs=aff_path+"ibs_full_"+affordance+"_"+obj_name+".pcd";
        pcl::io::savePCDFile(smoother_field.c_str(),*smoothField);
        pcl::io::savePCDFile(clean_ibs,*copyIBS);
        pcl::io::savePCDFile(full_ibs,*cloud_ibs);

        // By default compute spin cloud for 8 orientations
        // Can be changed and passed as parameter
        // int n_orientations=8;
        // It was simpler to compute and store the descriptor for X-orientations than
        // for 1 orientation and then rotate X-times at test time.
        // So we compute this X-orientations and store them for testing.

        // Spin cloud for weight-sampled
        createSpin(new_sampleCloud2,cloud_ibs,aff_path);
        // New representation for agglomerative descriptor
        // In following release, multiple affordaces can be detected
        // at same time, single affordance representation (this code)
        // is adapated to work with newer code. This "adaptation" is
        // basically wrap (or format) the descriptor in a highly
        // parallelizble way.

        if(getAggloRepresentation(mags_c,aff_path))
        {
            std::cout<<"Everything ok"<<std::endl;
        }
        // Spin cloud for uniform sampled
        createSpin(new_sampleCloudU,cloud_ibs,aff_path,8,true);
        if(getAggloRepresentation(mags_cU,aff_path,true))
        {
            std::cout<<"Everything ok"<<std::endl;
        }

        //Plot provenance vectors (uniform sampling) scaled by 50%
        if (viewer_->contains("provenanceVectors"))
            viewer_->removePointCloud("provenanceVectors",0);
        viewer_->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (provenanceVectorsAnchor, provenanceToPlot, 1, .5, "provenanceVectors");
        ui->qvtkWidget->update();
        //Save a copy of the query-object
        std::string command="cp "+obj_file+" "+aff_path;
        std::string commnad_output=exec(command.c_str());

    }
}

std::string MainWindow::exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

void MainWindow::rotateButtonPressed()
{
    std::string active_cloud=ui->cloudsBox->currentText().toStdString();
    std::cout<<"Rotate "<<active_cloud.c_str()<<" ";
    Eigen::Affine3f transform=Eigen::Affine3f::Identity();
    float angle=ui->zBox->value();
    if(angle==0.0)
    {
        ob_angle+=M_PI/(n_orientations_descriptor);
        transform.rotate(Eigen::AngleAxisf(M_PI/(n_orientations_descriptor),Eigen::Vector3f::UnitZ()));
    }
    else
    {
        ob_angle+=angle;
        transform.rotate(Eigen::AngleAxisf(angle,Eigen::Vector3f::UnitZ()));
    }
    transform.translation()<<0,0,0;
    if(active_cloud.compare("Scene")==0)
    {
        pcl::transformPointCloud (*cloud_scene, *cloud_scene, transform);
        pcl::compute3DCentroid(*cloud_scene,sCentroid);
        std::cout<<sCentroid<<"\n";
        display("scene");
    }
    if(active_cloud.compare("Object")==0)
    {
        //pcl::transformPointCloud (*cloud_object, *cloud_object, transform);
        //pcl::transformPointCloud(*plane,*plane,transform);
        if(angle==0.0)
            rotateCloud(cloud_object,cloud_object,M_PI/(n_orientations_descriptor),'z',box1);
        //rotateCloud(cloud_object, cloud_object, M_PI/(2*n_slices), 'z',false);
        else
            rotateCloud(cloud_object,cloud_object,angle,'z',box1);
        //rotateCloud(cloud_object, cloud_object, angle, 'z',false);
        pcl::compute3DCentroid(*cloud_object,oCentroid);
        std::cout<<oCentroid<<"\n";
        display("object");
    }
    ui->xBox->setValue(0.00);
    ui->yBox->setValue(0.00);
    ui->zBox->setValue(0.00);

    ui->qvtkWidget->update();

}

int MainWindow::showClosest(pcl::PointXYZ point_scene,PointCloudT::Ptr cloud,int r, int g, int b, char const *id)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> NNidx(1);
    std::vector<float> NNdist(1);
    PointCloudC::Ptr closePoint(new PointCloudC);
    pcl::PointXYZRGB a(r,g,b);
    //closePoint->push_back(a);
    std::cout<<"Getting NN ";
    if(kdtree.nearestKSearch (point_scene, 1, NNidx, NNdist) > 0 )
    {
        a.x=cloud->at(NNidx.at(0)).x;
        a.y=cloud->at(NNidx.at(0)).y;
        a.z=cloud->at(NNidx.at(0)).z;
        closePoint->push_back(a);
        viewer_->addPointCloud(closePoint,id,0);
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, id);
        std::cout<<"NN found"<<std::endl;
        return NNidx.at(0);
    }
    else
    {
        std::cout<<" NN not found "<<std::endl;
        return -1;
    }
}

int MainWindow::showClosest(pcl::PointXYZ point_scene, PointCloudT::Ptr cloud, int r, int g, int b, const char *id_cloud, float normal)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> NNidx(1);
    std::vector<float> NNdist(1);
    PointCloudC::Ptr closePoint(new PointCloudC);
    PointCloudC::Ptr closePoints(new PointCloudC);
    pcl::PointXYZRGB a(r,g,b);
    //closePoint->push_back(a);
    std::cout<<"Getting NN in radius ";
    float distance;
    if(kdtree.nearestKSearch (point_scene, 1, NNidx, NNdist) > 0 )
    {
        std::cout<<"Got closest with distace: ";
        distance=std::sqrt(NNdist.at(0));
        std::cout<<distance<<std::endl;
    }
    else
    {
        std::cout<<" NN not found "<<std::endl;
        return -1;
    }
    std::vector<int> NNidx2;
    std::vector<float> NNdist2;
    if(kdtree.radiusSearch(point_scene,2*sizeThreshold,NNidx2,NNdist2)>0)
    {
        std::cout<<"Got "<<NNidx2.size()<<" NN"<<std::endl;
        int id=-1;
        float d=10;
        float threshold=10*M_PI/180;
        Eigen::Vector3f axis=Eigen::Vector3f::UnitY();
        for(int i=0;i<NNidx2.size();i++)
        {
            PointC p(255,255,0);
            p.x=cloud->at(NNidx2.at(i)).x;
            p.y=cloud->at(NNidx2.at(i)).y;
            p.z=cloud->at(NNidx2.at(i)).z;
            closePoints->push_back(p);
            pcl::Normal aNormal=normalsCloud->at(NNidx2.at(i));
            Eigen::Vector3f normalVector(aNormal.normal_x,aNormal.normal_y,aNormal.normal_z);
            float angle=normalVector.dot(axis)/(normalVector.norm()*axis.norm());
            if(normal>0)
            {
                if( fabs(std::acos(angle)) <normal && fabs(std::acos(angle))>(normal-threshold))
                {
                    if(NNdist2.at(i)<d)
                    {
                        id=NNidx2.at(i);
                        d=NNdist2.at(i);
                    }
                }
            }
            else
            {
                if( fabs(std::acos(angle)) <normal+threshold)
                {
                    if(NNdist2.at(i)<d)
                    {
                        id=NNidx2.at(i);
                        d=NNdist2.at(i);
                    }
                }
            }
        }
        if(id!=-1)
        {
            a.x=cloud->at(id).x;
            a.y=cloud->at(id).y;
            a.z=cloud->at(id).z;
            closePoint->push_back(a);
            viewer_->addPointCloud(closePoint,id_cloud,0);
            //viewer_->addPointCloud(closePoints,"id_cloud",0);
            viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, id_cloud);
            //viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "id_cloud");
            std::cout<<"NN found"<<std::endl;
            return id;
        }
        else
        {
            std::cout<<" NN not found "<<std::endl;
            return -1;
        }
    }
    else
    {
        std::cout<<" NN not found "<<std::endl;
        return -1;
    }
}

bool MainWindow::createSpin(pcl::PointCloud<PointWithVector>::Ptr sample,PointCloudT::Ptr full_ibs,std::string pathh,int orientations,  bool uniform)
{
    std::stringstream ii;
    ii<<orientations;
    getSpinMatrix(sample,orientations,full_ibs);
    std::string spin_file;
    std::string spinvectors_file;
    if(uniform)
    {
        spin_file=pathh+affordance+"_"+obj_name+"_spinU_"+ii.str()+".dat";
        spinvectors_file=pathh+affordance+"_"+obj_name+"_spinUvectors_"+ii.str()+".dat";
    }
    else
    {
        spin_file=pathh+affordance+"_"+obj_name+"_spin_"+ii.str()+".dat";
        spinvectors_file=pathh+affordance+"_"+obj_name+"_spinvectors_"+ii.str()+".dat";
    }
    std::ofstream file(spin_file.c_str());
    pcl::saveBinary(descriptor,file);
    std::ofstream file2(spinvectors_file.c_str());
    pcl::saveBinary(vectors,file2);
    std::cout<<"Wrote: "<<spin_file<<" and "<<spinvectors_file<<std::endl;
    //if reached this point everything is ok
    return true;
}

bool MainWindow::getAggloRepresentation(std::vector<float> &mags, std::string pathh, bool uniform)
{

    int sampleSize=vectors.rows();
    PointCloudT::Ptr aux_cloud(new PointCloudT);
    aux_cloud->resize(descriptor.rows());
    PointCloudT::Ptr useful_cloud(new PointCloudT);
    useful_cloud->resize(descriptor.rows());
    PointCloudT::Ptr better_approx(new PointCloudT);
    better_approx->resize(descriptor.rows());
    PointCloudT::Ptr bare_points(new PointCloudT);
    bare_points->resize(descriptor.rows());
    PointCloudT::Ptr vector_ids_agglomerative(new PointCloudT);
    vector_ids_agglomerative->resize(descriptor.rows());
    PointCloudT::Ptr vectors_data_cloud(new PointCloudT);
    vectors_data_cloud->resize(descriptor.rows());
    // point counts per affordance per orientation
    // Mostly useful for normalization in multiple affordance prediction
    // for single affordance case: 1x8 matrix with sampleSize in each element
    int n_orientations=descriptor.rows()/vectors.rows();
    Eigen::MatrixXf data_individual(1,n_orientations);
    data_individual<<Eigen::MatrixXf::Zero(1,n_orientations);


    for(int i=0;i<descriptor.rows();i++)
    {
        int orientation_id=std::floor(i/sampleSize); // [0-nOrientations) default: 8 orientations
        data_individual(0,orientation_id)+=1;
        int smaller_id=i-(sampleSize*orientation_id); // [0-sampleSize)
        aux_cloud->at(i).x=1;
        aux_cloud->at(i).y=i;
        aux_cloud->at(i).z=0;

        useful_cloud->at(i).x=1;
        useful_cloud->at(i).y=orientation_id;
        useful_cloud->at(i).z=smaller_id;

        better_approx->at(i).x=bare_points->at(i).x=descriptor(i,0);
        better_approx->at(i).y=bare_points->at(i).y=descriptor(i,1);
        better_approx->at(i).z=bare_points->at(i).z=descriptor(i,2);

        vector_ids_agglomerative->at(i).x=vectors(smaller_id,0);
        vector_ids_agglomerative->at(i).y=vectors(smaller_id,1);
        vector_ids_agglomerative->at(i).z=vectors(smaller_id,2);

        Eigen::Vector3f aVector=vectors.row(smaller_id);
        vectors_data_cloud->at(i).x=aVector.norm();
        vectors_data_cloud->at(i).y=mags.at(smaller_id);
        vectors_data_cloud->at(i).z=0;
    }
    std::cout<<"Point counts "<<data_individual<<std::endl;
    std::string base_name;
    std::stringstream ii;
    ii<<n_orientations;
    // Save everything with a correct name
    // if uniform sampling or different
    if(uniform)
        base_name=pathh+"UNew_"+affordance+"_"+obj_name+"_descriptor_"+ii.str();
    else
        base_name=pathh+"New_"+affordance+"_"+obj_name+"_descriptor_"+ii.str();
    std::string file_name=base_name+"_members.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*aux_cloud);
    file_name=base_name+"_extra.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*useful_cloud);
    file_name=base_name+".pcd";
    pcl::io::savePCDFile(file_name.c_str(),*better_approx);
    file_name=base_name+"_points.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*bare_points);
    file_name=base_name+"_vectors.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*vector_ids_agglomerative);
    file_name=base_name+"_vdata.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*vectors_data_cloud);
    file_name=pathh+affordance+"_"+obj_name+"_point_count.dat";
    std::ofstream ofs (file_name, std::ofstream::out);
    pcl::saveBinary(data_individual,ofs);
    return true;
}

void MainWindow::translateButtonPressed()
{
    std::cout<<"pointScene: "<<pointScene<<" pointObject: "<<pointObject<<std::endl;
    if(pointScene && pointObject)
    {
        std::cout<<"Point to point"<<std::endl;
        float distance=ui->normalMagBox_2->value();
        Eigen::Affine3f transform=Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(0,Eigen::Vector3f::UnitX()));
        transform.translation()<<p_scene.x-p_ob.x,p_scene.y-p_ob.y,p_scene.z+distance-p_ob.z;
        pcl::transformPointCloud (*cloud_object, *cloud_object, transform);
        pcl::transformPoint(p_ob,transform);
        pcl::compute3DCentroid(*cloud_object,oCentroid);
        std::cout<<oCentroid<<"\n";
        display("object");
        viewer_->removePointCloud("neighbourhood",0);
        pointScene=false;
        //p_ob.x=p_scene.x;
        //p_ob.y=p_scene.y;
        //p_ob.z=p_scene.z;
        cb_args.clicked_points_3d->clear();
        ppTranslation=0;
        //showClosest(p_scene,cloud_object,0,255,0,"ClosestOB");
    }
    else
    {
        std::string active_cloud=ui->cloudsBox->currentText().toStdString();
        std::cout<<"Translate "<<active_cloud.c_str()<<" ";
        double x=(double)ui->xBox->value();
        double y=(double)ui->yBox->value();
        double z=(double)ui->zBox->value();
        std::cout<<x<<" "<<y<<" "<<z<<"\n";
        Eigen::Affine3f transform=Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(0,Eigen::Vector3f::UnitX()));
        if(active_cloud.compare("Scene")==0)
        {
            transform.translation()<<x,y,z;
            pcl::transformPointCloud (*cloud_scene, *cloud_scene, transform);
            pcl::compute3DCentroid(*cloud_scene,sCentroid);
            std::cout<<sCentroid<<"\n";
            display("scene");
        }
        if(active_cloud.compare("Object")==0)
        {
            transform.translation()<<x,y,z;
            pcl::transformPointCloud (*cloud_object, *cloud_object, transform);
            pcl::compute3DCentroid(*cloud_object,oCentroid);
            std::cout<<oCentroid<<"\n";
            display("object");
        }
        if(active_cloud.compare("Tensor")==0)
        {
            transform.translation()<<x,y,z;
            pcl::transformPointCloud (*cloud_ibs, *cloud_ibs, transform);
            pcl::compute3DCentroid(*cloud_ibs,iCentroid);
            std::cout<<iCentroid<<"\n";
            display("Tensor");
        }
    }
    ui->xBox->setValue(0.00);
    ui->yBox->setValue(0.00);
    ui->zBox->setValue(0.00);

    ui->qvtkWidget->update();

}

void MainWindow::display(char const *id)
{
    /*std::srand(std::time(0)); // use current time as seed for random generator
    int rand_R=std::rand()%255+0;
    int rand_G=std::rand()%255+0;
    int rand_B=std::rand()%255+0;*/
    int rand_R,rand_G,rand_B;
    PointCloudC::Ptr c2_color(new PointCloudC);
    if (id=="object")
    {
        //green
        pcl::copyPointCloud(*cloud_object,*c2_color);
        rand_R=0;
        rand_G=255;
        rand_B=0;
    }
    if (id=="scene")
    {
        //red
        pcl::copyPointCloud(*cloud_scene,*c2_color);
        rand_R=255;
        rand_G=0;
        rand_B=0;
    }
    if (id=="Tensor")
    {
        //blue
        pcl::copyPointCloud(*cloud_ibs,*c2_color);
        rand_R=0;
        rand_G=0;
        rand_B=255;
    }
    pcl::visualization::PointCloudColorHandlerCustom<PointC> c2_handler(c2_color,rand_R,rand_G,rand_B);
    if(viewer_->contains(id))
        viewer_->updatePointCloud(c2_color,c2_handler,id);
    else
        viewer_->addPointCloud(c2_color,c2_handler,id);
    if(!ui->resetButton->isEnabled())
        ui->resetButton->setEnabled(true);
    //if(!ui->saveAllButton->isEnabled())
    //ui->saveAllButton->setEnabled(true);
    if(!ui->scaleButton->isEnabled())
        ui->scaleButton->setEnabled(true);
    //if(!ui->saveButton->isEnabled())
    //ui->saveButton->setEnabled(true);
    if(!ui->actionSave->isEnabled())
        ui->actionSave->setEnabled(true);
}

void MainWindow::rotateCloud(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out, float angle, char axis,bool origin){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
    pcl::compute3DCentroid(*cloud_in, centroid);
    Eigen::Vector3f ax;
    switch(axis)
    {
    case 'x':ax=Eigen::Vector3f::UnitX();
        break;
    case 'y':ax=Eigen::Vector3f::UnitY();
        break;
    case 'z':ax=Eigen::Vector3f::UnitZ();
        break;
    }
    Eigen::Matrix3f rotation(Eigen::AngleAxisf (angle, ax));
    transform.rotate(rotation);
    transform.translation()<<0,0,0;
    pcl::transformPointCloud (*cloud_in, *cloud_out, transform);
    if (!origin)
    {
        Eigen::Vector4f centroid_new (Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*cloud_out, centroid_new);
        Eigen::Vector4f diff=centroid-centroid_new;
        translateCloud(cloud_out,cloud_out,pcl::PointXYZ(centroid[0],centroid[1],centroid[2]));
    }
}

void MainWindow::rotateCloud(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, float angle, char axis,pcl::PointXYZ pivot){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
    Eigen::Vector3f ax;
    //pcl::PointXYZ pivot;
    int id=cloud_in->size();
    cloud_in->push_back(pivot);
    //pivot.x=cloud_in->at(pivot_id).x;
    //pivot.y=cloud_in->at(pivot_id).y;
    //pivot.z=cloud_in->at(pivot_id).z;
    switch(axis)
    {
    case 'x':ax=Eigen::Vector3f::UnitX();
        break;
    case 'y':ax=Eigen::Vector3f::UnitY();
        break;
    case 'z':ax=Eigen::Vector3f::UnitZ();
        break;
    }
    Eigen::Matrix3f rotation(Eigen::AngleAxisf (angle, ax));
    transform.rotate(rotation);
    transform.translation()<<0,0,0;
    pcl::transformPointCloud (*cloud_in, *cloud_out, transform);
    pcl::PointXYZ newPivot=cloud_out->at(id);
    //newPivot.x=cloud_out->at(pivot_id).x;
    //newPivot.y=cloud_out->at(pivot_id).y;
    //newPivot.z=cloud_out->at(pivot_id).z;
    translateCloud(cloud_out,cloud_out,pivot,newPivot);
    PointCloud::iterator iter=cloud_out->end();
    --iter;
    cloud_out->erase(iter);
}

void MainWindow::translateCloud(PointCloudT::Ptr in, PointCloudT::Ptr out, pcl::PointXYZ translation){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*in,centroid);
    transform.translation() << translation.x-centroid[0],translation.y-centroid[1],translation.z-centroid[2];
    transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*in, *out, transform);
}
void MainWindow::translateCloud(PointCloud::Ptr in, PointCloud::Ptr out, pcl::PointXYZ translation,pcl::PointXYZ reference){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << translation.x-reference.x,translation.y-reference.y,translation.z-reference.z;
    transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*in, *out, transform);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::bottomBoxButtonPressed()
{
    if(viewer_->contains("object"))
    {
        pcl::PointXYZ min,max,middlePoint;
        pcl::getMinMax3D(*cloud_object,min,max);
        middlePoint.z=min.z;
        middlePoint.x=(max.x+min.x)/2;
        middlePoint.y=(max.y+min.y)/2;
        pointObject=true;
        std::cout<<"Got object point "<<middlePoint<<std::endl;
        p_ob=middlePoint;
        PointCloudC::Ptr boxPoint(new PointCloudC);
        pcl::PointXYZRGB p(0,255,255);
        p.x=p_ob.x;
        p.y=p_ob.y;
        p.z=p_ob.z;
        boxPoint->push_back(p);
        viewer_->addPointCloud(boxPoint,"boxPoint",0);
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"boxPoint");
        ui->qvtkWidget->update();
    }

}
void MainWindow::topBoxButtonPressed()
{
    if(viewer_->contains("object"))
    {
        pcl::PointXYZ min,max,middlePoint;
        pcl::getMinMax3D(*cloud_object,min,max);
        middlePoint.z=max.z;
        middlePoint.x=(max.x+min.x)/2;
        middlePoint.y=(max.y+min.y)/2;
        pointObject=true;
        std::cout<<"Got object point "<<middlePoint<<std::endl;
        p_ob=middlePoint;
        PointCloudC::Ptr boxPoint(new PointCloudC);
        pcl::PointXYZRGB p(0,255,255);
        p.x=p_ob.x;
        p.y=p_ob.y;
        p.z=p_ob.z;
        boxPoint->push_back(p);
        viewer_->addPointCloud(boxPoint,"boxPoint",0);
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"boxPoint");
        ui->qvtkWidget->update();
    }
    else
    {
        pcl::PointXYZ min,max,middlePoint;
        pcl::getMinMax3D(*cloud_scene,min,max);
        middlePoint.z=max.z;
        middlePoint.x=(max.x+min.x)/2;
        middlePoint.y=(max.y+min.y)/2;
        pointScene=true;
        std::cout<<"Got scene point "<<middlePoint<<std::endl;
        p_scene=middlePoint;
        PointCloudC::Ptr boxPoint(new PointCloudC);
        pcl::PointXYZRGB p(0,0,255);
        p.x=p_scene.x;
        p.y=p_scene.y;
        p.z=p_scene.z;
        boxPoint->push_back(p);
        viewer_->addPointCloud(boxPoint,"boxPointTop",0);
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"boxPointTop");
        ui->qvtkWidget->update();
        ui->okButton->setEnabled(true);
    }
}

void MainWindow::denseButtonPressed()
{
    std::string active_cloud=ui->cloudsBox->currentText().toStdString();
    if(active_cloud.compare("Scene")==0)
    {
        if(scene_file.find(".ply") != std::string::npos || scene_file.find(".obj") != std::string::npos)
        {
            PointCloudT::Ptr denserCloud=densecloud(scene_file,300000,1);
            pcl::copyPointCloud(*denserCloud,*cloud_scene);
            display("scene");
        }
        else
        {
            QMessageBox::information(this,tr("Confirmation Message"),tr("Only PLY/OBJ files to make dense"));
        }
    }
    else
    {
        if(active_cloud.compare("Object")==0)
        {
            if(obj_file.find(".ply") != std::string::npos || obj_file.find(".obj") != std::string::npos)
            {
                PointCloudT::Ptr denserCloud=densecloud(obj_file,100000,1);
                pcl::copyPointCloud(*denserCloud,*cloud_object);
                display("object");
            }
            else
            {
                QMessageBox::information(this,tr("Confirmation Message"),tr("Only PLY/OBJ files to make dense"));
            }
        }
        else
        {
            QMessageBox::information(this,tr("Confirmation Message"),tr("Only objects and scene can be dense"));
        }
    }
    ui->qvtkWidget->update ();
}
std::vector<int> MainWindow::sampleWithProbability(std::vector<float> *probabilities,int sampleSize, std::vector<sampledPoints> * aux){
    std::default_random_engine generator;
    std::discrete_distribution<int> distribution (probabilities->begin(),probabilities->end());
    int nrolls=2*probabilities->size();
    for (int i=0; i<nrolls; ++i)
    {
        int number = distribution(generator);
        aux->at(number).counter+=1;
    }
    int mycount = std::count_if (aux->begin(), aux->end(), IsZero);
    std::cout<<"\nNon zero elements "<<aux->size()-mycount<<std::endl;
    std::sort(aux->begin(),aux->end(),sortPointsProb);
    std::vector<int> v(sampleSize);
    for(int i=0;i<sampleSize;i++)
    {
        v.at(i)=aux->at((probabilities->size()-1)-i).id;
    }
    return v;
}

void MainWindow::on_actionLoad_triggered()
{
    loadFileButtonPressed();
}

void MainWindow::on_actionSave_triggered()
{
    saveFileButtonPressed();
}
void MainWindow::getSpinMatrix(pcl::PointCloud<PointWithVector>::Ptr sample, int orientations, pcl::PointCloud<pcl::PointXYZ>::Ptr full)
{
    pcl::PointCloud<PointWithVector>::iterator iter=sample->end();
    PointCloudT::Ptr relativePoints(new PointCloud);
    PointWithVector p=sample->at(sample->size()-2);
    pcl::PointXYZ refPointIBS(p.x,p.y,p.z);
    int refPointIBSId=int(p.v1);
    std::cout<<"ref: "<<refPointIBSId<<std::endl;
    pcl::PointXYZ ref(full->at(refPointIBSId).x-refPointIBS.x,full->at(refPointIBSId).y-refPointIBS.y,full->at(refPointIBSId).z-refPointIBS.z);
    pcl::PointXYZ ref2=full->at(refPointIBSId);

    //pcl::PointXYZ actual_ref()
    --iter;
    sample->erase(iter);
    --iter;
    sample->erase(iter);

    descriptor.resize(sample->size()*orientations,3);
    vectors.resize(sample->size(),3);
    PointCloud::Ptr xyz_target(new PointCloud);
    std::cout<<"REf: "<<ref<<std::endl;
    for(int j=0;j<sample->size();j++)
    {
        PointWithVector aP;
        aP.x=sample->at(j).x-ref.x;
        aP.y=sample->at(j).y-ref.y;
        aP.z=sample->at(j).z-ref.z;
        relativePoints->push_back(pcl::PointXYZ(aP.x,aP.y,aP.z));
        //descriptor.row(j)=aP.getVector3fMap();
        Eigen::Vector3f v(sample->at(j).v1,sample->at(j).v2,sample->at(j).v3);
        vectors.row(j)=v;
        //mags.at(j)=(v.norm()-v.norm()*.2)*(v.norm()-v.norm()*.2);
        //lengths.at(j)=1-((v.norm()- minW) * (1 - 0) / (maxW - minW) + 0);
        //xyz_target->push_back(pcl::PointXYZ(aP.x,aP.y,aP.z));
    }
    PointCloudC::Ptr anchor(new PointCloudC);
    pcl::PointXYZRGB coloredanchor(0,255,0);
    coloredanchor.x=ref.x;
    coloredanchor.y=ref.y;
    coloredanchor.z=ref.z;
    anchor->push_back(coloredanchor);
    //viewer->addPointCloud(anchor,"Anchor");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"Anchor");
    PointCloud::Ptr spinCloud(new PointCloud);
    PointCloud::Ptr relative_spin(new PointCloud);
    PointCloud::Ptr xyz_2(new PointCloud);
    pcl::copyPointCloud(*sample,*xyz_target);
    pcl::copyPointCloud(*xyz_target,*spinCloud);
    pcl::copyPointCloud(*relativePoints,*relative_spin);
    std::cout<<"Spining "<<xyz_2->size()<<" points"<<std::endl;
    //if(viewer->contains("Spincloud"))
    //  viewer->updatePointCloud(relative_spin,"Spincloud");
    //else
    //  viewer->addPointCloud(relative_spin,"Spincloud");
    //while(!viewer->wasStopped())
    //        viewer->spinOnce(100);
    //    viewer->resetStoppedFlag();

    for (int i=1;i<orientations;i++)
    {
        pcl::copyPointCloud(*sample,*xyz_2);
        rotateCloud(xyz_2,xyz_target, i*2*M_PI/orientations,'z',ref);
        *spinCloud+=*xyz_target;
        pcl::copyPointCloud(*relativePoints,*xyz_2);
        rotateCloud(xyz_2,xyz_target, i*2*M_PI/orientations,'z',true);
        *relative_spin+=*xyz_target;
        //std::cout<<"Spincloud: "<<xyz_target->size()<<std::endl;
        //if(viewer->contains("Spincloud"))
        //            viewer->updatePointCloud(relative_spin,"Spincloud");
        //        else
        //            viewer->addPointCloud(relative_spin,"Spincloud");
        //        while(!viewer->wasStopped())
        //            viewer->spinOnce(100);
        //        viewer->resetStoppedFlag();

    }
    for(int i=0;i<spinCloud->size();i++)
        descriptor.row(i)=Eigen::Vector3f(relative_spin->at(i).x,relative_spin->at(i).y,relative_spin->at(i).z);
    std::cout<<"Descriptor "<<descriptor.rows()<<std::endl;
}

PointCloud::Ptr MainWindow::densecloud(std::string input,int SAMPLE_POINTS,float leaf_size)
{
    // Parse command line arguments
    //int SAMPLE_POINTS_ = default_number_samples;
    //parse_argument (argc, argv, "-n_samples", SAMPLE_POINTS_);
    //float leaf_size = default_leaf_size;
    //parse_argument (argc, argv, "-leaf_size", leaf_size);
    //bool vis_result = ! find_switch (argc, argv, "-no_vis_result");

    // Parse the command line arguments for .ply and PCD files

    vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
    if(input.find(".ply") != std::string::npos)
    {
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFilePLY (input, mesh);
        pcl::io::mesh2vtk (mesh, polydata1);
    }
    else if (input.find(".obj") != std::string::npos)
    {
        vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New ();
        readerQuery->SetFileName (input.c_str());
        readerQuery->Update ();
        polydata1 = readerQuery->GetOutput ();
    }

    //make sure that the polygons are triangles!
    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
#if VTK_MAJOR_VERSION < 6
    triangleFilter->SetInput (polydata1);
#else
    triangleFilter->SetInputData (polydata1);
#endif
    triangleFilter->Update ();

    vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
    triangleMapper->Update();
    polydata1 = triangleMapper->GetInput();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
    uniform_sampling (polydata1, SAMPLE_POINTS, *cloud_1);
    return cloud_1;
}
void MainWindow::randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                                      Eigen::Vector4f& p)
{
    float r1 = static_cast<float> (uniform_deviate (rand ()));
    float r2 = static_cast<float> (uniform_deviate (rand ()));
    float r1sqr = sqrtf (r1);
    float OneMinR1Sqr = (1 - r1sqr);
    float OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = r1sqr * (r2 * c1 + b1) + a1;
    c2 = r1sqr * (r2 * c2 + b2) + a2;
    c3 = r1sqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
    p[3] = 0;
}

void MainWindow::randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p)
{
    float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

    std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
    vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());

    double A[3], B[3], C[3];
    vtkIdType npts = 0;
    vtkIdType *ptIds = NULL;
    polydata->GetCellPoints (el, npts, ptIds);
    polydata->GetPoint (ptIds[0], A);
    polydata->GetPoint (ptIds[1], B);
    polydata->GetPoint (ptIds[2], C);
    randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
            float (B[0]), float (B[1]), float (B[2]),
            float (C[0]), float (C[1]), float (C[2]), p);
}

void MainWindow::uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
    polydata->BuildCells ();
    vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();

    double p1[3], p2[3], p3[3], totalArea = 0;
    std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
    size_t i = 0;
    vtkIdType npts = 0, *ptIds = NULL;
    for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
    {
        polydata->GetPoint (ptIds[0], p1);
        polydata->GetPoint (ptIds[1], p2);
        polydata->GetPoint (ptIds[2], p3);
        totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
        cumulativeAreas[i] = totalArea;
    }

    cloud_out.points.resize (n_samples);
    cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
    cloud_out.height = 1;

    for (i = 0; i < n_samples; i++)
    {
        Eigen::Vector4f p;
        randPSurface (polydata, &cumulativeAreas, totalArea, p);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];
    }
}

void MainWindow::on_actionHow_to_triggered()
{

}
