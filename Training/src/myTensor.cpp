#include "myTensor.h"

Tensor::Tensor(bool plotdebug, float oSize, float sSize){

    sceneCloud.reset(new PointCloud);
    queryObjectCloud.reset(new PointCloud);
    bigCloud.reset(new PointCloud);
    auxCloud.reset(new PointCloud);
    ibs.reset(new PointCloud);
    ibs_parents.reset(new PointCloudIbs);
    sphere_rad=1;
    
}



void Tensor::prepareClouds(){
    // Object and scene are copied into the same pointcloud
    // to compute voronoi diagram over all points
    pcl::copyPointCloud(*queryObjectCloud,*bigCloud);
    ibs->clear();
    *bigCloud+=*auxCloud;
    cloud_coord.clear();
    copyCoordinates();
}

void Tensor::copyCoordinates(){
    double aux[3];
    // Loop through the cloud coordinates and copy them 
    // into a Coordinates object/array
    for(int i=0;i<bigCloud->size();i++)
    {
        aux[0] =bigCloud->at(i).x;
        aux[1] =bigCloud->at(i).y;
        aux[2] =bigCloud->at(i).z;
        cloud_coord.append(DIM,aux);
    }
}

void Tensor::qhull_allDistances(){
    Qhull qhull;
    Eigen::Vector4f centroid;
    pcl::PointXYZ min,max;
    pcl::compute3DCentroid(*queryObjectCloud,centroid);

    // this is the rad for clipping the voronoi diagram
    float RAD=0.5*sphere_rad*pcl::L2_Norm(minObj.getArray3fMap(),maxObj.getArray3fMap(),DIM);

    std::vector<pseudo_id> vertices_one;    //Voronoi vertices ids for one "cloud" -> queryobject
    std::vector<pseudo_id> vertices_two;    //Voronoi vertices ids for other "cloud" ->scene object
    int total_vertices;
    cout<<" Runing Qhull Voronoi"<<endl;
    int cloud1_size=queryObjectCloud->size();

    // Call the lib with -v flag to get voronoi diagram
    qhull.runQhull("",DIM,int(bigCloud->size()),&cloud_coord.at(0),"v");
    // Only way to read output data (that I find) is to read the output string and parse everything
    qhull.outputQhull("o");
    std::vector<pcl::PointXYZ> allVertices;
    cout<<"Done\nReading output "<<endl;
    int aux_count=0;

    //Start reading the output string and parse it

    if(qhull.hasQhullMessage()){
        std::string line;
        std::istringstream f(qhull.qhullMessage());
        int i=0,ignore=3,good_i=0;
        unsigned int read_regions=0;
        while(std::getline(f,line))
        {
            i++;
            if (i==1)   //First line is point dimension, don't use it now but could be useful
                continue;
            if (i==2)   //Second line is voronoi vertices and cloud vertices
            {
                std::string buf; // Have a buffer string
                std::stringstream ss(line); // Insert the line into a stream
                int dat=0;
                while(ss>>buf)  //The vertex coordinates
                {
                    dat++;
                    if(dat==1)
                    {
                        total_vertices=std::atoi(buf.c_str());
                        //std::cout<<"Vertices: "<<total_vertices<<"\n";
                    }

                }
            }
            if(i==3) //Third line is centre. Not used now
                continue;
            if (good_i>=total_vertices)    //Control reading vertices or regions
                read_regions=1;
            if (!read_regions)   //No regions, then get voronoi vertices coordinates
            {
                std::string buf; // Have a buffer string
                std::stringstream ss(line); // Insert the line into a stream
                Eigen::Vector4f coord;
                coord[3]=1;
                int coord_count=0;
                while(ss>>buf)  //The vertex coordinates
                {
                    float num= std::atof(buf.c_str());
                    coord[coord_count]=num;
                    //            cout<<num<<" ";
                    coord_count++;
                }
                pcl::PointXYZ pp(coord[0],coord[1],coord[2]);
                allVertices.push_back(pp);
                good_i++;
            }
            if(read_regions && aux_count<cloud1_size)   //Get id of voronoi vertices comprising each region of cloud 1
            {
                std::string buf; // Have a buffer string
                std::stringstream ss(line); // Insert the line into a stream
                int count=0,ignore=1;;
                while(ss>>buf)  //The vertex ids
                {
                    count++;
                    if (count<ignore)
                        continue;
                    pseudo_id a={aux_count,std::atoi(buf.c_str())};
                    vertices_one.push_back(a);
                }
                aux_count++;
            }
            if(read_regions && aux_count>=cloud1_size && aux_count<bigCloud->size())   //Get id of voronoi vertices comprising each region of cloud 2
            {
                std::string buf; // Have a buffer string
                std::stringstream ss(line); // Insert the line into a stream
                int count=0,ignore=1;;
                while(ss>>buf)  //The vertex ids
                {
                    count++;
                    if (count<ignore)
                        continue;
                    pseudo_id b={aux_count,std::atoi(buf.c_str())};
                    vertices_two.push_back(b);
                }
                aux_count++;
            }
        }
    }    
    cout<<bigCloud->size()<<" "<<aux_count<<"\n";
    
    qhull.clearQhullMessage();

    // At this point there are two sets of vertices ids
    // Ids comming from query object and scene object
    // We need to find commong ids and the points associated to
    // those vertices form the Bisector surface
    std::sort(vertices_one.begin(),vertices_one.end(),sortRegion);
    vertices_one.erase(std::unique(vertices_one.begin(),vertices_one.end(),uniqueRegion),vertices_one.end());
    std::sort(vertices_two.begin(),vertices_two.end(),sortRegion);
    vertices_two.erase(std::unique(vertices_two.begin(),vertices_two.end(),uniqueRegion),vertices_two.end());
    std::vector<pseudo_id> toSearch(vertices_one);
    toSearch.insert(toSearch.end(),vertices_two.begin(),vertices_two.end());
    std::sort(toSearch.begin(),toSearch.end(),sortRegion);
    // By now same ids (comming from different pointclouds) should be
    // together (one after the other) in the same container
    // Just need to loop through the container and detect 
    // same ids adjacent to each other.
    float sum=0;
    int k=0;

    float minD1=100,maxD1=0,minD2=100,minD3=100,minD4=100,maxD2=0,maxD3=0,maxD4=0;

    // Keep track of every distance combination for every point in the bisector surface
    std::cout<<"searching with all distances"<<std::endl;

    //    switch(distance)
    //    {
    //    case 0:std::cout<<" Distance to sampled point: "<<sampledPoint<<std::endl;
    //        break;
    //    case 1:std::cout<<" Distance to closest point in queryObject"<<std::endl;
    //        break;
    //    case 2:std::cout<<" Distance to both objects"<<std::endl;
    //        break;
    //    case 3:std::cout<<" Distance to query object centroid"<<std::endl;
    //        break;
    //    }
    distan.push_back(std::vector<float>());
    distan.push_back(std::vector<float>());
    distan.push_back(std::vector<float>());
    distan.push_back(std::vector<float>());
    while(!allVertices.empty())
    {
        if(k>=(toSearch.size()-1))
            break;
        int one=toSearch.at(k).region_id;
        int two=toSearch.at(k+1).region_id;
        if(one==two)    //Same ids
        {
            Eigen::Vector4f coord(allVertices.at(one).x,allVertices.at(one).y,allVertices.at(one).z,1);
            Eigen::Vector4f difference=coord-centroid;
            float mag=difference.norm();
            // check that point in the ibs is inside clipping sphere
            if(mag<=RAD)
            {
                pcl::PointXYZ point(coord[0],coord[1],coord[2]);
                ibsPointType aPoint;
                aPoint.x=coord[0];
                aPoint.y=coord[1];
                aPoint.z=coord[2];
                pcl::PointXYZ par1,par2;
                int cloud_id=toSearch.at(k).parent_id;
                int cloud_id2=toSearch.at(k+1).parent_id;

                if(cloud_id<cloud1_size) //id from query object
                {
                    par1=bigCloud->at(cloud_id);
                    aPoint.parent_id=cloud_id;
                    par2=bigCloud->at(cloud_id2);
                }
                else
                {
                    par1=bigCloud->at(cloud_id2);
                    aPoint.parent_id=cloud_id2;
                    par2=bigCloud->at(cloud_id);
                }
                ibs->push_back(point); //A vertex (PCL point) is pushed to the vector
                ibs_parents->push_back(aPoint);
                pcl::PointXYZ nC(centroid[0],centroid[1],centroid[2]);
                float d1=0,d2=0,dc=0;
                //distance to point
                dc=pcl::L2_Norm(sampledPoint.getArray3fMap(),aPoint.getArray3fMap(),DIM);
                distan.at(0).push_back(dc);
                if((dc)>maxD1)   maxD1=dc;
                if((dc)<minD1)   minD1=dc;

                //distance to query object point
                dc=pcl::L2_Norm(par1.getArray3fMap(),aPoint.getArray3fMap(),DIM);
                distan.at(1).push_back(dc);
                if((dc)>maxD2)   maxD2=dc;
                if((dc)<minD2)   minD2=dc;


                //Distance to query object and scene;
                d1=pcl::L2_Norm(par1.getArray3fMap(),aPoint.getArray3fMap(),DIM);
                d2=pcl::L2_Norm(par2.getArray3fMap(),aPoint.getArray3fMap(),DIM);
                dc=d1+d2;
                distan.at(2).push_back(dc);
                if((dc)>maxD3)   maxD3=dc;
                if((dc)<minD3)   minD3=dc;

                //Distance to centroid of queryobject;
                dc=pcl::L2_Norm(aPoint.getArray3fMap(),nC.getArray3fMap(),DIM);
                distan.at(3).push_back(dc);
                if((dc)>maxD4)   maxD4=dc;
                if((dc)<minD4)   minD4=dc;
            }
            k=k+2;
        }
        else
            k=k+1;
    }
    maxmin.push_back(pcl::PointXYZ(minD1,maxD1,0));
    maxmin.push_back(pcl::PointXYZ(minD2,maxD2,0));
    maxmin.push_back(pcl::PointXYZ(minD3,maxD3,0));
    maxmin.push_back(pcl::PointXYZ(minD4,maxD4,0));
    
    // In newer versions of qhull the next line is not needed
    // If working with an older version then uncomment to free memory
    //qhull.checkAndFreeQhullMemory();
}



void Tensor::extractCloud(std::vector<int> indices,PointCloud::Ptr inCloud, PointCloud::Ptr outCloud){
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr cIndices (new pcl::PointIndices);
    cIndices->indices=indices;
    extract.setInputCloud(inCloud);
    extract.setIndices(cIndices);
    extract.filter(*outCloud);
}


bool Tensor::getNNormals(const char *file, pcl::PointCloud<pcl::Normal>::Ptr normals, PointCloud::Ptr cloud){
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    std::string filename(file); //first command line argument
    std::cout << "Reading file " << filename << "..." << std::endl;
    vtkSmartPointer<vtkPLYReader> reader =vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( filename.c_str() );
    std::cout << "Reading " << filename << std::endl;
    reader->Update();
    polydata->DeepCopy(reader->GetOutput());
    std::cout << "Computing normals..." << std::endl;
    // Generate normals
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(polydata);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOff();
    normalGenerator->AutoOrientNormalsOn();
    normalGenerator->Update();
    //normalGenerator->FlipNormalsOn();
    /*
        // Optional settings
        normalGenerator->SetFeatureAngle(0.1);
        normalGenerator->SetSplitting(1);
        normalGenerator->SetConsistency(0);
        normalGenerator->SetAutoOrientNormals(0);
        normalGenerator->SetComputePointNormals(1);
        normalGenerator->SetComputeCellNormals(0);
        normalGenerator->SetFlipNormals(0);
        normalGenerator->SetNonManifoldTraversal(1);
       */
    polydata = normalGenerator->GetOutput();
    // Count points
    vtkIdType numPoints = polydata->GetNumberOfPoints();
    cout<<"Points "<<numPoints<<std::endl;
    for(int j=0;j<numPoints;j++)
    {
        double *point=polydata->GetPoint(j);
        pcl::PointXYZ aPoint(*point,*(point++),-*(point++));
        cloud->push_back(aPoint);
    }
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0.0, 0.0, 0.0;
    transform_2.rotate (Eigen::AngleAxisf (-M_PI/2, Eigen::Vector3f::UnitY()));
    pcl::transformPointCloud (*cloud, *cloud, transform_2);


    // Count triangles
    vtkIdType numPolys = polydata->GetNumberOfPolys();

    ////////////////////////////////////////////////////////////////
    // Double normals in an array
    vtkFloatArray* normalDataFloat =
            vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetArray("Normals"));
    if(normalDataFloat)
    {
        int nc = normalDataFloat->GetNumberOfTuples();
        for(int i=0;i<nc;i++)
        {
            double *point=normalDataFloat->GetTuple(i);
            pcl::Normal aNormal(*point,*(point++),*(point++));
            normals->push_back(aNormal);
        }
        //std::cout<<"X: "<<*point<<" Y: "<<*(point++)<<" Z: "<<*(point++)<<"\n";
        std::cout << "\nThere are " << nc
                  << " components in normalDataFloat";
        std::cout<<" done\n";
        return true;
    }
    std::cout << "Normals not found!" << std::endl;
    return false;
}
