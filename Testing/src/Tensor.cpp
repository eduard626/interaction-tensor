#include "Tensor.h"

Tensor::Tensor(int argC,char ** argV){
	//Parse input data: object, scene;
	sceneCloud.reset(new PointCloud);
	queryObjectCloud.reset(new PointCloud);
	targetCloud.reset(new PointCloud);
	auxCloud.reset(new PointCloud);

	//If not enough args provided
	if(argC!=4)
	{
		printUsage();
		exit(1);
	}

	// check file exists
	std::cout<<"Loading data"<<std::endl;
	int error_reading;
	// Load scene and object pointclouds, descriptorss and everything from disk
	error_reading=loadData(std::string(argV[1]),std::string(argV[3]),std::string(argV[2]));
	if(error_reading)
	{
		cout<<"Error reading files\n";
		exit(1);
	}

	converged=false;
	pcl::getMinMax3D(*sceneCloud,minScene,maxScene);
	sceneSize=pcl::L2_Norm(minScene.getArray3fMap(),maxScene.getArray3fMap(),DIM);
	pcl::getMinMax3D(*object_clouds.at(0),minObj,maxObj);
	// For voxel extraction and NN search
	desiredSize=originalSize=pcl::L2_Norm(minObj.getArray3fMap(),maxObj.getArray3fMap(),DIM);
	std::cout<<"desired size: "<<desiredSize<<std::endl;
	
	// If PLY/OBJ scene
	if(pointCloudType==0)
	{
		std::cout<<"Creating octree...";
		std::string denser_cloud=file_name+scn_name+"_d.pcd";
		cellCentres.reset(new PointCloud);
		if(pcl::io::loadPCDFile(denser_cloud.c_str(),*cellCentres)!=0)
		{
			std::cout<<"No dense cloud found "<<denser_cloud<<std::endl;
			std::cout<<"Continue using "<<argV[3]<<" [0-No/1-Yes] ?: ";
			int conti=0;
			std::cin>>conti;
			if (conti<1)
				exit(1);
			else
				cellCentres=sceneCloud;
		}
		std::cout<<"Read denser cloud with "<<cellCentres->size()<<" points"<<std::endl;
		octreeCentres.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(true));
		octreeCentres->setResolution(originalSize*2);
		octreeCentres->setInputCloud (cellCentres);
		octreeCentres->defineBoundingBox();
		octreeCentres->addPointsFromInputCloud ();
		std::cout<<"done"<<std::endl;

	}
	else
	{
		std::cout<<"Creating octree...";
		octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(true));
		octree->setResolution(originalSize*2);
		octree->setInputCloud (sceneCloud);
		octree->defineBoundingBox();
		octree->addPointsFromInputCloud ();
		std::cout<<"done"<<std::endl;
	}

#ifndef  TEXT_ONLY
	// Initialize viewer
	if(plotDebug>1)
	{
		std::cout<<"Instance viewer "<<std::endl;
		viewer.reset(new pcl::visualization::PCLVisualizer("Debug"));
		std::cout<<"done"<<std::endl;
	}
#endif
}


void Tensor::printObjInfo(){
	cout<<"\nCurrent iT object\n";
	cout<<"Sample size: "<<sampleSize<<"\n";
	cout<<"Scene cloud size: "<<sceneCloud->size()<<"\n";
	cout<<"Object cloud size: "<<queryObjectCloud->size()<<"\n";
	cout<<"Plot debug mode: "<<plotDebug<<"\n";
	cout<<"Metric: "<<metric<<std::endl;
}

int Tensor::loadData(std::string affordance, std::string scene_file, std::string obj_file){
	// First thing is to find the real path of the input scene. It should be under data/ dir
	// or potentially in some other location as long as there is also all the data
	// like descriptors for affordances, etc

	// If there are no slashes in the name assume data/ dir
	if (scene_file.find("/")==std::string::npos)
	{
		scene_file="../data/"+scene_file;
	}
	std::string command="readlink -f "+scene_file;
	std::string patht=exec(command.c_str());
	
	std::size_t found_slash=patht.find_last_of("/")+1;
	file_name=patht.substr(0,found_slash);

	// Some parameters needed
	// Currently only line 1,3,4 are used
	// Remaining lines are used in different versions, these are kept for compatibilty
	command=file_name+"parameters.txt";
	std::ifstream f(command.c_str());
	if(f.is_open())
	{
		std::string line;
		int n_line=0;

		while ( std::getline (f,line) )
		{
			//Line 0 - Weight: Sample point, distance to query, two objects, centroid
			//Line 1 - Method: full ICP, slices, 3D, Naive
			//Line 2 - N. of slices
			//Line 3 - Scene sample size
			//Line 4 - Debug On/Off
			//Line 5 - Agglomerarive desriptor ID
			//Line 6 - Vector Therhold
			//Line 7 - Perform and save local search/gradient
			std::size_t number=line.find(" ")+1;
			switch(n_line)
			{
			case 0: std::istringstream(line.substr(number))>>metric;
			//std::cout<<metric<<std::endl;
			break;
			case 1:
				if(line.substr(number)=="0")
					method_name="Slices";
				if(line.substr(number)=="1")
					method_name="Full";
				if(line.substr(number)=="2")
					method_name="3D";
				if(line.substr(number)=="3")
					method_name="Naive";
				//std::cout<<line.substr(number)<<std::endl;
				break;
			case 2:
				std::istringstream(line.substr(number))>>nOrientations;
				//          std::cout<<nSlices<<std::endl;
				break;
			case 3:
				float aux;
				std::istringstream(line.substr(number))>>aux;
				sampleSize=aux/100;
				std::cout<<sampleSize<<std::endl;
				break;
			case 4:
				std::istringstream(line.substr(number))>>plotDebug;
				break;
			case 5:
				n_descriptor=line.substr(number);
				std::istringstream(line.substr(number))>>n_descriptor_id;
				break;
			case 6:
				float aux2;
				std::istringstream(line.substr(number))>>aux2;
				agg_th=aux2/100;
				break;
			case 7:
				std::istringstream(line.substr(number))>>localResponse;
				break;
			case 8:
				float aux_prediction;
				std::istringstream(line.substr(number))>>aux_prediction;
				pred_t=aux_prediction/100;
				break;
			default:
				std::cout<<"Not handled"<<std::endl;
				break;
			}
			n_line++;
		}
		f.close();
		std::cout<<"Parameters file OK"<<std::endl;
	}
	else
	{
		std::cout<<"parameters.txt file not found"<<std::endl;
		exit(1);
	}
	
	//Orientations in descriptor to predict: 8
	
	std::stringstream orientations;
	orientations<<nOrientations;


	// Single-affordance prediction therefore dataSize=1
	int dataSize=1;

	affordance_name.resize(dataSize);
	affordance_name.at(0)=affordance;
	ob_names.resize(dataSize);

	// Some previous code needs object file extension
	if (obj_file.find(".ply")!=std::string::npos)
	{
		// Called with object.ply
		// Probably from older code
		std::vector<std::string> aux;
		Tokenize(obj_file,aux,".");
		ob_names.at(0)=aux.at(0);
	}
	else
	{
		ob_names.at(0)=obj_file;
	}

	// Path to data by default under "Affordance" dir
	std::string dir;
	if(plotDebug>1)
		std::cout<<"Affordances "<<dataSize<<std::endl;
	
	dir=file_name+affordance_name.at(0)+"/";

	// To read training data such as reference points
	std::string aux_n=dir+"ibs_full_"+affordance_name.at(0)+"_"+ob_names.at(0)+".txt";
	// Query-object mesh
	std::string ob_mesh_file=dir+ob_names.at(0)+".ply";
	pcl::PolygonMesh aMesh;
#if VTK_MAJOR_VERSION < 6
	if(plotDebug>1)
		std::cout<<"Trying to read "<<ob_mesh_file<<std::endl;
	pcl::io::loadPolygonFile(ob_mesh_file, aMesh);
#else
	if(plotDebug>1)
		std::cout<<"Trying to read "<<ob_mesh_file<<std::endl;
	pcl::io::loadPolygonFilePLY(ob_mesh_file, aMesh);
#endif
	if(aMesh.cloud.width==0)
	{
		std::cout<<" Problem loading object in "<<ob_mesh_file<<std::endl;
		exit(1);
	}
	object_meshes.push_back(aMesh);

	// Also save the pointclouds associated to mesh
	PointCloud::Ptr objCloud;
	object_clouds.push_back(objCloud);
	object_clouds.at(0).reset(new PointCloud);
	pcl::fromPCLPointCloud2(aMesh.cloud,*object_clouds.at(0));

	// To align the object relative to the test-point in the scene
	// this data comes from training and its read from txt file
	original_points.reset(new PointCloud);
	toObject.reset(new PointCloud);
	
	// Read data from training *.txt file

	std::vector<float> dataPoints=getTemplateData(aux_n,command);

	// Workout the transformation applied to query-object during training and store it
	Eigen::Affine3f transform2=Eigen::Affine3f::Identity();
	transform2.translation()<<-dataPoints.at(6),-dataPoints.at(7),-dataPoints.at(8);
	transform2.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
	pcl::PCLPointCloud2 vertices;
	rotateCloud(object_clouds.at(0),object_clouds.at(0),dataPoints.at(9),'z',false);
	pcl::PointXYZ target,original,vectorToObj;
	vectorToObj.x=dataPoints.at(3);
	vectorToObj.y=dataPoints.at(4);
	vectorToObj.z=dataPoints.at(5);
	target.x=dataPoints.at(0)+vectorToObj.x;
	target.y=dataPoints.at(1)+vectorToObj.y;
	target.z=dataPoints.at(2)+vectorToObj.z;
	original=pcl::transformPoint(target,transform2);
	pcl::toPCLPointCloud2(*object_clouds.at(0),vertices);
	pcl::copyPointCloud(vertices,object_meshes.at(0).cloud);
	original_points->push_back(original);
	toObject->push_back(vectorToObj);
	
	// For normalization count the keypoints
	Eigen::MatrixXf data_counts_ori;
	// All relevant descriptor files
	std::string vectors_file,kp_file,extra_file,members_file,points_file,v_data;
	std::string point_data_name;
	
	// Provenance vectors
	vectors_file=dir+"UNew_"+affordance_name.at(0)+"_"+ob_names.at(0)+"_descriptor_8_vectors.pcd";
	// Affordance keypoints
	kp_file=dir+"UNew_"+affordance_name.at(0)+"_"+ob_names.at(0)+"_descriptor_8.pcd";
	// Keypoints per orientation for score nomalization
	point_data_name=dir+affordance_name.at(0)+"_"+ob_names.at(0)+"_point_count.dat";
	// Data about affordance id, orientation id and provenance vector id associated to every keypoint in descriptor
	extra_file=dir+"UNew_"+affordance_name.at(0)+"_"+ob_names.at(0)+"_descriptor_8_extra.pcd";
	// This is mostly useful for multiple-affordance. For single affordance is "1 keypoint per keypoint", makes more sense in multiple-affordances
	members_file=dir+"UNew_"+affordance_name.at(0)+"_"+ob_names.at(0)+"_descriptor_8_members.pcd";
	// For single-affordance this is equivalent to the descriptor
	points_file=dir+"UNew_"+affordance_name.at(0)+"_"+ob_names.at(0)+"_descriptor_8_points.pcd";
	// Provenance vector data sucha as original magnitudes and weights
	v_data=dir+"UNew_"+affordance_name.at(0)+"_"+ob_names.at(0)+"_descriptor_8_vdata.pcd";

	// Point counds for normalization
	std::ifstream ifs (point_data_name, std::ifstream::in);
	if(ifs.is_open())
	{
		std::cout<<"Trying to read"<<point_data_name<<std::endl;;
		data_counts.resize(affordance_name.size(),nOrientations);
		data_counts_ori.resize(affordance_name.size(),nOrientations);
		pcl::loadBinary(data_counts,ifs);
		data_counts_ori=data_counts;
		//data_counts.transposeInPlace();
		//data_counts_ori.transposeInPlace();
		data_counts=data_counts.cwiseInverse();
		std::cout<<" size: "<<data_counts.rows()<<","<<data_counts.cols()<<std::endl;
		std::cout<<" done"<<std::endl;
	}
	else
	{
		std::cout<<point_data_name<<" not found"<<std::endl;
		exit(1);
	}
	// Read all the previous files into pointclouds
	PointCloud::Ptr aux_cloud(new PointCloud);
	std::cout<<"Trying to read "<<vectors_file;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(vectors_file.c_str(), *aux_cloud) != 0)
	{
		cout<<"File not found: "<<vectors_file<<std::endl;
		return -1;
	}
	std::cout<<"...done."<<std::endl;
	largeVectors.resize(aux_cloud->size(),3);
	largeVectors=aux_cloud->getMatrixXfMap(3,4,0).transpose();

	largeDescriptorCloud.reset(new PointCloud);
	std::cout<<"Trying to read "<<kp_file;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(kp_file.c_str(), *largeDescriptorCloud) != 0)
	{
		cout<<"File not found: "<<kp_file<<std::endl;
		return -1;
	}
	std::cout<<"...done."<<std::endl;
	spinCloud.reset(new PointCloud);
	std::cout<<"Trying to read "<<points_file;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(points_file.c_str(), *spinCloud) != 0)
	{
		cout<<"File not found: "<<points_file<<std::endl;
		return -1;
	}
	std::cout<<"...done."<<std::endl;
	largeData.reset(new PointCloud);
	std::cout<<"Trying to read "<<extra_file;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(extra_file.c_str(), *largeData) != 0)
	{
		cout<<"File not found: "<<extra_file<<std::endl;
		return -1;
	}
	std::cout<<"...done."<<std::endl;
	largeIds.reset(new PointCloud);
	std::cout<<"Trying to read "<<members_file;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(members_file.c_str(), *largeIds) != 0)
	{
		cout<<"File not found: "<<members_file<<std::endl;
		return -1;
	}
	std::cout<<"...done."<<std::endl;
	std::cout<<"Reading point data ids...";
	largeDescriptor.resize(largeDescriptorCloud->size(),3);
	std::cout<<"LargeDescriptor "<<largeDescriptor.rows()<<std::endl;
	for(int k=0;k<largeDescriptorCloud->size();k++)
	{
		largeDescriptor.row(k)=largeDescriptorCloud->at(k).getVector3fMap();
		ppCentroid.push_back(int(largeIds->at(k).x));
		startppCentroid.push_back(int(largeIds->at(k).y));
	}
	std::cout<<"done."<<std::endl;

	//Check data counts is same
	if(plotDebug>1)
	{
		std::cout<<data_counts_ori<<std::endl;
		std::cout<<data_counts<<std::endl;
	}

	alternative_data_counts=Eigen::MatrixXf::Zero(data_counts_ori.rows(),data_counts_ori.cols());
	for(int k=0;k<largeData->size();k++)
	{
		int row=0;
		if (affordance_name.size()>1)
			row=int(largeData->at(k).x)-1;
		int col=int(largeData->at(k).y);
		data_counts_ori(row,col)-=1;
	}
	//After previous step, matrix should have all zeros.
	float max_c=data_counts_ori.maxCoeff();
	float min_c=data_counts_ori.minCoeff();
	if(std::fabs(min_c)+std::fabs(max_c)>0)
	{
		std::cout<<"Data count did not check out"<<std::endl;
		std::cout<<"Verify point_count.dat and descriptor_extra.pcd"<<std::endl;
		exit(1);
	}
	PointCloud::Ptr vdata;
	std::cout<<"Getting weights and vector mags from file"<<std::endl;
	vdata.reset(new PointCloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(v_data.c_str(), *vdata) != 0)
	{
		cout<<"File not found: "<<v_data<<std::endl;
		return -1;
	}
	
	for(int k=0;k<largeVectors.rows();k++)
	{
		large_mags.push_back(vdata->at(k).x);
		float remap=vdata->at(k).y;
		large_lengths.push_back(remap);
		int row=0;
		if (affordance_name.size()>1)
			row=int(largeData->at(k).x)-1;
		int col=int(largeData->at(k).y);
		//std::cout<<col<<","<<row<<std::endl;
		alternative_data_counts(row,col)+=remap;
	}
	alternative_data_counts=alternative_data_counts.cwiseInverse();

	//If got here, then read scene file
	
	std::cout<<"File name "<<file_name<<std::endl;
	std::size_t found_dot=patht.find_last_of(".");
	scn_name=patht.substr(found_slash,found_dot-found_slash);
	
	if (scene_file.find(".ply")!=std::string::npos)
	{
		//file is synthetic
		std::cout<<"Synthetic scene file"<<std::endl;
		pointCloudType=0;
	}
	else
	{
		std::cout<<"Real scene file"<<std::endl;
		pointCloudType=1;
	}
	
		//It this point is reached then is a basic PCD file with no more info.
	if(pointCloudType==1)
	{
		std::string filee=file_name+scn_name+".pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(filee.c_str(), *sceneCloud) != 0)
		{
			cout<<"Scene file not found: "<<patht<<std::endl;;
			return -1;
		}
	}
	else
	{
		std::string filee=file_name+scn_name+".ply";
		if (pcl::io::loadPLYFile(filee, *sceneCloud) != 0)
		{
			cout<<"Scene file not found: "<<patht<<std::endl;
			return -1;
		}
	}
	return 0;

}
void Tensor::printUsage(){
	std::cout<<"Not enough parameters"<<std::endl;
	std::cout<<"Usage:\n"<<std::endl;
	std::cout<<"./tensorTest Affordance Object Scene"<<std::endl;
	std::cout<<" For Instance:\n ./tensorTest Place bowl kitchen5.ply"<<std::endl;
}
void Tensor::init(){
	std::cout<<"In init"<<std::endl;
	if(plotDebug>1)
	{
		viewer->addCoordinateSystem(0.1,"id",0);
		viewer->setSize(1280,720);
		plot(SCENE,192,192,192);
	}
	randomSampleSize=int(sampleSize*sceneCloud->size());
	randomSample.setSample(randomSampleSize);
}

void Tensor::sampleCloud(){
	randomSampleCloud.reset(new PointCloud);
	randomSample.filter(*randomSampleCloud);
	randomSample.filter(randomSampleIdx);
	std::srand ( unsigned ( std::time(0) ) );
	std::random_shuffle(randomSampleIdx.begin(),randomSampleIdx.end());
	randomSampleIdx_original=randomSampleIdx;
}

std::string Tensor::saveClouds(PointCloudC::Ptr points,PointCloudC::Ptr data)
{
	//Save data of last run
	saveCloudC.reset(new PointCloudC);
	std::stringstream ss;
	std::string abs_path(file_name);
	if(converged)
	{
		sampleColor.reset(new PointCloudC);
		pcl::copyPointCloud(*randomSampleCloud,*sampleColor);
		for(int i=0;i<sampleColor->size();i++)
		{
			sampleColor->at(i).r=255;
			sampleColor->at(i).g=0;
			sampleColor->at(i).b=255;
		}
		goodColor.reset(new PointCloudC);
		pcl::copyPointCloud(*points,*goodColor);
		for(int i=0;i<goodColor->size();i++)
		{
			goodColor->at(i).r=255;
			goodColor->at(i).g=0;
			goodColor->at(i).b=0;
		}
		//File name code: affordance_object_scene_method_sampleSize.pcd
		time_t now = time(0);

		ss<< nOrientations;
		file_name=abs_path+affordance_name.at(0)+"_"+ob_names.at(0)+"_"+scn_name+"_"+method_name+"_"+ss.str();
		ss.str(std::string());
		ss<< int(pred_t*100);
		file_name=file_name+"_"+ss.str();
		ss.str(std::string());
		file_name=file_name+"_"+n_descriptor;
		ss.str(std::string());
		ss<< now;
		file_name=file_name+"_"+ss.str()+".pcd";
		*saveCloudC+=*sampleColor;
		*saveCloudC+=*goodColor;
		std::cout<<"File name example: "<<file_name<<"\n";
		pcl::io::savePCDFileBinaryCompressed(file_name,*saveCloudC);
		file_name=abs_path+ss.str()+"_samplePoints.pcd";
		pcl::io::savePCDFileBinaryCompressed(file_name,*sampleColor);
		file_name=abs_path+ss.str()+"_goodPoints.pcd";
		pcl::io::savePCDFileBinaryCompressed(file_name,*data);
		// goodPointsX.pcd has number predictions per test-point, their score and orientations
		file_name=abs_path+ss.str()+"_goodPointsX.pcd";
		pcl::io::savePCDFileBinaryCompressed(file_name,*points);
	}
	else
	{
		//If did not find anything
		time_t now = time(0);
		ss<< nOrientations;
		file_name+=affordance_name.at(0)+"_"+ob_names.at(0)+"_NoConv_"+scn_name+"_"+method_name+"_"+ss.str();
		ss.str(std::string());
		ss<< int(pred_t*100);
		file_name=file_name+"_"+ss.str();
		ss.str(std::string());
		file_name=file_name+"_"+n_descriptor;
		ss.str(std::string());
		ss<< now;
		file_name=file_name+"_"+ss.str()+".pcd";
		std::cout<<"File name example: "<<file_name<<"\n";
		std::string command="touch "+file_name;
		system(command.c_str());
		sampleColor.reset(new PointCloudC);
		pcl::copyPointCloud(*randomSampleCloud,*sampleColor);
		for(int i=0;i<sampleColor->size();i++)
		{
			sampleColor->at(i).r=255;
			sampleColor->at(i).g=0;
			sampleColor->at(i).b=255;
		}
		file_name=ss.str()+"_samplePoints.pcd";
		pcl::io::savePCDFileBinaryCompressed(file_name,*sampleColor);
	}
	return ss.str();
}

// Shows in viewer main pointclouds like scene, query-object and sample
void Tensor::plot(int id,int r,int g, int b){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr c2_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	char const *name;
	
	if(id==OBJECT )
	{
		pcl::copyPointCloud(*queryObjectCloud,*c2_color);
		name="Object";
		cout<<"Object\n";
	}
	
	if(id==SCENE)
	{
		pcl::copyPointCloud(*sceneCloud,*c2_color);
		name="Scene";
		cout<<"scene\n";
	}
	
	if(id==RNDSAMPLE)
	{
		pcl::copyPointCloud(*randomSampleCloud,*c2_color);
		name="RandomSample";
		cout<<"sample\n";
	}
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> c2_handler(c2_color,r,g,b);
	if(std::strcmp(name,"Normals")!=0)
	{
		if(viewer->contains(name))
		{
			viewer->updatePointCloud(c2_color,c2_handler,name);
		}
		else
		{
			viewer->addPointCloud(c2_color,c2_handler,name);
		}
		if(std::strcmp(name,"RandomSample")==0 || std::strcmp(name,"Heatmap")==0 || std::strcmp(name,"IBS")==0)
		{
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,name);
		}

	}
}
// Shows in viewer a poincloud given as input with name and rgb color and a pointsize
void Tensor::plot(PointCloud::Ptr cloud,char const * name, int r,int g, int b, int pointSize){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr c2_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud,*c2_color);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> c2_handler(c2_color,r,g,b);
	int size=pointSize;
	if (pointSize>10 || pointSize<1)
		size=1;
	if(viewer->contains(name))
	{
		viewer->updatePointCloud(c2_color,c2_handler,name);
	}
	else
	{
		viewer->addPointCloud(c2_color,c2_handler,name);
	}
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size,name);
}

void Tensor::plot(pcl::PolygonMesh &object,char const *name,int r,int g, int b)
{
	if (viewer->contains(name))
		viewer->updatePolygonMesh(object,name);
	else
		viewer->addPolygonMesh(object,name,0);
	double rr=double(r)/255,gg=double(g)/255,bb=double(b)/255;
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,rr,gg,bb,name);
}

// Shows in viewer a poincloud given as input with a name and pointsize
void Tensor::plot(PointCloudC::Ptr cloud,char const * name, int pointSize){
	int size=pointSize;
	if (pointSize>10 || pointSize<1)
		size=1;
	if(viewer->contains(name))
	{
		viewer->updatePointCloud(cloud,name);
	}
	else
	{
		viewer->addPointCloud(cloud,name);
	}
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size,name);
}

// Extract voxels od pointclouds from a larger pointcloud
void Tensor::extractCloud(std::vector<int> indices,PointCloud::Ptr inCloud, PointCloud::Ptr outCloud){
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr cIndices (new pcl::PointIndices);
	cIndices->indices=indices;
	extract.setInputCloud(inCloud);
	extract.setIndices(cIndices);
	extract.filter(*outCloud);
}

// Translate pointcloud with translation vector encoded in a PoinXYZ coordinates
void Tensor::translateCloud(PointCloud::Ptr in, PointCloud::Ptr out, pcl::PointXYZ translation){
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*in,centroid);
	transform.translation() << translation.x-centroid[0],translation.y-centroid[1],translation.z-centroid[2];
	transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud (*in, *out, transform);
}

// Translate pointcloud using a reference point. Computes the transformation neede to take a reference point to a new poistion called "translation"
void Tensor::translateCloud(PointCloud::Ptr in, PointCloud::Ptr out, pcl::PointXYZ translation,pcl::PointXYZ reference){
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << translation.x-reference.x,translation.y-reference.y,translation.z-reference.z;
	transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud (*in, *out, transform);
}

// Rotate pointcloud around an axis, relative to world origin or relative to pointcloud centroid
void Tensor::rotateCloud(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, float angle, char axis,bool origin){
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

// Rotate pointcloud around and axis relative to an "anchor" point or pivot
void Tensor::rotateCloud(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, float angle, char axis,pcl::PointXYZ pivot){
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
	Eigen::Vector3f ax;
	int id=cloud_in->size();
	cloud_in->push_back(pivot);
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
	translateCloud(cloud_out,cloud_out,pivot,newPivot);
	PointCloud::iterator iter=cloud_out->end();
	--iter;
	cloud_out->erase(iter);
}

// Tokenize as string using delimiters
void Tensor::Tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ")
{
	// Skip delimiters at beginning.
	std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

// Reads data from training data file
std::vector<float> Tensor::getTemplateData(std::string file_txt, std::string &scene_name_file){
	//output:
	// 3 first elements are scene ref point coordinates
	// 3 elements vector from scene point to obj
	// 3 elements for object translation
	// 1 element for rotation in Z axis
	// 3 elements for boundingBox centroid translation
	// 3 elements for vector scene to boundingBox centroid.
	float rotation;
	std::vector<float> out;
	std::string scene_name;
	pcl::PointXYZ translation,refPoint,toObj,translationBox,toBox;
	std::ifstream f(file_txt.c_str());
	if(f.is_open())
	{
		std::string line;
		//std::cout<<"Parameters file OK"<<std::endl;
		int n_line=0;

		while ( std::getline (f,line) )
		{
			std::vector<std::string> line_elem;
			if(n_line<3)
			{
				if(n_line==0)
				{
					Tokenize(line,line_elem,":");
					scene_name=line_elem.at(1);
					scene_name_file=line_elem.at(1);
					//std::cout<<"Scene name: "<<scene_name_file<<std::endl;
				}
				if(n_line==1)
				{
					Tokenize(line,line_elem,":");
					//object_name=line_elem.at(1);
					//          if(object_name.compare("mug1")!=0)
					//            object_name_file=line_elem.at(1)+".ply";
					//          else
					//            object_name_file=line_elem.at(1)+".pcd";
					//          std::cout<<"Obj name: "<<object_name_file<<std::endl;
				}
				if(n_line==2)
				{
					Tokenize(line,line_elem,":");
					int n_clusters;
					std::istringstream(line_elem.at(1))>>n_clusters;
					//std::cout<<"Clusters: "<<n_clusters<<std::endl;
					std::vector<std::string>().swap(line_elem);
					for(int j=0;j<n_clusters;j++)
					{
						std::getline (f,line);
						Tokenize(line,line_elem,",");
						float data[3];
						for(int i=0;i<line_elem.size();i++)
						{
							std::istringstream(line_elem.at(i))>>data[i];
						}
						//            pcl::PointXYZ point(data[0],data[1],data[2]);
						//            cluster_centres.push_back(point);
						//            std::cout<<point<<std::endl;
						line_elem.clear();
					}
				}
			}
			if(n_line==3)
			{
				Tokenize(line,line_elem,":");
				//        std::istringstream(line_elem.at(1))>>distance_th;
				//        std::cout<<"Distance threshold: "<<distance_th<<std::endl;
			}
			if(n_line==4)
			{
				Tokenize(line,line_elem,":");
				//        reference=line_elem.at(1);
				//        std::cout<<"Reference: "<<reference<<std::endl;
			}
			if(n_line==5)
			{
				Tokenize(line,line_elem,",");
				//                float data[3];
				//                for(int i=0;i<line_elem.size();i++)
				//                {
				//                  std::istringstream(line_elem.at(i))>>data[i];
				//                }
				//                refPoint.x=data[0];
				//                refPoint.y=data[1];
				//                refPoint.z=data[2];
				//                std::cout<<refPoint<<std::endl;
			}
			if(n_line==6)
			{
				//skip
				std::getline (f,line);
				Tokenize(line,line_elem,":");
				std::vector<std::string> tmp;
				Tokenize(line_elem.at(1),tmp,",");
				float data[3];
				for(int i=0;i<tmp.size();i++)
				{
					std::istringstream(tmp.at(i))>>data[i];
				}
				refPoint.x=data[0];
				refPoint.y=data[1];
				refPoint.z=data[2];
				//std::cout<<refPoint<<std::endl;
			}
			if(n_line==7)
			{
				std::getline (f,line);
				Tokenize(line,line_elem,":");
				//        std::istringstream(line_elem.at(0))>>ibs_id;
				//        std::cout<<"Ibs id: "<<ibs_id<<std::endl;
			}
			if(n_line==8)
			{
				std::getline (f,line);
				Tokenize(line,line_elem,":");
				std::vector<std::string> coord;
				Tokenize(line_elem.at(1),coord,",");
				std::istringstream(coord.at(0))>>toObj.x;
				std::istringstream(coord.at(1))>>toObj.y;
				std::istringstream(coord.at(2))>>toObj.z;
			}
			if(n_line==9)
			{
				std::getline (f,line);
				Tokenize(line,line_elem,",");
				float data[3];
				for(int i=0;i<line_elem.size();i++)
				{
					std::istringstream(line_elem.at(i))>>data[i];
				}
				translation.x=data[0];
				translation.y=data[1];
				translation.z=data[2];
				//std::cout<<"Object transformation"<<std::endl;
				//  std::cout<<translation<<std::endl;
			}
			if(n_line==10)
			{
				std::istringstream(line)>>rotation;
				//std::cout<<rotation<<std::endl;
			}
			if(n_line==11)
			{
				std::getline (f,line);	//skip text
				Tokenize(line,line_elem,",");
				float data[3];
				for(int i=0;i<line_elem.size();i++)
				{
					std::istringstream(line_elem.at(i))>>data[i];
				}
				translationBox.x=data[0];
				translationBox.y=data[1];
				translationBox.z=data[2];
				//std::cout<<"Box translation"<<std::endl;
				//std::cout<<translationBox<<std::endl;
			}
			if(n_line==12)
			{
				std::getline (f,line);	//skip text
				Tokenize(line,line_elem,",");
				float data[3];
				for(int i=0;i<line_elem.size();i++)
				{
					std::istringstream(line_elem.at(i))>>data[i];
				}
				toBox.x=data[0];
				toBox.y=data[1];
				toBox.z=data[2];
				//std::cout<<"Scene to box"<<std::endl;
				//std::cout<<toBox<<std::endl;
			}
			n_line++;
		}
		f.close();
	}
	else
	{
		std::cout<<file_txt<<" file not found"<<std::endl;
		rotation=0;
	}
	out.push_back(refPoint.x);
	out.push_back(refPoint.y);
	out.push_back(refPoint.z);
	out.push_back(toObj.x);
	out.push_back(toObj.y);
	out.push_back(toObj.z);
	out.push_back(translation.x);
	out.push_back(translation.y);
	out.push_back(translation.z);
	out.push_back(rotation);
	out.push_back(translationBox.x);
	out.push_back(translationBox.y);
	out.push_back(translationBox.z);
	out.push_back(toBox.x);
	out.push_back(toBox.y);
	out.push_back(toBox.z);
	return out;
}

// A simpler version to read training data that only returns an angle
float Tensor::getTemplateData(std::string file_txt){
	float rotation;
	std::ifstream f(file_txt.c_str());
	if(f.is_open())
	{
		std::string line;
		//    std::cout<<"Parameters file OK"<<std::endl;
		int n_line=0;

		while ( std::getline (f,line) )
		{
			std::vector<std::string> line_elem;
			if(n_line<3)
			{
				if(n_line==0)
				{
					Tensor::Tokenize(line,line_elem,":");
					//scene_name=line_elem.at(1);
					//scene_name_file=line_elem.at(1)+".ply";
					//std::cout<<"Scene name: "<<scene_name_file<<std::endl;
				}
				if(n_line==1)
				{
					Tensor::Tokenize(line,line_elem,":");
					//object_name=line_elem.at(1);
					//          if(object_name.compare("mug1")!=0)
					//            object_name_file=line_elem.at(1)+".ply";
					//          else
					//            object_name_file=line_elem.at(1)+".pcd";
					//          std::cout<<"Obj name: "<<object_name_file<<std::endl;
				}
				if(n_line==2)
				{
					Tensor::Tokenize(line,line_elem,":");
					int n_clusters;
					std::istringstream(line_elem.at(1))>>n_clusters;
					std::cout<<"Clusters: "<<n_clusters<<std::endl;
					std::vector<std::string>().swap(line_elem);
					for(int j=0;j<n_clusters;j++)
					{
						std::getline (f,line);
						Tensor::Tokenize(line,line_elem,",");
						float data[3];
						for(int i=0;i<line_elem.size();i++)
						{
							std::istringstream(line_elem.at(i))>>data[i];
						}
						//            pcl::PointXYZ point(data[0],data[1],data[2]);
						//            cluster_centres.push_back(point);
						//            std::cout<<point<<std::endl;
						line_elem.clear();
					}
				}
			}
			if(n_line==3)
			{
				Tensor::Tokenize(line,line_elem,":");
				//        std::istringstream(line_elem.at(1))>>distance_th;
				//        std::cout<<"Distance threshold: "<<distance_th<<std::endl;
			}
			if(n_line==4)
			{
				Tensor::Tokenize(line,line_elem,":");
				//        reference=line_elem.at(1);
				//        std::cout<<"Reference: "<<reference<<std::endl;
			}
			if(n_line==5)
			{
				Tensor::Tokenize(line,line_elem,",");
				//        float data[3];
				//        for(int i=0;i<line_elem.size();i++)
				//        {
				//          std::istringstream(line_elem.at(i))>>data[i];
				//        }
				//        refPoint.x=data[0];
				//        refPoint.y=data[1];
				//        refPoint.z=data[2];
				//        std::cout<<refPoint<<std::endl;
			}
			if(n_line==6)
			{
				//skip
				std::getline (f,line);
				Tensor::Tokenize(line,line_elem,":");
				//        std::istringstream(line_elem.at(0))>>scene_id;
				//        std::cout<<"Scene id: "<<scene_id<<std::endl;
			}
			if(n_line==7)
			{
				std::getline (f,line);
				Tensor::Tokenize(line,line_elem,":");
				//        std::istringstream(line_elem.at(0))>>ibs_id;
				//        std::cout<<"Ibs id: "<<ibs_id<<std::endl;
			}
			if(n_line==8)
			{
				std::getline (f,line);
				Tensor::Tokenize(line,line_elem,":");
				//        std::istringstream(line_elem.at(0))>>obj_id;
				//        std::cout<<"Obj id: "<<obj_id<<std::endl;
				//        std::vector<std::string> coord;
				//        Tokenize(line_elem.at(1),coord,",");
				//        std::istringstream(coord.at(0))>>vector.x;
				//        std::istringstream(coord.at(1))>>vector.y;
				//        std::istringstream(coord.at(2))>>vector.z;
			}
			if(n_line==9)
			{
				std::getline (f,line);
				Tensor::Tokenize(line,line_elem,",");
				//        float data[3];
				//        for(int i=0;i<line_elem.size();i++)
				//        {
				//          std::istringstream(line_elem.at(i))>>data[i];
				//        }
				//        translation.x=data[0];
				//        translation.y=data[1];
				//        translation.z=data[2];
				//        std::cout<<"Object transformation"<<std::endl;
				//        std::cout<<translation<<std::endl;
			}
			if(n_line==10)
			{
				std::istringstream(line)>>rotation;
				std::cout<<rotation<<std::endl;
			}
			n_line++;
		}
		f.close();
	}
	else
	{
		std::cout<<file_txt<<" file not found"<<std::endl;
		rotation=0;
	}
	return rotation;
}

// Aux function to execute linux commands from c++
std::string Tensor::exec(const char* cmd) {
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

// Copy EigenMatrix data to Pointcloud structure
PointCloud::Ptr Tensor::copyAsPointCloud(Eigen::MatrixXf *matrixData){
	PointCloud::Ptr out(new PointCloud);
	for(int i=0;i<matrixData->rows();i++)
		out->push_back(pcl::PointXYZ(matrixData->coeff(i,0),matrixData->coeff(i,1),matrixData->coeff(i,2)));
	return out;
}

// Extract pointcloud/voxel using indices where these can be repeated
// PCL extraction does not allow that.
// Furthermore, is more useful for us to have a point (NN) for every keypoint
void Tensor::extractRepeatedIds(std::vector<int> indices, PointCloud::Ptr inCloud, PointCloud::Ptr outCloud){
	for(int i=0;i<indices.size();i++)
	{
		outCloud->push_back(inCloud->at(indices.at(i)));
	}
}