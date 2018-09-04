//#define TEXT_ONLY 1

#include "Tensor.h"
#include "myCuda.cuh"
#include "dev_array.h"

// For GPU search/containers
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/containers/initialization.h>


#define O 8	//orientations
#define n 512	//keypoints
#define N O*n	//orientations x keypoints
#define C 3	//components
#define SIZE  N*C   //This is the size of the NNs. 1 for each keypoint in 8 orientations

bool fexists(const std::string& filename) {
	std::ifstream ifile(filename.c_str());
	return (bool)ifile;
}

float prob_th;	//Prediction threshold read from parameters file
PointCloud::Ptr inputCloud;


//Aux function to colour text
void generateColours(std::vector<pcl::RGB>& v);
// Get the best prediction per test-point. Used when "live mode" or viewer is enabled. Only best-to-date prediction is shown while running
int getMinMax(std::vector<float>& bigScores,std::vector <float> &score, std::vector<int> & orientation, std::vector<int> & aff_id, Eigen::MatrixXf thresholds, PointCloud::Ptr data);
// Get all the good predictions per test-point. Used when text-only mode is enabled.
int getMinMaxMulti(std::vector<float>& bigScores,std::vector <float> &score, std::vector<int> & orientation, std::vector<int> & aff_id, Eigen::MatrixXf thresholds, PointCloud::Ptr data,PointCloudC::Ptr cloudToSave);
// Estimate test vectors and compute score using cuda
std::vector<float> computeScore(PointCloud::Ptr local_nn,float *samplePoint, dev_array<float> & kpData,dev_array<float> & pvData,dev_array<float> & wData,dev_array<float> & mData, dev_array<int> & ppC, dev_array<int> & startppC,dev_array<float> & ppCData,Tensor*object);

int main(int argc, char * argv[])
{
	std::cout<<"Creating new Tensor object...";
	Tensor myTensor(argc,argv);
	cout<<"done "<<std::endl;
	std::cout<<"Initialising..."<<std::endl;
	myTensor.init();
	std::cout<<"done."<<std::endl;
	prob_th=myTensor.pred_t;
	std::cout<<"Prediction thershold "<<prob_th<<std::endl;
	if(myTensor.plotDebug>1)
		myTensor.printObjInfo();	

	// We save data in pointcloud structure
	PointCloudC::Ptr goodData(new PointCloudC);
	PointCloudC::Ptr goodPoints(new PointCloudC);
	PointCloud::Ptr sampledPoints(new PointCloud);

	inputCloud.reset(new PointCloud);

	inputCloud=myTensor.sceneCloud;


	// Prepare the randomSampling input
	myTensor.randomSample.setInputCloud(inputCloud);
	// Do the sampling
	myTensor.sampleCloud();
	// To keep track of the progress
	int topCount=myTensor.randomSampleIdx.size();
	int progress=0;
	std::string scene_name;

	// Print twice just to double check that they are the same
	std::cout<<"Point to sample: "<<topCount<<" "<<myTensor.randomSampleCloud->size()<<std::endl;

	//If ploting stuff
	PointCloud::Ptr object(new PointCloud);
	pcl::PolygonMesh::Ptr best_object(new pcl::PolygonMesh);
	
	// To keep track of good locations, angles and scores
	std::vector<float> best_local_anle(myTensor.affordance_name.size()),best_angle(myTensor.affordance_name.size());
	std::vector<float> best_local_score(myTensor.affordance_name.size(),0),best_score(myTensor.affordance_name.size(),0);
	bool goodPoint=false;

	//Print to info regarding files/descriptor loaded from disk
	std::cout<<"Large vectors: "<<myTensor.largeVectors.rows()<<"x"<<myTensor.largeVectors.cols()<<" mags: "<<myTensor.large_mags.size()<<" lenghts: "<<myTensor.large_lengths.size()<<std::endl;
	std::vector<pcl::RGB> colours(myTensor.affordance_name.size());
  	generateColours(colours);
	std::vector<int> success_counter(myTensor.affordance_name.size(),0);


if(myTensor.plotDebug>0)
{
	// If debugging, show input scene in pointcloud mode to viewer
	if(myTensor.plotDebug>1)
	{
		
		myTensor.plot(inputCloud,"windowC",255,255,255,1);
		myTensor.viewer->setPosition(0,0);
		myTensor.viewer->setShowFPS(false);
		myTensor.viewer->addText("PPS 0",50,50,"Rate");
	}
	else
	{
		// If not debugging first instantiate the viewer and set some basic settings
		myTensor.viewer.reset(new pcl::visualization::PCLVisualizer);
		myTensor.viewer->setSize(1280,720);
		myTensor.viewer->addCoordinateSystem(.1,"Coord",0);
		myTensor.viewer->setPosition(0,0);
		myTensor.viewer->setShowFPS(false);
		myTensor.viewer->addText("PPS 0",50,50,"Rate");

		// Try to find a CAD (PLY/OBJ) model of the input scene, just because it looks nicer
		// Otherwise, show the scene pointcloud
		// Code searches for the same file name as pointcloud but with
		// prefix "display"
		scene_name=myTensor.file_name+myTensor.scn_name+".ply";
		std::cout<<"Looking for "<<scene_name<<std::endl;
		if(fexists(scene_name))
		{
			std::cout<<"Nice display file found: "<<scene_name<<std::endl;
#if VTK_MAJOR_VERSION < 6
			myTensor.viewer->addModelFromPLYFile(scene_name.c_str(),"scene",0);
#else
			pcl::PolygonMesh mesh;
			pcl::io::loadPolygonFilePLY(scene_name.c_str(),mesh);
			if(argc==2)
				myTensor.viewer->addPolygonMesh(mesh,scene_name.c_str(),0);
			else
				myTensor.viewer->addPolygonMesh(mesh,scene_name.c_str(),0);
#endif
		}
		else
		{
			std::string scene_name=myTensor.file_name+myTensor.scn_name+".obj";
			if(fexists(scene_name))
			{
				pcl::PolygonMesh mesh;
				pcl::io::loadOBJFile(scene_name.c_str(),mesh);
				std::cout<<"Nice display file found: "<<scene_name<<std::endl;
				myTensor.viewer->addPolygonMesh(mesh,scene_name.c_str(),0);
			}
			else
			{	
				// Load input pointcloud into viewer if CAD model was not found
				std::cout<<"Nice display file not found. Loading point cloud"<<std::endl;
				myTensor.plot(inputCloud,"windowC",255,255,255,1);

			}
		}
	}
	for(int k=0;k<colours.size();k++)
    {
      std::stringstream count;
      count << success_counter.at(k);
      std::string window_text=myTensor.affordance_name.at(k)+" "+count.str();
      myTensor.viewer->addText(window_text.c_str(),100,200-k*20,20.0,double(colours.at(k).r)/255,double(colours.at(k).g)/255,double(colours.at(k).b)/255,myTensor.affordance_name.at(k).c_str(),0);
      count.str(std::string());
    }
		// Show and wait for input
		while(!myTensor.viewer->wasStopped())
		{
			myTensor.viewer->spinOnce();
		}
		myTensor.viewer->resetStoppedFlag();
}

	// clock to compute "frame rate" or points tested per second
	std::clock_t ppsecond;

	// Set GPU/cuda stuff 

	// If more than 1 GPU  you can set it here
	dev_array<float>::setGPU(0);
	// Keypoints: 2048 points x 3 components
	dev_array<float> d_kpData(myTensor.largeDescriptor.rows()*3);
	d_kpData.set(myTensor.largeDescriptor.data(),myTensor.largeDescriptor.rows()*3);
	// Provenance Vectors: 512 vectors x 3 components
	dev_array<float> d_pvData(myTensor.largeVectors.rows()*3);
	d_pvData.set(myTensor.largeVectors.data(),myTensor.largeVectors.rows()*3);
	// Provenance Vectors: 512 vector lengths
	dev_array<float> d_weights(myTensor.large_lengths.size());
	d_weights.set(&myTensor.large_lengths[0],myTensor.large_lengths.size());
	// Provenance Vectors: 512 vector weights
	dev_array<float> d_mags(myTensor.large_mags.size());
	d_mags.set(&myTensor.large_mags[0],myTensor.large_mags.size());
	// Usefull for multiple affordance, for single affordance this
	// is a 2048 array full on 1's
	dev_array<int> d_ppCentroid(myTensor.largeDescriptor.rows());
	d_ppCentroid.set(&myTensor.ppCentroid[0],myTensor.largeDescriptor.rows());
	// Usefull for multiple affordance, for single affordance this
	// is a 2048 with the cumsum of ppCentroid 
	dev_array<int> d_startppCentroid(myTensor.largeDescriptor.rows());
	d_startppCentroid.set(&myTensor.startppCentroid[0],myTensor.largeDescriptor.rows());
	//Times 4 because pcl alignment x,y,z + extra byte
	// This has data for orientation, keypoint and affordance id
	dev_array<float> d_ppCData(myTensor.largeData->size()*4);
	d_ppCData.set(&myTensor.largeData->at(0).getArray3fMap()[0],myTensor.largeData->size()*4);

	// GPU stuff in PCL
	// Container for indices in NN-search. Same size as decscriptor since 1-NN for every
	// keypoint in descriptor
	pcl::gpu::NeighborIndices result_device(myTensor.largeDescriptor.rows(), 1);
	// Octree for NN search: cloud and structure
	pcl::gpu::Octree::PointCloud cloud_device2;
	pcl::gpu::Octree octree_device2;
	// Container for descriptor keypoints (point queries)
	pcl::gpu::Octree::Queries queries_device;

	// For "frame rate"
	pcl::console::TicToc tt;
	int points_tested=0;
	ppsecond = std::clock();
	int max_cloud=0;
	// Test point counter
	int i=0;
	while(i < topCount)
	{
		// the current test point sampled from scene
		pcl::PointXYZ sampledPoint;
		sampledPoint=inputCloud->at(myTensor.randomSampleIdx.at(i));
		// Save all testpoints sampled
		sampledPoints->push_back(sampledPoint);
		// Some functions expec data as vector or array, so make a copy
		// of the sampled point in different containers. Surely there's a better way to do this
		float testPoint[3]={sampledPoint.x,sampledPoint.y,sampledPoint.z};
		Eigen::Vector3f asp(sampledPoint.x,sampledPoint.y,sampledPoint.z);

		//Just some info about total progress
		int current_percent=int(100*i/topCount);
		if(current_percent-progress>0)
		{
			progress=current_percent;
			if(myTensor.plotDebug>1)
				cout<<"\n+++++++++   "<<current_percent<<"   ++++++++++"<<std::endl;
			else
				std::cout<<" "<<current_percent;
				//std::cout<<std::endl;
			std::cout<<std::flush;
		}
		else
		{
				//std::cout<<" ";
			std::cout<<std::flush;
		}
		
		// Only best_point will be shown at run time
		std::vector<pcl::PointXYZ> best_point(myTensor.affordance_name.size());
		// Should a "best point" be found then need to update viewer
		std::vector<bool> update(myTensor.affordance_name.size(),false);

		// If showing viewer, plot a line at the current test-point
		// just as visual aid
if(myTensor.plotDebug>0)
{
		if(myTensor.viewer->contains("test"))
			myTensor.viewer->removeShape("test");
		myTensor.viewer->addLine(sampledPoint,pcl::PointXYZ(sampledPoint.x,sampledPoint.y,sampledPoint.z+(myTensor.desiredSize*.5)),1,0,0,"test");
		myTensor.viewer->spinOnce();
}
		// In some pointclouds there were points with coordinates 0,0,0
		// ignore those
		// I think this is a bug during the data capturing in real scenes
		if(sampledPoint.x==0 && sampledPoint.y==0 && sampledPoint.z==0) 
			continue;
		// This is a copy of the descriptor that will be
		// moving around tom compute scores
		Eigen::MatrixXf moving_descriptor(myTensor.largeDescriptor.rows(),3);
		// Move or transform the descriptor relative to the current test-point
		moving_descriptor=myTensor.largeDescriptor.rowwise()+asp.transpose();
		if(myTensor.plotDebug>1)
			std::cout<<"Descriptor size "<<moving_descriptor.rows()<<std::endl;
		// Copy the new descriptor data as pointcloud.
		// I think PCL has a way to map the memory from a Eigen Matrix to a pointcloud structure
		// but haven't found it. So do it manually
		PointCloud::Ptr moving_spinCloud=Tensor::copyAsPointCloud(&moving_descriptor);

		/*
		Will do a NN-search to extract a voxel from the scene pointcloud
		around the current test-point
		*/

		// Containers for NN search: point indices and distances
		std::vector<int> pointIdxVec;
		std::vector<float> pointRadiusSquaredDistance;
		// If debugging, count time to extract voxel
		if(myTensor.plotDebug>1)
			tt.tic();
		bool voxelOutcome=false;

		// Clear the pointcloud that will store the voxel extracted from scene
		myTensor.auxCloud->clear();

		/* PointcloudType;
		0 -> Synthetic PLY scene with access to polygons and vertices data
			Use the middle point in polygons to extract voxel

		1 -> PCD file with only vertices or point data
		*/
		// This search is done in GPU. No important difference was notice when using GPU-version
		if(myTensor.pointCloudType==0)
		{
			// Notice the search rad is half size of "desired size" which is diagonal of query-object bouding box
			voxelOutcome=myTensor.octreeCentres->radiusSearch(sampledPoint,0.5*myTensor.desiredSize,pointIdxVec,pointRadiusSquaredDistance);
			// Voxel should have at leats some points, this could change
			// acording to descriptor size
			if(pointIdxVec.size()<2)
			{
				if(myTensor.plotDebug>1)
					std::cout<<"Not enough points in scene voxel "<<pointIdxVec.size()<<std::endl;
				// Free gpu memory and continue sampling test-points
				cloud_device2.release();
				result_device.data.release();
				continue;
			}
			// If enough points in voxel: extract and store in auxCloud
			Tensor::extractCloud(pointIdxVec,myTensor.cellCentres,myTensor.auxCloud);
		}
		else
		{
			// Same for PCD scene
			voxelOutcome=myTensor.octree->radiusSearch(sampledPoint,0.5*myTensor.desiredSize,pointIdxVec,pointRadiusSquaredDistance);
			if(pointIdxVec.size()<2)
			{
				cloud_device2.release();
				result_device.data.release();
				continue;
			}
			Tensor::extractCloud(pointIdxVec,inputCloud,myTensor.auxCloud);
		}

		if(myTensor.plotDebug>1)
			std::cout<<i<<" "<<myTensor.auxCloud->size()<<" cloud extraction ["<<tt.toc()<<" ms]"<<std::endl;

		// With the current voxel perform 1-NN search for affordance keypoints
		// these will be used to estimate test-vectors
		// Upload voxel to GPU and build octree
		cloud_device2.upload(myTensor.auxCloud->points);
		octree_device2.setCloud(cloud_device2);
		if(myTensor.plotDebug>1)
			std::cout<<"Building tree in GPU"<<std::endl;
		octree_device2.build();
		if(myTensor.plotDebug>1)
			std::cout<<"...done"<<std::endl;
		
		// Upload descriptor (queries) to GPU
		queries_device.upload(moving_spinCloud->points);

		// If debugging, take the time it takes to do NN search and get data
		if(myTensor.plotDebug>1) 
			tt.tic();
		// Perform 1-NN search for affordance keypoints in descriptor
		octree_device2.nearestKSearchBatch(queries_device, 1, result_device);
		// Download data to PC memory, point indices are returned
		std::vector<int> downloaded;
		result_device.data.download(downloaded);
		
		if(myTensor.plotDebug>1)
			std::cout<<"NN search ["<<tt.toc()<<" ms]"<<std::endl;

		// Copy NN data to a pointcloud structure, there could be
		// repeated indices
		PointCloud::Ptr local_nn(new PointCloud);;
		Tensor::extractRepeatedIds(downloaded,myTensor.auxCloud,local_nn);
		
		// Estimate test vectors and compute scores
		// in gpu with CUDA. If debugging, take the time
		std::clock_t tscore;
		if(myTensor.plotDebug>1)
		{
			tscore = std::clock();
		}
		std::vector<float> score=computeScore(local_nn,testPoint,d_kpData,d_pvData,d_weights,d_mags,d_ppCentroid,d_startppCentroid,d_ppCData,&myTensor);
		if(myTensor.plotDebug>1)
		{
			std::clock_t tscoref = std::clock();
			std::cout<<"GPU vector comparison ["<<1000*( tscoref - tscore ) / (double) CLOCKS_PER_SEC<<" ms]"<<std::endl;
		}

		std::vector<float> max;
		std::vector<int> ids,orientations;
		// If showing viewer
if(myTensor.plotDebug>0)
{
		// Get the location/orientation that scores higher than prediction Thershold
		// This only gets the top orientation/score to show in viewer
		int idMax=getMinMax(score,max,orientations,ids,myTensor.alternative_data_counts,myTensor.largeData);

		// Increment the number of test-points actually tested
		// and compute the "points per second" stats
		points_tested+=1;
		std::clock_t ppsecond2 = std::clock();
		double timeInSeconds=1000*( ppsecond2 - ppsecond ) / (double) CLOCKS_PER_SEC;
		if(timeInSeconds>=1000)
		{
			//updateText in viewer
			std::stringstream points_teste_text;
			points_teste_text << points_tested;
			std::string point_rate="PPS "+points_teste_text.str();
			myTensor.viewer->updateText(point_rate.c_str(),50,50,"Rate");
			myTensor.viewer->spinOnce();
			points_tested=0;
			ppsecond=ppsecond2;
		}

		// If idMax is >0 then actually found something good
		// Otherwilse keep searching
		int idSuccess=idMax>=0?ids.at(idMax):-1;
		if(ids.size()>0)
		{
			// Save some data about the good location
			// coordinates
			pcl::PointXYZRGB agp(0,0,0);
			agp.x=sampledPoint.x;
			agp.y=sampledPoint.y;
			agp.z=sampledPoint.z;
			// Enconde in colors some other data
			// red is how many good matches (affordances x orientations) in this point
			agp.r=ids.size();
			goodPoints->push_back(agp);
			// For single affordance case this loop does not make sense
			// but will keep it for future release (multiple affordances)
			for(int sId=0;sId<ids.size();sId++)
			{

				int anId=ids.at(sId);
				pcl::PointXYZRGB datap(0,0,0);
				//x is affordance id;
				datap.x=anId;
				//y is orientation id;
				datap.y=orientations.at(sId);
				//z is score;
				datap.z=max.at(sId);

				// save data in pointcloud structure
				goodData->push_back(datap);

				//keep track how many good predictions
				success_counter.at(anId)+=1;

				// If current result is the best
				// save it and update viewer (flag)
				if(max.at(sId)>best_score.at(anId))
				{
					best_score.at(anId)=max.at(sId);
					best_angle.at(anId)=2*orientations.at(sId)*M_PI/O;
					best_point.at(anId)=sampledPoint;
					update.at(anId)=true;
				}
				// update text in viwewer
				std::stringstream count,be;
				count << success_counter.at(anId);
				be<<best_score.at(anId);
				std::string window_text=" "+be.str()+" "+myTensor.affordance_name.at(anId)+" "+count.str();
      			myTensor.viewer->updateText(window_text.c_str(),100,200-anId*20,myTensor.affordance_name.at(anId).c_str());

				best_local_anle.at(anId)=2*orientations.at(sId)*M_PI/O;
				best_local_score.at(anId)=max.at(sId);

				if(update.at(anId))
				{
					// Set everything for viewer

					std::string object_cloud_name="best_"+myTensor.affordance_name.at(anId)+"_"+myTensor.ob_names.at(anId);

					// Transformations needed for query-object cloud/meshes
					Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
					transform_2.translation() << 0.0, 0.0, 0.0;
					transform_2.rotate (Eigen::AngleAxisf (best_angle.at(anId), Eigen::Vector3f::UnitZ()));
					pcl::PointXYZ rotToObj=pcl::transformPoint(myTensor.toObject->at(anId),transform_2);
					pcl::PointXYZ obRef(sampledPoint.x+rotToObj.x,sampledPoint.y+rotToObj.y,sampledPoint.z+rotToObj.z);
					pcl::fromPCLPointCloud2(myTensor.object_meshes.at(anId).cloud,*object);
					Tensor::rotateCloud(object,object,best_angle.at(anId),'z',myTensor.original_points->at(anId));
					Tensor::translateCloud(object,object,obRef,myTensor.original_points->at(anId));
					// Update the query-object mesh					
					pcl::PCLPointCloud2 vertices;
					pcl::toPCLPointCloud2(*object,vertices);
					pcl::copyPointCloud(vertices,best_object->cloud);
					best_object->polygons.resize(myTensor.object_meshes.at(anId).polygons.size());
					std::copy(myTensor.object_meshes.at(anId).polygons.begin(),myTensor.object_meshes.at(anId).polygons.end(),best_object->polygons.begin());

					// Actually plot the query-objet in new location
					myTensor.plot(*best_object,object_cloud_name.c_str(),colours.at(anId).r,colours.at(anId).g,colours.at(anId).b);
					
					// If debugging plot and wait
					if(myTensor.plotDebug>1 && update.at(anId))
					{
						while(!myTensor.viewer->wasStopped())
						{
							myTensor.viewer->spinOnce();
						}
						myTensor.viewer->resetStoppedFlag();

					}
					else
					{
						// if not plot and continue
						myTensor.viewer->spinOnce();
					}
				}
				// Flip the update viewer flag
				if(update.at(anId))update.at(anId)=false;
			}
		}
}
else
{
		// If not showing viewer 
		// compute and get every orientation that is predicted as good
		int thisManyAffordances=getMinMaxMulti(score,max,orientations,ids,myTensor.alternative_data_counts,myTensor.largeData,goodData);
		// If actually found something save everything
		if(thisManyAffordances>0)
		{
			//save datapoint with info
			pcl::PointXYZRGB agp(0,0,0);
			agp.x=sampledPoint.x;
			agp.y=sampledPoint.y;
			agp.z=sampledPoint.z;
			//red is how many good matches (affordances x orientations) in this point
			// we can use 3 8-bit integers if needed
			// All other data such as scores, orientations and ids are store
			// inside the getMinMaxMulti function
			if(ids.size()>255)
			{
				agp.r=255;
				if(ids.size()>510)
				{
					agp.g=255;
					agp.b=ids.size()-510;
				}
				else
				{
					agp.g=ids.size()-255;
				}
			}
			else
				agp.r=ids.size();
			goodPoints->push_back(agp);
		}
}
		// Free GPU memory
		cloud_device2.release();
		result_device.data.release();

		// get the next test-point
		i+=1;
	}
	std::cout<<" 100"<<std::endl;
if(myTensor.plotDebug>0)
{
	//spin and exit
	myTensor.viewer->resetStoppedFlag();
	while(!myTensor.viewer->wasStopped())
		myTensor.viewer->spinOnce(100);
}
	if(goodPoints->size()>0)
	{
		myTensor.converged=true;
		cout<<"\n\n ========= Success ==========\n"<<std::endl;
		std::string fileID=myTensor.saveClouds(goodData,goodPoints);
		cout<<"Good places: "<<goodPoints->size()<<"\n";
	}
	else
	{
		cout<<"\n\n ========= Finish with no success ==========\n\n";
		myTensor.saveClouds(goodData,goodPoints);
	}


	//#endif

	return 0;
}

void generateColours(std::vector<pcl::RGB>& v){
	for(int i=0;i<v.size();i++)
	{
		pcl::RGB acolor=pcl::GlasbeyLUT::at(i);
		v.at(i)=acolor;
		//std::cout<<"R: "<<double(v.at(i).r)/255<<" G: "<<static_cast<int>(acolor.g)<<" B: "<<static_cast<int>(acolor.b)<<std::endl;
	}
}

std::vector<float> computeScore(PointCloud::Ptr local_nn,float *samplePoint, dev_array<float> & kpData,dev_array<float> & pvData,dev_array<float> & wData,dev_array<float> & mData, dev_array<int> & ppC, dev_array<int> & startppC,dev_array<float> & ppCData,Tensor *object){

	//Get the size of the result container
	int inner_N=object->largeVectors.rows();;
	// This is max for my GPU, can/should be modified
	int maxThreads=2048;
	// Allocate memory on the device
	//Result containers in CPU and GPU memory
	std::vector<float> h_C(inner_N,0);
	dev_array<float> d_C(inner_N);
	d_C.set(&h_C[0],inner_N);
	// NN at test point:  same size as descriptor or neightbours cloud
	dev_array<float> d_nnData(local_nn->size()*4);   
	// Test point: 1 point x 3 components
	dev_array<float> d_testPoint(C);  
	// NN data
	d_nnData.set(&local_nn->at(0).getArray3fMap()[0], local_nn->size()*4);
	// Current test-point
	d_testPoint.set(samplePoint,C);
	// Need to copy batches of maxThreads to GPU
	int thisManyBatches=std::ceil(double(local_nn->size())/double(maxThreads));  //Possible BUG in batch computation!
	if(object->plotDebug>1)
		std::cout<<"Sending "<<thisManyBatches<<" batches to gpu"<<std::endl;
	for (int i=0;i<thisManyBatches;i++)
	{
		int start=i*maxThreads;
		int end=start+maxThreads-1;
		if(end>local_nn->size())
			end=local_nn->size()-1;
		// Call CUDA to compute scores
		bayesian_scores(d_nnData.getData(),kpData.getData(),pvData.getData(),d_testPoint.getData(),d_C.getData(),start,end,4,wData.getData(),mData.getData(),object->agg_th,ppC.getData(),startppC.getData(),ppCData.getData());
	}
	// This a long flat vector with individual keypoint scores
	// That is read in getMinMax function
	d_C.get(&h_C[0],inner_N);
	return h_C;
}


int getMinMax(std::vector<float>& bigScores,std::vector <float> &score, std::vector<int> & orientation, std::vector<int> & aff_id, Eigen::MatrixXf thresholds, PointCloud::Ptr data)
{
	// Matrix with (point_counts)^-1 to normalize 
	Eigen::MatrixXf results=Eigen::MatrixXf::Zero(thresholds.rows(),O);
	for(int i=0;i<data->size();i++)
	{
		int row=0;  //Affordance
		if(thresholds.rows()>1)
			row=int(data->at(i).x)-1;
		int col=int(data->at(i).y); //Orientation
		results(row,col)+=bigScores.at(i);
	}
	// Normalize scores
	Eigen::MatrixXf results_scores=results.cwiseProduct(thresholds); 
	// At this point everything is in [0,1]
	// Get the max score per affodance
	// For now this is a row vector, so 1 max value
	Eigen::VectorXf perAffordance=results_scores.rowwise().maxCoeff();
	int output=-1;
	float local_max=-1;
	// Check if the max value is >= prediction thershold, save it and return it
	for(int i=0;i<perAffordance.rows();i++)
	{
		if((perAffordance[i]-prob_th)>=0)
		{
			if((perAffordance[i]-prob_th)>local_max)
				output=aff_id.size();
			score.push_back(perAffordance[i]);
			aff_id.push_back(i);
			int ori;
			float nu=results_scores.row(i).maxCoeff(&ori);
			orientation.push_back(ori);
		}
	}
	return output;
}

int getMinMaxMulti(std::vector<float>& bigScores,std::vector <float> &score, std::vector<int> & orientation, std::vector<int> & aff_id, Eigen::MatrixXf thresholds, PointCloud::Ptr data,PointCloudC::Ptr cloudToSave)
{
	// Matrix with (point_counts)^-1 to normalize 
	Eigen::MatrixXf results=Eigen::MatrixXf::Zero(thresholds.rows(),8);
	for(int i=0;i<data->size();i++)
	{
		int row=0;  //Affordance
		if(thresholds.rows()>1)
			row=int(data->at(i).x)-1;
		int col=int(data->at(i).y); //Orientation
		results(row,col)+=bigScores.at(i);
	}
	// Normalize scores
	Eigen::MatrixXf results_scores=results.cwiseProduct(thresholds);
	// At this point everything is in [0,1]
	// Check every score and save those >= prediction thershold
	int output=0;
	for(int i=0;i<results_scores.rows();i++)
	{
		int aff_counter=0;
		for(int j=0;j<results_scores.cols();j++)
		{
			if((results_scores(i,j)-prob_th)>0)  //good
			{
				score.push_back(results_scores(i,j));
				aff_id.push_back(i);
				//        if(i==3 && (j==2 || j==6))
				//          std::cout<<"HERE!!"<<std::endl;
				orientation.push_back(j);
				aff_counter+=1;
				pcl::PointXYZRGB datap(0,0,0);
				//x is affordances id;
				datap.x=i;
				//y is orientation;
				datap.y=j;
				//z is score;
				datap.z=results_scores(i,j);
				cloudToSave->push_back(datap);
			}
		}
		if(aff_counter>0)
			output+=1;
	}
	return output;
}