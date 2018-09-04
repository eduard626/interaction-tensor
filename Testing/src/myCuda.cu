  #include <thrust/reduce.h>
  #include <thrust/host_vector.h>
  #include <thrust/device_vector.h>

__constant__ double PI = 3.141592653589;

  __global__ void bayesianKernel(float *nn,float *kp, float *pv, float *tp, float *C, int start, int end, int comp, float *weights, float *mags, float a_th, int *ppCentroid, int* startppCentroid, float *ppCentroidData){

    //I think I only need row
    int inner_ele = blockIdx.y*blockDim.y+threadIdx.y;    //This goes 0-2048
    int actual_ele=inner_ele + start; //This goes 0-2048 for now, could get larger
    //Get the actual_ele neighbour and compute vectors and stuff
    float xt= nn[actual_ele*comp+0]-(tp[0]+kp[actual_ele*3+0]);
    float yt= nn[actual_ele*comp+1]-(tp[1]+kp[actual_ele*3+1]);
    float zt= nn[actual_ele*comp+2]-(tp[2]+kp[actual_ele*3+2]);
    for (int i=0;i<ppCentroid[actual_ele];i++)
    {
      int idx=startppCentroid[actual_ele]+i;  //0-2969 for now, could get larger
      //int aff_id=ppCentroidData[idx*comp+0];
      int or_id=ppCentroidData[idx*comp+1];
      int pv_id=ppCentroidData[idx*comp+2];
      //int kp_id=ppCentroidData[idx*comp+2];
      //int large_idx=aff_id*512+kp_id;  //0-2048 => [0-3]*[0-511]
      //prev_idx is the id of kp in original set [0-16364]
      //int prev_idx=4096*aff_id+(kp_id+or_id*512); //[0-16384] "ordered"

      //This is some old code
      // when Y was pointing upwards

//      float angle=or_id*2*PI/8;
//      float xpv=pv[idx*3+0]*cos(angle)+pv[idx*3+2]*sin(angle);
//      float ypv=pv[idx*3+1];
//      float zpv=-pv[idx*3+0]*sin(angle)+pv[idx*3+2]*cos(angle);


      float angle=or_id*2*PI/8;
      float xpv=pv[idx*3+0]*cos(angle)-pv[idx*3+1]*sin(angle);
      float ypv=sin(angle)*pv[idx*3+0]+cos(angle)*pv[idx*3+1];
      float zpv=pv[idx*3+2];


      float diff=sqrt(((xt-xpv)*(xt-xpv))+((yt-ypv)*(yt-ypv))+((zt-zpv)*(zt-zpv)))/mags[idx];  //This is the difference as proportion of expected magnitude

      //Likelihood is the sample from a normal distribution with mean 0 and std=0.1/weighs;
      float sigma=a_th*(1+weights[idx]);
      //float sigma=weights[idx];
      float likelyhood=expf(-(diff*diff)/(2*sigma*sigma));
      //float likelyhood=(1/sigma*(sqrt(2*PI)))*expf(-(diff*diff)/(2*sigma*sigma));
      //float prior=1/ppCentroid[actual_ele];
      C[idx]=likelyhood*weights[idx];

//      if(or_id==0)
//      {
//    	  printf("Vid:%d Idx:%d LK:%f C:%f\n",pv_id,idx, likelyhood,weights[idx]*likelyhood);
//    	  //printf("ID: %d ToGo: %d\n",idx,ppCentroid[actual_ele]);
//      }
  /*    if(diff<=mags[idx])
      {
        float num=xpv*xt+ypv*yt+zpv*zt;
        float den=xt*xt+yt*yt+zt*zt;
        if((num*num/den)<=(mags[idx]*cos(a_th[aff_id]))^2)
          C[idx]= 1;//weights[idx];
        else
          C[idx]=0;
      }*/
    }
  //  if(inner_ele<5)
  //  {
  //    printf("\n");
  //  }
  }

  void bayesian_scores(float *nn,float *kp, float *pv, float *tp, float *C, int start, int end, int comp, float *weights, float *mags, float a_th, int *ppCentroid, int* startppCentroid, float *ppCentroidData){
    int maxThreads=128;      //From tables
    int N=end-start;
    dim3 threadsPerBlock(1, maxThreads);  //1x128
    dim3 blocksPerGrid(1, N/maxThreads);  //1x(4096/128) => 1x32
    bayesianKernel<<<blocksPerGrid,threadsPerBlock>>>(nn, kp, pv, tp, C, start, end, comp, weights, mags, a_th, ppCentroid, startppCentroid, ppCentroidData);
    cudaDeviceSynchronize();
  }
