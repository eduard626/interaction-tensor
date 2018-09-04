#!/bin/bash

#	This script assumes that cuda has been already installed
#	and Nvidia drivers are working.
#	Update repos and get git


sudo apt-get update  
sudo apt-get install git 

mkdir ./tensorLibs
cd aPCL
git clone https://github.com/PointCloudLibrary/pcl.git pcl-trunk

#	Get all libraries/dependecies 

sudo apt-get install -y g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libflann1.8 libflann-dev
sudo apt-get install -y libeigen3-dev libboost-all-dev libusb-dev libgtest-dev git-core freeglut3-dev pkg-config
sudo apt-get install -y build-essential libxmu-dev libxi-dev libusb-1-0-dev graphviz
sudo apt-get install -y phonon-backend-gstreamer phonon-backend-vlc 

#	QT, not completly sure about this but it worked.
sudo apt-get install -y build-essential libgl1-mesa-dev
sudo apt-get install -y qtdeclarative5-dev qt5-default qttools5-dev


#	VTK from source. Tested with v 7.0.0

git clone https://gitlab.kitware.com/vtk/vtk.git
cd vtk && git checkout v7.0.0
mkdir build && cd build
cmake -DVTK_QT_VERSION:STRING=5 -DQT_QMAKE_EXECUTABLE:PATH=/usr/lib/x86_64-linux-gnu/qt5/bin/qmake -DVTK_Group_Qt:BOOL=ON -DCMAKE_PREFIX_PATH:PATH=/usr/lib/x86_64-linux-gnu/cmake/ -DBUILD_SHARED_LIBS:BOOL=ON -DVTK_RENDERING_BACKEND=OpenGL ..
make -j4 && sudo make install

#	This is optional in my case, but worth installing
cd ../../	# go back to aPCL
git clone https://github.com/occipital/OpenNI2.git
cd OpenNI2 && make
cd Packaging
python ReleaseVersion.py x64	#For 64 bits OS
echo "export OPENNI2_INCLUDE=$PWD/Packaging/OpenNI-Linux-x64-2.2/Include" >> ~/.bashrc
echo "export OPENNI2_REDIST=$PWD/Packaging/OpenNI-Linux-x64-2.2/Redist" >> ~/.bashrc
echo "export OPENNI2_LIB=$PWD/Bin/x64-Release" >> ~/.bashrc
echo "export OPENNI2_INC=$PWD/Include" >> ~/.bashrc
echo "export PATH=$PATH:$PWD/Bin/x64-Release" >> ~/.bashrc
echo "export PATH=$PATH:$PWD/bin/x64-Release/OpenNI2" >> ~/.bashrc

cd ../../

#	Get qhull
git clone https://github.com/qhull/qhull.git
cd qhull/build/
cmake ..
make && sudo make install
cd ../
make && sudo make install
echo "export LD_LIBRARY_PATH=$PWD/lib:$LD_LIBRARY_PATH" >> ~/.bashrc

## If error during qhull libraries install
# Need to fix manually the -lm flags in Makefile
##cd src/libqhull && make 
##make && sudo make install
##	Need to fix manually the -lm flags in Makefile
#	also need to fix rbox_r.c include of "libqhull/random_r.h" -> "random_r.h"
# cd ../libqhull_r/ && make && sudo make install

# go to root dir -> tensorLibs
cd ../

cd pcl-trunk
source ~/.bashrc
mkdir build && cd build
cmake -DBUILD_GPU=ON -DBUILD_CUDA=ON -DBUILD_gpu_kinfu=OFF -DBUILD_examples=OFF -DBUILD_OPENNI2=ON -DBUILD_examples=ON -DPCL_VERSION=1.8 -DWITH_VTK=ON -DWITH_QT=ON ..
make -j4 && sudo make install
