cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (newGUI)
find_package (Qt5 REQUIRED COMPONENTS Widgets Core)
find_package (VTK 7.0 REQUIRED)
find_package (PCL 1.8 REQUIRED)
find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp mainwindow.cpp myTensor.cpp popup.cpp savedialog.cpp)
set  (project_HEADERS mainwindow.h myTensor.h popup.h savedialog.h)
set  (project_FORMS   mainwindow.ui popup.ui savedialog.ui)
#set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)
CMAKE_HOST_SYSTEM_INFORMATION(RESULT HOSTNAME QUERY HOSTNAME)
if ("${HOSTNAME}" STREQUAL "it057384")
	MESSAGE (STATUS "Hostname:${HOSTNAME}")
	set  (QHULL_LIBRARIES /home/er13827/Downloads/qhull/install/lib/libqhullcpp.a /home/er13827/Downloads/qhull/install/lib/libqhullstatic_r.a)
else()
	set  (QHULL_LIBRARIES /usr/local/lib/libqhullcpp.a /usr/local/lib/libqhullstatic_r.a)
endif()

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

#INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (newGUI ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (newGUI ${QHULL_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
set_target_properties(newGUI PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
qt5_use_modules (newGUI Widgets)
