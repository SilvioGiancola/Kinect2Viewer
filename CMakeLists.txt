cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (Kinect2Viewer)  # name of the project


find_package (Qt5 REQUIRED COMPONENTS Widgets Core) # component of QT5
#find_package (VTK REQUIRED) # find VTK
find_package (PCL REQUIRED) # find PCL

 # find freenect2
find_package (freenect2 REQUIRED)



include_directories (../Kinect2Viewer) # include the project in se
include_directories (../Kinect2Viewer-build) # include the project in se

include_directories (${freenect2_INCLUDE_DIR})
#link_directories    (${freenect2_LIBRARY})
#add_definitions     (${PCL_DEFINITIONS})


include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp mykinect.cpp)
set  (project_HEADERS mykinect.h)
set  (project_FORMS   )
#set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

#INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})


ADD_EXECUTABLE  (Kinect2Viewer 	${project_HEADERS}
				${project_SOURCES}
				${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})


TARGET_LINK_LIBRARIES (Kinect2Viewer 
#			${QT_LIBRARIES} 
#			${VTK_LIBRARIES} 
			${PCL_LIBRARIES}
 			${freenect2_LIBRARY})

#qt5_use_modules (Kinect2Viewer Widgets)


