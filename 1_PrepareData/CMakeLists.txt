set(PREPARE_DATA_SRC 
	${CMAKE_CURRENT_SOURCE_DIR}/CuttingFaceMaker/CuttingFaceMaker.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CuttingFaceMaker/CrossSectionMaker/CrossSectionMaker.cpp
	PARENT_SCOPE)

set(PREPARE_DATA_LIB
	${CGAL_LINK_LIBRARIES}
PARENT_SCOPE)