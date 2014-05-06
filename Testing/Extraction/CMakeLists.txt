#-----------------------------------------------------------------------------
# Set additional include directories in the project configuration.
# Directories in binary tree are needed for generated ui and resource headers.
#
INCLUDE_DIRECTORIES(
  ${ITK_INCLUDE_DIR}
  ${ITK_INCLUDE_DIRS}
)

#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestFixedScaleHessianBasedVesselSectionEstimator
  ivanFixedScaleHessianBasedVesselSectionEstimatorTest.cxx
)

TARGET_LINK_LIBRARIES( TestFixedScaleHessianBasedVesselSectionEstimator
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestFixedScaleHessianBasedVesselSectionEstimator ${EXECUTABLE_OUTPUT_PATH}/TestFixedScaleHessianBasedVesselSectionEstimator
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussianTube_1_0.mhd FixedScaleHessianSectionEstimator.txt 2.0 5 5 20 0 0 20 )


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestFixedScaleOOFBasedVesselSectionEstimator
  ivanFixedScaleOOFBasedVesselSectionEstimatorTest.cxx
)

TARGET_LINK_LIBRARIES( TestFixedScaleOOFBasedVesselSectionEstimator
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestFixedScaleOOFBasedVesselSectionEstimator ${EXECUTABLE_OUTPUT_PATH}/TestFixedScaleOOFBasedVesselSectionEstimator
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussianTube_1_0.mhd FixedScaleOOFSectionEstimator.txt 2.0 5 5 20 0 0 20 )


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscaleAdaptiveHessianBasedVesselSectionEstimator
  ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimatorTest.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscaleAdaptiveHessianBasedVesselSectionEstimator
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestMultiscaleAdaptiveHessianBasedVesselSectionEstimator ${EXECUTABLE_OUTPUT_PATH}/TestMultiscaleAdaptiveHessianBasedVesselSectionEstimator
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussianTube_3_37.mhd MultiscaleHessianSectionEstimator.txt 5 1.0 5.0 0 5 5 20 0 0 20 )
  
  
#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscaleAdaptiveHessianBasedVesselSectionEstimator2
  ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimatorTest2.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscaleAdaptiveHessianBasedVesselSectionEstimator2
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestMultiscaleAdaptiveHessianBasedVesselSectionEstimator2 ${EXECUTABLE_OUTPUT_PATH}/TestMultiscaleAdaptiveHessianBasedVesselSectionEstimator2
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussianTube_3_37.mhd MultiscaleHessianSectionEstimator.txt 5 1.0 5.0 0 5 5 20 0 0 20 )
  
  
#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscaleAdaptiveOOFBasedVesselSectionEstimator
  ivanMultiscaleAdaptiveOOFBasedVesselSectionEstimatorTest.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscaleAdaptiveOOFBasedVesselSectionEstimator
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestMultiscaleAdaptiveHessianBasedVesselSectionEstimator ${EXECUTABLE_OUTPUT_PATH}/TestMultiscaleAdaptiveOOFBasedVesselSectionEstimator
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussianTube_3_37.mhd MultiscaleOOFSectionEstimator.txt  5 1.0 5.0 0 5 5 20 0 0 20 )


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscaleAdaptiveOOFBasedVesselSectionEstimator2
  ivanMultiscaleAdaptiveOOFBasedVesselSectionEstimatorTest2.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscaleAdaptiveOOFBasedVesselSectionEstimator2
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestMultiscaleAdaptiveHessianBasedVesselSectionEstimator2 ${EXECUTABLE_OUTPUT_PATH}/TestMultiscaleAdaptiveOOFBasedVesselSectionEstimator2
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussianTube_3_37.mhd MultiscaleOOFSectionEstimator.txt 5 1.0 5.0 0 5 5 20 0 0 20 )
  

#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestOptimizedVesselnessBasedSearchVesselTrackerFilter
  ivanOptimizedVesselnessBasedSearchVesselTrackerFilterTest.cxx
)

TARGET_LINK_LIBRARIES( TestOptimizedVesselnessBasedSearchVesselTrackerFilter
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

# Needs review
#ADD_TEST( TestOptimizedVesselnessBasedSearchVesselTrackerFilter ${EXECUTABLE_OUTPUT_PATH}/TestOptimizedVesselnessBasedSearchVesselTrackerFilter 
 # ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussianTube_3_37.mhd 5 5 20 )


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestOffsetMedialnessVesselSectionFitCostFunction 
  ivanOffsetMedialnessVesselSectionFitCostFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestOffsetMedialnessVesselSectionFitCostFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestVesselnessRidgeSearchVesselTrackerFilter 
  ivanVesselnessRidgeSearchVesselTrackerFilterTest.cxx
)

TARGET_LINK_LIBRARIES( TestVesselnessRidgeSearchVesselTrackerFilter
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscaleVesselnessRidgeSearchVesselTrackerFilter 
  ivanMultiscaleVesselnessRidgeSearchVesselTrackerFilterTest.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscaleVesselnessRidgeSearchVesselTrackerFilter
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)
  
  
#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscaleVesselnessRidgeSearchVesselTrackerFilter2
  ivanMultiscaleVesselnessRidgeSearchVesselTrackerFilterTest2.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscaleVesselnessRidgeSearchVesselTrackerFilter2
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)