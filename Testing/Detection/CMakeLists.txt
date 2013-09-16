#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularGaussianSectionSecondDerivativeScaleInvariance
  ivanCircularGaussianSectionSecondDerivativeScaleInvarianceTest.cxx
)

TARGET_LINK_LIBRARIES( TestCircularGaussianSectionSecondDerivativeScaleInvariance
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestFrangiVesselnessImageFunction
  ivanFrangiVesselnessImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestFrangiVesselnessImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestFrangiVesselnessImageFunction ${EXECUTABLE_OUTPUT_PATH}/TestFrangiVesselnessImageFunction
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd 1 FrangiVesselness.png 0.5 2.0 2.0 1 1
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestSatoVesselnessImageFunction
  ivanSatoVesselnessImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestSatoVesselnessImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestSatoVesselnessImageFunction ${EXECUTABLE_OUTPUT_PATH}/TestSatoVesselnessImageFunction 
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd 1 SatoVesselness.png 2.0 1.0 1.0 0.25 1 1
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestOffsetMedialnessImageFunction
  ivanOffsetMedialnessImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestOffsetMedialnessImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestOffsetMedialnessImageFunction ${EXECUTABLE_OUTPUT_PATH}/TestOffsetMedialnessImageFunction
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd 1 OffsetMedialness.png 2.0 3.5 0 0.5 1
)

  
#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestOffsetMedialnessImageFunction2
  ivanOffsetMedialnessImageFunctionTest2.cxx
)

TARGET_LINK_LIBRARIES( TestOffsetMedialnessImageFunction2
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

# No proper test yet
#ADD_TEST( TestOffsetMedialnessImageFunction2 ${EXECUTABLE_OUTPUT_PATH}/TestOffsetMedialnessImageFunction2
  #${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd )
  
  
#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestFilterByEigenValuesVesselnessImageFunction
  ivanFilterByEigenValuesVesselnessImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestFilterByEigenValuesVesselnessImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestFilterByEigenValuesVesselnessImageFunction ${EXECUTABLE_OUTPUT_PATH}/TestFilterByEigenValuesVesselnessImageFunction
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd 1 FilterByEigenvaluesVesselness.png 2.0 255.0 
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestPolarProfileVesselnessImageFunction
  ivanPolarProfileVesselnessImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestPolarProfileVesselnessImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

# No proper test yet
#ADD_TEST( TestPolarProfileVesselnessImageFunction
#  ${EXECUTABLE_OUTPUT_PATH}/TestPolarProfileVesselnessImageFunction
#  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd )
  

#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscaleOffsetMedialnessImageFunction
  ivanMultiscaleOffsetMedialnessImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscaleOffsetMedialnessImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestMultiscaleOffsetMedialnessImageFunction ${EXECUTABLE_OUTPUT_PATH}/TestMultiscaleOffsetMedialnessImageFunction
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd 1 MultiscaleOffsetMedialness.png 5 1.0 5.0 1 1 0.5 0.5 1
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscaleOOFVesselnessImageFunction
  ivanMultiscaleOOFVesselnessImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscaleOOFVesselnessImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestMultiscaleOOFVesselnessImageFunction ${EXECUTABLE_OUTPUT_PATH}/TestMultiscaleOOFVesselnessImageFunction
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd 1 MultiscaleOOFVesselness.png 5 1.0 5.0 1
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscalePolarProfileVesselnessImageFunction
  ivanMultiscalePolarProfileVesselnessImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscalePolarProfileVesselnessImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

# No proper test yet
#ADD_TEST( TestMultiscalePolarProfileVesselnessImageFunction
#  ${EXECUTABLE_OUTPUT_PATH}/TestMultiscalePolarProfileVesselnessImageFunction
#  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd )
  
  
#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestNonLinearSteerableFluxImageFunction
  ivanNonLinearSteerableFluxImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestNonLinearSteerableFluxImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestNonLinearSteerableFluxImageFunction ${EXECUTABLE_OUTPUT_PATH}/TestNonLinearSteerableFluxImageFunction
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd 1 NonLinearSteerableFluxVesselness.png 2.0 1
)
  
  
#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularSectionFluxImageFunction
  ivanCircularSectionFluxImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestCircularSectionFluxImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestCircularSectionFluxImageFunction ${EXECUTABLE_OUTPUT_PATH}/TestCircularSectionFluxImageFunction
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd 1 CircularSectionFluxVesselness.png 2.0 1
)
  

#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestHessianEigenValuesImageFunction
  ivanHessianEigenValuesImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestHessianEigenValuesImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

# No proper test yet
#ADD_TEST( TestHessianEigenValuesImageFunction
#  ${EXECUTABLE_OUTPUT_PATH}/TestHessianEigenValuesImageFunction
#  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd )



#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestOptimallyOrientedFluxVesselnessImageFunction
  ivanOptimallyOrientedFluxVesselnessImageFunctionTest.cxx
)

TARGET_LINK_LIBRARIES( TestOptimallyOrientedFluxVesselnessImageFunction
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestOptimallyOrientedFluxVesselnessImageFunction ${EXECUTABLE_OUTPUT_PATH}/TestOptimallyOrientedFluxVesselnessImageFunction
  ${IVAN_DATA_ROOT}/Testing/Input/CircularGaussian4Tubes1-4.mhd 1 OOFVesselness.png 2.0 1
 )
  