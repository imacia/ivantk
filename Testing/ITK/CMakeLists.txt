#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscaleDiscreteGaussianDerivativeImageFilter
  ivanMultiscaleDiscreteGaussianDerivativeImageFilterTest.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscaleDiscreteGaussianDerivativeImageFilter
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestMultiscaleDiscreteGaussianDerivativeOperatorTest
  ivanMultiscaleDiscreteGaussianDerivativeOperatorTest.cxx
)

TARGET_LINK_LIBRARIES( TestMultiscaleDiscreteGaussianDerivativeOperatorTest
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

