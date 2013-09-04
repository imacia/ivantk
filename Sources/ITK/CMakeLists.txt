SET( IVAN_ITK_SRCS
  itkAdditiveGaussianNoiseImageFilter.h
  itkAdditiveGaussianNoiseImageFilter.txx
  itkDiscreteGaussianDerivativeImageFilter.h
  itkDiscreteGaussianDerivativeImageFilter.hxx  
  itkDiscreteGaussianDerivativeImageFunction.h  
  itkDiscreteGaussianDerivativeImageFunction.h  
  itkDiscreteGradientMagnitudeGaussianImageFunction.h  
  itkDiscreteGradientMagnitudeGaussianImageFunction.hxx  
  itkDiscreteHessianGaussianImageFunction.h
  itkDiscreteHessianGaussianImageFunction.hxx
  itkGaussianDerivativeOperator.h
  itkGaussianDerivativeOperator.hxx
  itkImageToVTKImageFilter.h
  itkImageToVTKImageFilter.hxx
  itkThreadSafeMersenneTwisterRandomVariateGenerator.h  
  itkThreadSafeMersenneTwisterRandomVariateGenerator.cxx  
  itkVTKImageToImageFilter.h
  itkVTKImageToImageFilter.hxx
)

ADD_LIBRARY( ivanITK ${IVAN_ITK_SRCS} )

TARGET_LINK_LIBRARIES( ivanITK ${ITK_LIBRARIES} )
