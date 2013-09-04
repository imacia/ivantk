#################################################################################################

ADD_EXECUTABLE( UtilityGaussianNoiseAdder 
#  itkThreadSafeMersenneTwisterRandomVariateGenerator.cxx
  GaussianNoiseAdder.cxx
)

TARGET_LINK_LIBRARIES( UtilityGaussianNoiseAdder
  ${ITK_LIBRARIES}
  ivanITK
)  
  