#------------------------------------------------------------------------------------------------
   
ADD_EXECUTABLE( TestCircularGaussianSection ivanCircularGaussianSectionTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularGaussianSection
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularGaussianTube ivanCircularGaussianTubeTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularGaussianTube
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularGaussianTubeSinusoidalRadius ivanCircularGaussianTubeSinusoidalRadiusTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularGaussianTubeSinusoidalRadius
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularBarConvolvedTubeSinusoidalRadius ivanCircularBarConvolvedTubeSinusoidalRadiusTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularBarConvolvedTubeSinusoidalRadius
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularBarSection ivanCircularBarSectionTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularBarSection
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularBarTube ivanCircularBarTubeTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularBarTube
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularBarConvolvedTube ivanCircularBarConvolvedTubeTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularBarConvolvedTube
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestGaussianBall ivanGaussianBallTest.cxx )

TARGET_LINK_LIBRARIES( TestGaussianBall
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCurvedCircularGaussianTube ivanCurvedCircularGaussianTubeTest.cxx )

TARGET_LINK_LIBRARIES( TestCurvedCircularGaussianTube
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCurvedCircularGaussianTube2 ivanCurvedCircularGaussianTube2Test.cxx )

TARGET_LINK_LIBRARIES( TestCurvedCircularGaussianTube2
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularGaussianToroid 
  ivanCircularGaussianToroidTest.cxx
)

TARGET_LINK_LIBRARIES( TestCircularGaussianToroid
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularGaussianHelix 
  ivanCircularGaussianHelixTest.cxx
)

TARGET_LINK_LIBRARIES( TestCircularGaussianHelix
  ${ITK_LIBRARIES}
)

#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularBarToroid 
	ivanCircularBarToroidTest.cxx
)

TARGET_LINK_LIBRARIES( TestCircularBarToroid
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularBarStraightTube
	ivanCircularBarStraightTubeTest.cxx
)

TARGET_LINK_LIBRARIES( TestCircularBarStraightTube
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularBarHelix
	ivanCircularBarHelixTest.cxx
)

TARGET_LINK_LIBRARIES( TestCircularBarHelix
  ${ITK_LIBRARIES}
)


  