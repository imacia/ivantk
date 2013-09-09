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

ADD_EXECUTABLE( TestCircularFlatConvolvedTubeSinusoidalRadius ivanCircularFlatConvolvedTubeSinusoidalRadiusTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularFlatConvolvedTubeSinusoidalRadius
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularFlatSection ivanCircularFlatSectionTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularFlatSection
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularFlatTube ivanCircularFlatTubeTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularFlatTube
  ${ITK_LIBRARIES}
)


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularFlatConvolvedTube ivanCircularFlatConvolvedTubeTest.cxx )

TARGET_LINK_LIBRARIES( TestCircularFlatConvolvedTube
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


  