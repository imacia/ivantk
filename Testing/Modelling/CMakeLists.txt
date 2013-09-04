#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestCircularVesselSection 
  ivanCircularVesselSectionTest.cxx
)

TARGET_LINK_LIBRARIES( TestCircularVesselSection
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestCircularVesselSection ${EXECUTABLE_OUTPUT_PATH}/TestCircularVesselSection )


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestVesselGraph
  ivanVesselGraphTest.cxx
)

TARGET_LINK_LIBRARIES( TestVesselGraph
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestVesselGraph ${EXECUTABLE_OUTPUT_PATH}/TestVesselGraph )


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestVesselNodeVisitor
  ivanVesselNodeVisitorTest.cxx
)

TARGET_LINK_LIBRARIES( TestVesselNodeVisitor
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestVesselNodeVisitor ${EXECUTABLE_OUTPUT_PATH}/TestVesselNodeVisitor )


#------------------------------------------------------------------------------------------------

ADD_EXECUTABLE( TestVesselNodeVisitorSpeed
  ivanVesselNodeVisitorSpeedTest.cxx
)

TARGET_LINK_LIBRARIES( TestVesselNodeVisitorSpeed
  ${ITK_LIBRARIES}
  ${IVAN_TEST_LINK_LIBRARIES}
)

ADD_TEST( TestVesselNodeVisitorSpeed ${EXECUTABLE_OUTPUT_PATH}/TestVesselNodeVisitorSpeed )
