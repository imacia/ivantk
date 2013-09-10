#------------------------------------------------------------------------------------------------
# Set the link libraries for the tests

SET( IVAN_TEST_LINK_LIBRARIES
  ivanCommon
  ivanITK
  ivanModeling
)

SET( IVAN_TEST_DATA_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/Data
)

SUBDIRS(
  Detection
  Extraction
  ITK
  Modeling
  Synthetic
)
 