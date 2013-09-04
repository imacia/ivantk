#-----------------------------------------------------------------------------
# Include directories for other projects installed on the system.
SET( IVAN_INCLUDE_DIRS_SYSTEM "" )


#-----------------------------------------------------------------------------
# Include directories from the build tree.
# These directories are always needed.
SET( IVAN_INCLUDE_DIRS_BUILD_TREE 
  ${IVAN_INCLUDE_DIRS_BUILD_TREE}
  ${IVAN_BINARY_DIR}
  ${IVAN_BINARY_DIR}/bin
  ${IVAN_SOURCE_DIR}
  ${IVAN_SOURCE_DIR}/Sources/Common
  ${IVAN_SOURCE_DIR}/Sources/Detection
  ${IVAN_SOURCE_DIR}/Sources/Experimental
  ${IVAN_SOURCE_DIR}/Sources/Extraction
  ${IVAN_SOURCE_DIR}/Sources/ITK
  ${IVAN_SOURCE_DIR}/Sources/Modelling
  ${IVAN_SOURCE_DIR}/Sources/Quantification
  ${IVAN_SOURCE_DIR}/Sources/Synthetic
)


#-----------------------------------------------------------------------------
# Include directories needed for .cxx files in IVAN.  These include
# directories will NOT be available to user projects.

SET( IVAN_INCLUDE_DIRS_BUILD_TREE_CXX "" )


#-----------------------------------------------------------------------------
# Include directories from the install tree.

SET( IVAN_INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/IVAN" )

SET( IVAN_INCLUDE_DIRS_INSTALL_TREE 
  ${IVAN_INCLUDE_DIRS_INSTALL_TREE}
  ${IVAN_INSTALL_INCLUDE_DIR}
  ${IVAN_INSTALL_INCLUDE_DIR}/Common
  ${IVAN_INSTALL_INCLUDE_DIR}/Detection
  ${IVAN_INSTALL_INCLUDE_DIR}/Experimental
  ${IVAN_INSTALL_INCLUDE_DIR}/Extraction
  ${IVAN_INSTALL_INCLUDE_DIR}/ITK
  ${IVAN_INSTALL_INCLUDE_DIR}/Modelling
  ${IVAN_INSTALL_INCLUDE_DIR}/Quantification
  ${IVAN_INSTALL_INCLUDE_DIR}/Synthetic
)

