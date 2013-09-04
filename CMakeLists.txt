PROJECT( IVAN )

CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

#-----------------------------------------------------------------------------
# Single directory for library and executable outputs
SET( LIBRARY_OUTPUT_PATH ${IVAN_BINARY_DIR}/bin/ CACHE PATH "Single output directory for building all libraries." FORCE )
SET( EXECUTABLE_OUTPUT_PATH ${IVAN_BINARY_DIR}/bin/ CACHE PATH "Single output directory for building all executables." FORCE )


#-----------------------------------------------------------------------------
# Change the name of the library postfix to have a 'd' in order to be different for Debug and Release
SET( CMAKE_DEBUG_POSTFIX d )


#-----------------------------------------------------------------------------
# Force output binary directory to be the same for all targets
#
SET( LIBRARY_OUTPUT_PATH ${IVAN_BINARY_DIR}/bin/ CACHE PATH "Single output directory for building all libraries." FORCE )
SET( EXECUTABLE_OUTPUT_PATH ${IVAN_BINARY_DIR}/bin/ CACHE PATH "Single output directory for building all executables." FORCE )


#-----------------------------------------------------------------------------
# Include custom CMake macros

INCLUDE( ivanCMakeMacros.cmake )


#-----------------------------------------------------------------------------
# IVAN version number.

SET( IVAN_MAJOR_VERSION 0 )
SET( IVAN_MINOR_VERSION 1 )
SET( IVAN_PATCH_VERSION 0 )
SET( IVAN_BUILD_VERSION 0 )
SET( IVAN_VERSION_STRING "${IVAN_MAJOR_VERSION}.${IVAN_MINOR_VERSION}.${IVAN_PATCH_VERSION}.${IVAN_BUILD_VERSION}")
SET( IVAN_SHORT_VERSION_STRING "${IVAN_MAJOR_VERSION}.${IVAN_MINOR_VERSION}.${IVAN_PATCH_VERSION}")
##Backward compatibility
SET( IVAN_VERSION "${IVAN_MAJOR_VERSION}.${IVAN_MINOR_VERSION}.${IVAN_PATCH_VERSION}.${IVAN_BUILD_VERSION}")

MESSAGE(STATUS "Configuring IVAN Version " ${IVAN_VERSION_STRING})


#-----------------------------------------------------------------------------
# Find and use VTK libraries 
#
FIND_PACKAGE( VTK )
IF ( VTK_FOUND )
  INCLUDE ( ${USE_VTK_FILE} )
ENDIF ()


#-----------------------------------------------------------------------------
# Find and use ITK libraries 
#
FIND_PACKAGE( ITK REQUIRED )
IF ( ITK_FOUND )
  INCLUDE ( ${USE_ITK_FILE} )
ELSE ()
  MESSAGE( FATAL_ERROR "This application requires ITK. One of these components is missing. Please verify configuration")
ENDIF ()


#-----------------------------------------------------------------------------
# Find and use Qt libraries
#
SET( QT_USE_QTOPENGL 1 )

FIND_PACKAGE( Qt REQUIRED )
IF( QT_USE_FILE )
  INCLUDE( ${QT_USE_FILE} )
ELSE( QT_USE_FILE )
  SET( QT_LIBRARIES  ${QT_QT_LIBRARY})
ENDIF( QT_USE_FILE )


#-----------------------------------------------------------------------------
# Find and use Doxygen
#
FIND_PACKAGE( Doxygen )

IF( DOXYGEN_FOUND )

  FIND_HHC_LOCATION()

  IF(EXISTS "${HHC_LOCATION}")
    SET(GENERATE_CHI "YES")
    SET(CHM_FILE "${CMAKE_CURRENT_BINARY_DIR}/Doxygen/IVAN-${IVAN_SHORT_VERSION_STRING}.chm")
  ELSE(EXISTS "${HHC_LOCATION}")
    SET(GENERATE_CHI "NO")
    SET(CHM_FILE "")
    SET(HHC_LOCATION "")
  ENDIF(EXISTS "${HHC_LOCATION}")

  FIND_PACKAGE(Perl QUIET)

  IF(EXISTS "${PERL_EXECUTABLE}")
    SET(GENERATE_PERLMOD "YES")
  ELSE(EXISTS "${PERL_EXECUTABLE}")
    SET(GENERATE_PERLMOD "NO")
  ENDIF(EXISTS "${PERL_EXECUTABLE}")

  SET(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/Doxygen CACHE PATH "Output path for doxygen.")

  CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/Doc/Doxyfile.in ${DOXYGEN_OUTPUT_DIRECTORY}/Doxyfile @ONLY )

  ADD_CUSTOM_TARGET( COMPILE_DOXYGEN 
    ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUTPUT_DIRECTORY}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
  )

ENDIF( DOXYGEN_FOUND )


#-----------------------------------------------------------------------------
# Configure the path for the MIST data used for testing etc.

SET( IVAN_DATA_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Data CACHE PATH "Directory for IVAN data." )
SET( IVAN_TEST_DATA_DIR ${IVAN_DATA_ROOT}/Testing/Input )
  

#-----------------------------------------------------------------------------
# Enable testing
ENABLE_TESTING()


#-----------------------------------------------------------------------------
# Create the list of include directories needed for MIST header files.

INCLUDE( ${IVAN_SOURCE_DIR}/IVANIncludeDirectories.cmake )

# This should be the only INCLUDE_DIRECTORIES command in the entire
# tree. We need to do this in one place to make sure the order is correct.

INCLUDE_DIRECTORIES( BEFORE
  ${IVAN_INCLUDE_DIRS_BUILD_TREE}
  ${IVAN_INCLUDE_DIRS_BUILD_TREE_CXX}
  ${IVAN_INCLUDE_DIRS_SYSTEM}
)


#-----------------------------------------------------------------------------
# Help other projects use IvanLib

SET( IVAN_LIBRARY_PATH ${LIBRARY_OUTPUT_PATH} )

# Copy the UseMIST.cmake file to the binary tree for backward compatability.
CONFIGURE_FILE(${IVAN_SOURCE_DIR}/UseIVAN.cmake.in
               ${IVAN_BINARY_DIR}/UseIVAN.cmake COPYONLY IMMEDIATE)

# Save library dependencies.
EXPORT_LIBRARY_DEPENDENCIES( ${IVAN_BINARY_DIR}/IVANLibraryDepends.cmake )

# Create the IVANConfig.cmake file containing the IVAN configuration.
INCLUDE( ${IVAN_SOURCE_DIR}/mistGenerateIVANConfig.cmake )

SUBDIRS( 
  Sources
  Testing  
  Utilities
)

IF( VTK_FOUND )
  SUBDIRS(
    Visualization
  )
ENDIF( VTK_FOUND )


