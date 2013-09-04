# Generate the IVANConfig.cmake file in the build tree.  Also configure
# one for installation.  The file tells external projects how to use
# IVAN.


#-----------------------------------------------------------------------------
# Settings specific to the build tree.

# The "use" file.
SET( IVAN_USE_FILE ${IVAN_BINARY_DIR}/UseIVAN.cmake )

# The library dependencies file.
SET( IVAN_LIBRARY_DEPENDS_FILE ${IVAN_BINARY_DIR}/IVANLibraryDepends.cmake )

# The build settings file.
SET( IVAN_BUILD_SETTINGS_FILE ${IVAN_BINARY_DIR}/IVANBuildSettings.cmake )

# Library directory.
SET( IVAN_LIBRARY_DIRS_CONFIG ${IVAN_LIBRARY_PATH} )

# Determine the include directories needed.
SET( IVAN_INCLUDE_DIRS_CONFIG
  ${IVAN_INCLUDE_DIRS_BUILD_TREE}
  ${IVAN_INCLUDE_DIRS_SOURCE_TREE}
  ${IVAN_INCLUDE_DIRS_SYSTEM}
)

#-----------------------------------------------------------------------------
# Configure IVANConfig.cmake for the build tree.
CONFIGURE_FILE( ${IVAN_SOURCE_DIR}/IVANConfig.cmake.in
                ${IVAN_BINARY_DIR}/IVANConfig.cmake @ONLY IMMEDIATE)

#-----------------------------------------------------------------------------
# Settings specific to the install tree.

# The "use" file.
SET( IVAN_USE_FILE ${CMAKE_INSTALL_PREFIX}/lib/IVAN/UseIVAN.cmake )

# The library dependencies file.
SET( IVAN_LIBRARY_DEPENDS_FILE
  ${CMAKE_INSTALL_PREFIX}/lib/IVAN/IVANLibraryDepends.cmake )

# The build settings file.
SET( IVAN_BUILD_SETTINGS_FILE
  ${CMAKE_INSTALL_PREFIX}/lib/IVAN/IVANBuildSettings.cmake )

# Include directories.
SET( IVAN_INCLUDE_DIRS_CONFIG
  ${IVAN_INCLUDE_DIRS_INSTALL_TREE}
  ${IVAN_INCLUDE_DIRS_SYSTEM}
)

# Link directories.
SET( IVAN_LIBRARY_DIRS_CONFIG ${CMAKE_INSTALL_PREFIX}/lib/IVAN )

#-----------------------------------------------------------------------------
# Configure IVANConfig.cmake for the install tree.
CONFIGURE_FILE( ${IVAN_SOURCE_DIR}/IVANConfig.cmake.in
  ${IVAN_BINARY_DIR}/Utilities/IVANConfig.cmake @ONLY IMMEDIATE)