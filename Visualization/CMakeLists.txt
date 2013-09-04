#################################################################################################

IF( VTK_MAJOR_VERSION EQUAL 5 )
	SET( VTK_LIBRARIES
		vtkCommon
		vtkHybrid
		vtkRendering
		vtkGraphics
		vtkImaging
		vtkFiltering
		vtkGenericFiltering
		vtkVolumeRendering
		vtkWidgets
		vtkIO
	)
ELSE( VTK_MAJOR_VERSION EQUAL 5 )
	SET( VTK_LIBRARIES
		vtkCommon
		vtkHybrid
		vtkRendering
		vtkGraphics
		vtkImaging
		vtkFiltering
		vtkPatented
		vtkIO
	)
ENDIF( VTK_MAJOR_VERSION EQUAL 5 ) 

ADD_EXECUTABLE( VTKVesselTrackerPlotter 
  VTKVesselTrackerPlotter.cxx
)

TARGET_LINK_LIBRARIES( VTKVesselTrackerPlotter
  ${ITK_LIBRARIES}
  ${VTK_LIBRARIES}
  ${IVAN_LIBRARIES}
)
