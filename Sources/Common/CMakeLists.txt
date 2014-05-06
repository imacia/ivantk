SET( IVAN_COMMON_SRCS
  ivanAngularValue.h
  ivanGlobals.h
  ivanGraphCountNodeVisitor.h
  ivanGraphCountNodeVisitor.hxx
  ivanGraphNode.cxx
  ivanGraphNode.h
  ivanGraphNodeVisitor.cxx
  ivanGraphNodeVisitor.h
  ivanGraphNodeVisitorDispatcher.h
  ivanImage3DPlaneFunctionToCostFunctionAdaptor.h
  ivanImage3DPlaneFunctionToCostFunctionAdaptor.hxx
  ivanImageFunctionToCostFunctionAdaptor.h
  ivanImageFunctionToCostFunctionAdaptor.hxx
  ivanImageToVesselDataObjectFilter.h
  ivanImageToVesselDataObjectFilter.hxx
  ivanMacros.h
  ivanVesselCommon.h
  ivanVesselDataObject.h
  ivanVesselDataObjectSource.h
  ivanVesselDataObjectSource.hxx
)

IF( IVAN_BUILD_SHARED_LIBS )
  ADD_LIBRARY( ivanCommon SHARED ${IVAN_COMMON_SRCS} )
ELSE( IVAN_BUILD_SHARED_LIBS )
  ADD_LIBRARY( ivanCommon STATIC ${IVAN_COMMON_SRCS} )
ENDIF( IVAN_BUILD_SHARED_LIBS )

TARGET_LINK_LIBRARIES( ivanCommon ${ITK_LIBRARIES} )
