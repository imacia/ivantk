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
  ivanVesselCommon.h
  ivanVesselDataObject.h
  ivanVesselDataObjectSource.h
  ivanVesselDataObjectSource.hxx
)

ADD_LIBRARY( ivanCommon ${IVAN_COMMON_SRCS} )

TARGET_LINK_LIBRARIES( ivanCommon ${ITK_LIBRARIES} )
