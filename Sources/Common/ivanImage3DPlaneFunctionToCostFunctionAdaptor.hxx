/*=========================================================================

Image-based Vascular Analysis Toolkit (IVAN)

Copyright (c) 2012, Iván Macía Oliver
Vicomtech Foundation, San Sebastián - Donostia (Spain)
University of the Basque Country, San Sebastián - Donostia (Spain)

All rights reserved

See LICENSE file for license details

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.

==========================================================================*/
// File: ivanImage3DPlaneFunctionToCostFunctionAdaptor.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/06/12


#ifndef __ivanImage3DPlaneFunctionToCostFunctionAdaptor_hxx
#define __ivanImage3DPlaneFunctionToCostFunctionAdaptor_hxx

#include "ivanImage3DPlaneFunctionToCostFunctionAdaptor.h"
#include "ivanGlobals.h"

#include "vnl/vnl_vector_fixed.h"


namespace ivan
{

/**
 *
 */
template<class TImage>
Image3DPlaneFunctionToCostFunctionAdaptor<TImage>
::Image3DPlaneFunctionToCostFunctionAdaptor()
{
  this->m_PlaneCenter.Fill( 0.0 );
  this->m_PlaneNormal.Fill( 0.0 );
  this->m_PlaneFirstBaseVector.Fill( 0.0 );
  this->m_PlaneSecondBaseVector.Fill( 0.0 );
}


/**
 *
 */
template<class TImage>
void
Image3DPlaneFunctionToCostFunctionAdaptor<TImage>
::SetPlaneNormal( const VectorType & normal )
{
  this->m_PlaneNormal = normal;
  
  // Compute the plane base
  
  typedef vnl_vector_fixed<double,3>  VnlVectorType;
  
  VnlVectorType  normalVector, firstBaseVector, secondBaseVector;
  
  normalVector[0] = this->m_PlaneNormal[0];
	normalVector[1] = this->m_PlaneNormal[1];
	normalVector[2] = this->m_PlaneNormal[2];
	
	// Calculate an orthonormal base of vectors in the plane of the section given the normal vector
  ComputePlaneBasisVectorsFromNormal( normalVector, firstBaseVector, secondBaseVector );
	
  this->m_PlaneFirstBaseVector[0] = firstBaseVector[0];
  this->m_PlaneFirstBaseVector[1] = firstBaseVector[1];
  this->m_PlaneFirstBaseVector[2] = firstBaseVector[2];

  this->m_PlaneSecondBaseVector[0] = secondBaseVector[0];
  this->m_PlaneSecondBaseVector[1] = secondBaseVector[1];
  this->m_PlaneSecondBaseVector[2] = secondBaseVector[2];

  this->Modified(); 
}


/**
 *
 */
template<class TImage>
typename Image3DPlaneFunctionToCostFunctionAdaptor<TImage>::MeasureType
Image3DPlaneFunctionToCostFunctionAdaptor<TImage>
::GetValue( const ParametersType & parameters ) const
{
  // P(X) = P0(X) + alpha * e1(X) + beta * e2(X) where alpha = parameters[0] and beta = parameters[1]
  
  PointType point;
  
  for( unsigned int dim=0; dim < ImageType::GetImageDimension(); ++dim )
    point[dim] = this->m_PlaneCenter[dim] + 
      static_cast<typename PointType::ValueType>( parameters[0] ) * this->m_PlaneFirstBaseVector[dim] +
      static_cast<typename PointType::ValueType>( parameters[1] ) * this->m_PlaneSecondBaseVector[dim];
  
  // Bounds check
  if( this->GetImageFunction()->IsInsideBuffer( point ) )
    return static_cast<MeasureType>( this->GetImageFunction()->Evaluate( point ) );
  else 
    return itk::NumericTraits<MeasureType>::min();
}


template <class TImage>
typename Image3DPlaneFunctionToCostFunctionAdaptor<TImage>::BaseMatrixType
Image3DPlaneFunctionToCostFunctionAdaptor<TImage>::GetPlaneBaseMatrix() const
{
  BaseMatrixType baseMatrix;

  for( unsigned int dim = 0; dim < ImageType::ImageDimension; ++dim )
	{
	  baseMatrix[dim][0] = this->m_PlaneFirstBaseVector[dim];
	  baseMatrix[dim][1] = this->m_PlaneSecondBaseVector[dim];
	}
	
	return baseMatrix;
}


/**
 *
 */
template<class TImage>
void 
Image3DPlaneFunctionToCostFunctionAdaptor<TImage>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
  os << indent << "Center: " << this->m_PlaneCenter << std::endl;
  os << indent << "PlaneNormal: " << this->m_PlaneNormal << std::endl;
  os << indent << "PlaneFirstBaseVector: " << this->m_PlaneFirstBaseVector << std::endl;
  os << indent << "PlaneSecondBaseVector: " << this->m_PlaneSecondBaseVector << std::endl;
}

} // end namespace ivan

#endif
