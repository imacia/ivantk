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
// File: ivanOrientedFluxMatrixBasedVesselnessImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/01/17

#ifndef __ivanOrientedFluxMatrixBasedVesselnessImageFunction_hxx
#define __ivanOrientedFluxMatrixBasedVesselnessImageFunction_hxx

#include "ivanOrientedFluxMatrixBasedVesselnessImageFunction.h"

#include "itkVector.h"


namespace ivan
{

/** Set the Input Image */
template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::OrientedFluxMatrixBasedVesselnessImageFunction()
{
  this->m_VectorField = VectorFieldType::New(); 
}


/** Print self method */
template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
void
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
  
}


/** Set the input image */
template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
void
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::SetInputImage( const InputImageType * ptr )
{
  Superclass::SetInputImage( ptr );
  
}


template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
void
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::SetVectorField( VectorFieldType * vectorField )
{
  this->m_VectorField = vectorField;  
}


/** Evaluate the function at the specifed index */
template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
typename OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>::OutputType
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::EvaluateAtIndex( const IndexType& index ) const
{
  return this->EvaluateVesselnessAtIndex( this->EvaluateFluxMatrixAtIndex( index ), index );
}


/** Evaluate the function at the specifed point */
template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
typename OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>::OutputType
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::Evaluate( const PointType& point ) const
{
  return this->EvaluateVesselness( this->EvaluateFluxMatrix( point ), point ); 
}


/** Evaluate the function at specified ContinousIndex position.*/
template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
typename OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>::OutputType
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::EvaluateAtContinuousIndex( const ContinuousIndexType & cindex ) const
{
  return this->EvaluateVesselnessAtContinuousIndex
    ( this->EvaluateFluxMatrixAtContinuousIndex( cindex ), cindex );
}


/** Evaluate the function at the specifed point */
template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
typename OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>::FluxMatrixType
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::EvaluateFluxMatrix( const PointType& point ) const
{
  typename SphereType::PointsContainer *spherePoints = this->m_Sphere->GetPoints();
  typename SphereType::PointsContainerIterator pointIterator = spherePoints->Begin();
  
  this->m_SphereNormals.clear();
  this->m_SphereNormals.resize( spherePoints->Size() );
    
  NormalVectorType  currentNormal;
  PointType         currentPoint;
  VectorType        currentVector; // for the field, i.e. the gradient vector
  
  unsigned int pointIdx = 0;
  
  // Initialize Q matrix
  FluxMatrixType fluxMatrix;
  fluxMatrix.Fill( 0.0 );
  
  while( pointIterator != spherePoints->End() )
  {
    for( unsigned int dim = 0; dim < TInputImage::GetImageDimension(); ++dim )
      currentNormal[dim] = pointIterator.Value()[dim];
    
    currentNormal.Normalize();
    
    this->m_SphereNormals[pointIdx++] = currentNormal;
       
    // Calculate current point on sphere (sphere points have origin at (0,0,0)     
    for( unsigned int dim = 0; dim < TInputImage::GetImageDimension(); ++dim )
      currentPoint[dim] = point[dim] + this->m_Radius * currentNormal[dim];
    
    if( this->m_VectorField->IsInsideBuffer( currentPoint ) )
      currentVector = this->m_VectorField->Evaluate( currentPoint );
    else
    {
      ++pointIterator;
      continue;
    }
    
    // Compute elements of the Q matrix
    
    unsigned int idx = 0;
    
    for( unsigned int i = 0; i < TInputImage::GetImageDimension(); ++i )
    {
      for( unsigned int j = i; j < TInputImage::GetImageDimension(); ++j )
      {
        // We have to sum to the previous value since we all calculating the integral 
        // for all points on the sphere surface
        fluxMatrix[idx++] += currentVector[i] * currentNormal[j];
      }      
    }
        
    ++pointIterator;    
  }

  return fluxMatrix;
}


/** Evaluate the function at the specifed index */
template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
typename OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>::FluxMatrixType
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::EvaluateFluxMatrixAtIndex( const IndexType& index ) const
{
  PointType point;
  this->GetInputImage()->TransformIndexToPhysicalPoint( index, point );
  return this->EvaluateFluxMatrix( point );  
}


/** Evaluate the function at specified ContinousIndex position.*/
template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
typename OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>::FluxMatrixType
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::EvaluateFluxMatrixAtContinuousIndex( const ContinuousIndexType & cindex ) const
{
  PointType point;
  this->GetInputImage()->TransformContinuousIndexToPhysicalPoint( cindex, point );
  return this->EvaluateFluxMatrix( point );
}


template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
typename OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>::OutputType
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::EvaluateVesselnessAtIndex( const FluxMatrixType & fluxMatrix, const IndexType & index ) const
{
  PointType point;
  this->GetInputImage()->TransformIndexToPhysicalPoint( index, point );
  return this->EvaluateVesselness( fluxMatrix, point );
}


template <class TInputImage, class TVectorField, class TOutput, class TCoordRep>
typename OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>::OutputType
OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
::EvaluateVesselnessAtContinuousIndex( const FluxMatrixType & fluxMatrix, const ContinuousIndexType & cindex ) const
{
  PointType point;
  this->GetInputImage()->TransformContinuousIndexToPhysicalPoint( cindex, point );
  return this->EvaluateVesselness( fluxMatrix, point );
}

} // end namespace ivan

#endif
