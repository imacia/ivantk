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
// File: ivanHessianBasedVesselnessImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanHessianBasedVesselnessImageFunction_hxx
#define __ivanHessianBasedVesselnessImageFunction_hxx

#include "ivanHessianBasedVesselnessImageFunction.h"


namespace ivan
{

/** Set the Input Image */
template <class TInputImage, class TOutput, class TCoordRep>
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::HessianBasedVesselnessImageFunction() :
  m_Sigma( 2.0 )
{
  m_HessianFunction = HessianFunctionType::New();
  m_HessianFunction->NormalizeAcrossScaleOn();
  m_HessianFunction->UseImageSpacingOn();
  m_HessianFunction->SetInterpolationMode( HessianFunctionType::LinearInterpolation );
  m_HessianFunction->SetSigma( m_Sigma );
}


/** Print self method */
template <class TInputImage, class TOutput, class TCoordRep>
void
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  os << indent << "Sigma: " << this->m_Sigma << std::endl;
  os << indent << "HessianFunction: " << this->m_HessianFunction.GetPointer() << std::endl;
  this->m_HessianFunction->Print( os, indent.GetNextIndent() );  
}


/** Set the input image */
template <class TInputImage, class TOutput, class TCoordRep>
void
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::SetInputImage( const InputImageType * ptr )
{
  Superclass::SetInputImage( ptr );
  m_HessianFunction->SetInputImage( ptr );
}


/** Evaluate the function at the specifed index */
template <class TInputImage, class TOutput, class TCoordRep>
typename HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateAtIndex( const IndexType& index ) const
{
  return this->EvaluateVesselnessAtIndex( this->EvaluateHessianAtIndex( index ), index );
}


/** Evaluate the function at the specifed point */
template <class TInputImage, class TOutput, class TCoordRep>
typename HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::Evaluate( const PointType& point ) const
{
  return this->EvaluateVesselness( this->EvaluateHessian( point ), point );
}


/** Evaluate the function at specified ContinousIndex position.*/
template <class TInputImage, class TOutput, class TCoordRep>
typename HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateAtContinuousIndex( const ContinuousIndexType & cindex ) const
{
  return this->EvaluateVesselnessAtContinuousIndex
    ( this->EvaluateHessianAtContinuousIndex( cindex ), cindex );
}


/** Evaluate the function at the specifed index */
template <class TInputImage, class TOutput, class TCoordRep>
typename HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::HessianTensorType
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateHessianAtIndex( const IndexType& index ) const
{
  if( m_HessianFunction->IsInsideBuffer( index ) )
    return m_HessianFunction->EvaluateAtIndex( index );
  else
  {
    HessianTensorType hessian;
    hessian.Fill( 0.0 );
    return hessian;
  }
}


/** Evaluate the function at the specifed point */
template <class TInputImage, class TOutput, class TCoordRep>
typename HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::HessianTensorType
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateHessian( const PointType& point ) const
{
  if( m_HessianFunction->GetInterpolationMode() == 
    HessianFunctionType::NearestNeighbourInterpolation )
    {
    IndexType index;
    this->ConvertPointToNearestIndex( point , index );
    return this->EvaluateHessianAtIndex ( index );
    }
  else // linear interpolation
    {
    ContinuousIndexType cindex;
    this->ConvertPointToContinuousIndex( point, cindex );
    return this->EvaluateHessianAtContinuousIndex( cindex );
    }
}


/** Evaluate the function at specified ContinousIndex position.*/
template <class TInputImage, class TOutput, class TCoordRep>
typename HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::HessianTensorType
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateHessianAtContinuousIndex( const ContinuousIndexType & cindex ) const
{
  if( m_HessianFunction->GetInterpolationMode() == 
    HessianFunctionType::NearestNeighbourInterpolation )
    {
    IndexType index;
    this->ConvertContinuousIndexToNearestIndex( cindex, index  );
    return this->EvaluateHessianAtIndex( index );
    }
  else
    {
    unsigned int dim;  // index over dimension
    unsigned long neighbors = 1 << ImageDimension;

    // Compute base index = closet index below point
    // Compute distance from point to base index
    signed long baseIndex[ImageDimension];
    double distance[ImageDimension];
    long tIndex;

    for( dim = 0; dim < ImageDimension; dim++ )
      {
      // The following "if" block is equivalent to the following line without
      // having to call floor.
      //    baseIndex[dim] = (long) vcl_floor(index[dim] );
      if (cindex[dim] >= 0.0)
        {
        baseIndex[dim] = (long) cindex[dim];
        }
      else
        {
        tIndex = (long) cindex[dim];
        if (double(tIndex) != cindex[dim])
          {
          tIndex--;
          }
        baseIndex[dim] = tIndex;
        }
      distance[dim] = cindex[dim] - double( baseIndex[dim] );
      }

    // Interpolated value is the weighted sum of each of the surrounding
    // neighbors. The weight for each neighbor is the fraction overlap
    // of the neighbor pixel with respect to a pixel centered on point.
    HessianTensorType hessian, currentHessian;
    double totalOverlap = 0.0;

    for( unsigned int counter = 0; counter < neighbors; counter++ )
      {
      double overlap = 1.0;          // fraction overlap
      unsigned int upper = counter;  // each bit indicates upper/lower neighbour
      IndexType neighIndex;

      // get neighbor index and overlap fraction
      for( dim = 0; dim < ImageDimension; dim++ )
        {
        if ( upper & 1 )
          {
          neighIndex[dim] = baseIndex[dim] + 1;
          overlap *= distance[dim];
          }
        else
          {
          neighIndex[dim] = baseIndex[dim];
          overlap *= 1.0 - distance[dim];
          }
        upper >>= 1;
        }

      // get neighbor value only if overlap is not zero
      if( overlap )
        {
        currentHessian = this->EvaluateHessianAtIndex( neighIndex );
        for( unsigned int i=0; i<hessian.Size(); ++i )
          hessian[i] += overlap * currentHessian[i];
        totalOverlap += overlap;
        }
  
      if( totalOverlap == 1.0 )
        {
        // finished
        break;
        }
      }

    return hessian;
    }
}


template <class TInputImage, class TOutput, class TCoordRep>
typename HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateVesselness( const HessianTensorType & hessian, const PointType & point ) const
{
  ContinuousIndexType cindex;
  this->ConvertPointToContinuousIndex( point, cindex );
  
  return this->EvaluateVesselnessAtContinuousIndex( hessian, cindex );
}


template <class TInputImage, class TOutput, class TCoordRep>
typename HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateVesselnessAtIndex( const HessianTensorType & hessian, const IndexType & index ) const
{
  // Convert point to nearest index and calculate
  ContinuousIndexType cindex;
  
  for( unsigned int i=0; i < TInputImage::GetImageDimension(); ++i )
    cindex[i] = static_cast<typename ContinuousIndexType::ValueType>( index[i] );
  
  return this->EvaluateVesselnessAtContinuousIndex( hessian, cindex );
}

} // end namespace ivan

#endif
