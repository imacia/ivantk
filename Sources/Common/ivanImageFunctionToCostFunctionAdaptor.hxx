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
// File: ivanImageFunctionToCostFunctionAdaptor.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/05/27


#ifndef __ivanImageFunctionToCostFunctionAdaptor_hxx
#define __ivanImageFunctionToCostFunctionAdaptor_hxx

#include "ivanImageFunctionToCostFunctionAdaptor.h"


namespace ivan
{

/**
 *
 */
template <class TImageFunction>
ImageFunctionToCostFunctionAdaptor<TImageFunction>
::ImageFunctionToCostFunctionAdaptor()
{

}


/**
 *
 */
template <class TImageFunction>
typename ImageFunctionToCostFunctionAdaptor<TImageFunction>::MeasureType
ImageFunctionToCostFunctionAdaptor<TImageFunction>
::GetValue( const ParametersType & parameters ) const
{
  PointType point;
  
  for( unsigned int i=0; i<ImageType::GetImageDimension(); ++i )
    point[i] = static_cast<typename PointType::ValueType>( parameters[i] );
    
  // Bounds check
  if( this->GetImageFunction()->IsInsideBuffer( point ) )
    return static_cast<MeasureType>( this->GetImageFunction()->Evaluate( point ) );
  else 
    return itk::NumericTraits<MeasureType>::min();
}


/**
 *
 */
template <class TImageFunction>
void 
ImageFunctionToCostFunctionAdaptor<TImageFunction>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
}

} // end namespace ivan

#endif
