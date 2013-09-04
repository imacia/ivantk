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
// File: ivanSatoVesselnessImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanSatoVesselnessImageFunction_hxx
#define __ivanSatoVesselnessImageFunction_hxx

#include "ivanSatoVesselnessImageFunction.h"

#define ZERO_TOLERANCE 1e-2

namespace ivan
{
  
template <class TInputImage, class TOutput, class TCoordRep>
SatoVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::SatoVesselnessImageFunction() :
  m_Gamma12( 1.0 ),
  m_Gamma23( 1.0 ),
  m_Alpha( 0.25 ),
  m_FilterByEigenValues( true )
{
  
}


template <class TInputImage, class TOutput, class TCoordRep>
SatoVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::~SatoVesselnessImageFunction()
{
  
}


/** Print self method */
template <class TInputImage, class TOutput, class TCoordRep>
void
SatoVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
  os << indent << "Gamma12: " << m_Gamma12 << std::endl;
  os << indent << "Gamma23: " << m_Gamma23 << std::endl;
  os << indent << "Alpha: " << m_Alpha << std::endl;
  os << indent << "FilterByEigenValues: " << m_FilterByEigenValues << std::endl;
}


template <class TInputImage, class TOutput, class TCoordRep>
typename SatoVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
SatoVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateVesselness( const HessianTensorType & hessian ) const
{
  typedef typename HessianTensorType::EigenValuesArrayType   EigenValuesArrayType;
  EigenValuesArrayType   eigenValues;
  
  // Calculate eigenvalues of Hessian matrix at the current location
  hessian.ComputeEigenValues( eigenValues );
  
  if( this->m_FilterByEigenValues )
  {
    if( eigenValues[0] >= 0.0 || eigenValues[1] >= 0.0 || eigenValues[2] >= vnl_math_abs( eigenValues[1] ) / m_Alpha )
      return 0.0;
  }
      
  double lineMeasure = vnl_math_abs( eigenValues[0] ) * vcl_pow( eigenValues[1] / eigenValues[0], m_Gamma23 );
  double mantissa;
  
  if( eigenValues[2] > 0.0 )
  {
    if( eigenValues[2] * m_Alpha < vnl_math_abs( eigenValues[1] ) )
      mantissa = 1.0 - m_Alpha * eigenValues[2] / vnl_math_abs( eigenValues[1] );
    else
    {
      lineMeasure = 0.0;
      return lineMeasure;
    }
  }
  else
    mantissa = 1.0 + eigenValues[2] / vnl_math_abs( eigenValues[1] );
    
  lineMeasure *= vcl_pow( mantissa ,m_Gamma12 );
  
  return lineMeasure;  
}

} // end namespace ivan

#endif
