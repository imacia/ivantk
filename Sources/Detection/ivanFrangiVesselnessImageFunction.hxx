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
// File: ivanFrangiVesselnessImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanFrangiVesselnessImageFunction_hxx
#define __ivanFrangiVesselnessImageFunction_hxx

#include "ivanFrangiVesselnessImageFunction.h"

#include "itkSymmetricEigenAnalysis.h"


namespace ivan
{
  
template <class TInputImage, class TOutput, class TCoordRep>
FrangiVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::FrangiVesselnessImageFunction() :
  m_Alpha( 0.5 ),
  m_Beta( 2.0 ),
  m_Gamma( 1.0 )
{
  
}


template <class TInputImage, class TOutput, class TCoordRep>
FrangiVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::~FrangiVesselnessImageFunction()
{
  
}


/** Print self method */
template <class TInputImage, class TOutput, class TCoordRep>
void
FrangiVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
  os << indent << "Alpha: " << m_Alpha << std::endl;
  os << indent << "Beta: " << m_Beta << std::endl;
  os << indent << "Gamma: " << m_Gamma << std::endl;  
}


template <class TInputImage, class TOutput, class TCoordRep>
typename FrangiVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
FrangiVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateVesselness( const HessianTensorType & hessian ) const
{
  typedef typename HessianTensorType::EigenValuesArrayType   EigenValuesArrayType;
  EigenValuesArrayType   eigenValues;
  
  // Calculate eigenvalues of Hessian matrix at the current location
  // We cannot use SymmetricSecondRankTensor::ComputeEigenValues() since currently it computes
  // eigenvalues ordering by value and not by absolute value which is required by this filter
  
  //hessian.ComputeEigenValues( eigenValues );
  
  typedef typename HessianTensorType::SymmetricEigenAnalysisType EigenAnalysisType;
  typedef typename HessianTensorType::MatrixType                 MatrixType;
    
  EigenAnalysisType eigenAnalysis = EigenAnalysisType( HessianTensorType::Dimension );
    
  MatrixType hessianMatrix;
  for( unsigned int row=0; row < HessianTensorType::Dimension; row++ )
  {
    for( unsigned int col=0; col < HessianTensorType::Dimension; col++ )
    {
      hessianMatrix[row][col] = hessian(row,col);
    }
  }
  
  eigenAnalysis.SetOrderEigenMagnitudes( true );
  eigenAnalysis.ComputeEigenValues( hessianMatrix, eigenValues );     
  
  if( vnl_math_abs( eigenValues[2] ) < 1e-3 )
    return 0.0;
    
  double lineMeasure = 0.0;
  
  // Term that filters plate-like structures
  lineMeasure *= 1.0 - vcl_exp( - 0.5 * vnl_math_sqr( vnl_math_abs( eigenValues[1] ) / 
    ( m_Alpha * vnl_math_abs( eigenValues[2] ) ) ) );
    
  // Term that filters blob-like structures
  lineMeasure *= vcl_exp( - 0.5 * vnl_math_sqr( eigenValues[0] / 
    ( m_Beta * vcl_sqrt( vnl_math_abs( eigenValues[1] * eigenValues[2] ) ) ) ) );
  
  // Term that filters noise
  
  double frobeniusNorm = 0.0;
  
  for( unsigned int i=0; i < HessianTensorType::Dimension; i++ )
  {
    frobeniusNorm += eigenValues[i] * eigenValues[i];
  }
  
  frobeniusNorm = vcl_sqrt( frobeniusNorm );
  
  lineMeasure *= 1.0 - vcl_exp( - 0.5 * vnl_math_sqr( frobeniusNorm / m_Gamma ) );
    
  return lineMeasure;   
}

} // end namespace ivan

#endif
