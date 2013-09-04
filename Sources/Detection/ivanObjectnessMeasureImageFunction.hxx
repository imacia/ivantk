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
// File: ivanObjectnessMeasureImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanObjectnessMeasureImageFunction_hxx
#define __ivanObjectnessMeasureImageFunction_hxx

#include "ivanObjectnessMeasureImageFunction.h"

#include "itkSymmetricEigenAnalysis.h"


namespace ivan
{
  
template <class TInputImage, class TOutput, class TCoordRep>
ObjectnessMeasureImageFunction<TInputImage,TOutput,TCoordRep>
::ObjectnessMeasureImageFunction() :
  m_Alpha( 0.5 ),
  m_Beta( 2.0 ),
  m_Gamma( 1.0 ),
  m_ObjectDimension( 1 )
{
  
}


template <class TInputImage, class TOutput, class TCoordRep>
ObjectnessMeasureImageFunction<TInputImage,TOutput,TCoordRep>
::~ObjectnessMeasureImageFunction()
{
  
}


/** Print self method */
template <class TInputImage, class TOutput, class TCoordRep>
void
ObjectnessMeasureImageFunction<TInputImage,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
  os << indent << "Alpha: " << m_Alpha << std::endl;
  os << indent << "Beta: " << m_Beta << std::endl;
  os << indent << "Gamma: " << m_Gamma << std::endl;  
  os << indent << "ObjectDimension: " << m_ObjectDimension << std::endl;  
}


template <class TInputImage, class TOutput, class TCoordRep>
typename ObjectnessMeasureImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
ObjectnessMeasureImageFunction<TInputImage,TOutput,TCoordRep>
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
  
  //if( vnl_math_abs( eigenValues[2] ) < 1e-3 )
    //return 0.0;
  
  // initialize the objectness measure
  double objectnessMeasure = 1.0;

  // compute objectness from eigenvalue ratios and second-order structureness
  if( m_ObjectDimension < ImageDimension-1 )
  {
    double rA = eigenValues[m_ObjectDimension];
    double rADenominatorBase = 1.0;
    for ( unsigned int j=m_ObjectDimension+1; j<ImageDimension; j++ )
    {
      rADenominatorBase *= eigenValues[j];
    }
    
    if (vcl_fabs(rADenominatorBase) > 0.0)
    {
      if ( vcl_fabs( m_Alpha ) > 0.0 )
        {
        rA /= vcl_pow(rADenominatorBase, 1.0 / (ImageDimension-m_ObjectDimension-1));
        objectnessMeasure *= 1.0 - vcl_exp(- 0.5 * vnl_math_sqr(rA) / vnl_math_sqr(m_Alpha));
        }
      }
      else
      {
      objectnessMeasure = 0.0;
      }
  }

  if (m_ObjectDimension > 0)
  {
    double rB = eigenValues[m_ObjectDimension-1];
    double rBDenominatorBase = 1.0;
    for (unsigned int j=m_ObjectDimension; j<ImageDimension; j++)
    {
      rBDenominatorBase *= eigenValues[j];
    }
    if (vcl_fabs(rBDenominatorBase) > 0.0 && vcl_fabs( m_Beta ) > 0.0 )
    {
      rB /= vcl_pow(rBDenominatorBase, 1.0 / (ImageDimension-m_ObjectDimension) );

      objectnessMeasure *= vcl_exp(- 0.5 * vnl_math_sqr(rB) / vnl_math_sqr(m_Beta) );
    }
    else
    {
      objectnessMeasure = 0.0;
    }
  }

  if ( vcl_fabs( m_Gamma ) > 0.0 )
  {
    double frobeniusNormSquared = 0.0;
    for (unsigned int i=0; i<ImageDimension; i++)
    {
      frobeniusNormSquared += vnl_math_sqr(sortedAbsEigenValues[i]);
    }
    objectnessMeasure *= 1.0 - vcl_exp(- 0.5 * frobeniusNormSquared / vnl_math_sqr(m_Gamma));
  }

  return objectnessMeasure;   
}

} // end namespace ivan

#endif
