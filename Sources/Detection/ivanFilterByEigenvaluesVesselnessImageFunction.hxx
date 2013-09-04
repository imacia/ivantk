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
// File: ivanFilterByEigenvaluesVesselnessImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/08/16

#ifndef __ivanFilterByEigenvaluesVesselnessImageFunction_hxx
#define __ivanFilterByEigenvaluesVesselnessImageFunction_hxx

#include "ivanFilterByEigenvaluesVesselnessImageFunction.h"


namespace ivan
{
  
template <class TInputImage, class TOutput, class TFunctor, class TCoordRep>
FilterByEigenvaluesVesselnessImageFunction<TInputImage,TOutput,TFunctor,TCoordRep>
::FilterByEigenvaluesVesselnessImageFunction() :
  m_OutputValue( 255.0 )
{

}


template <class TInputImage, class TOutput, class TFunctor, class TCoordRep>
FilterByEigenvaluesVesselnessImageFunction<TInputImage,TOutput,TFunctor,TCoordRep>
::~FilterByEigenvaluesVesselnessImageFunction()
{
  
}


/** Print self method */
template <class TInputImage, class TOutput, class TFunctor, class TCoordRep>
void
FilterByEigenvaluesVesselnessImageFunction<TInputImage,TOutput,TFunctor,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
  os << indent << "OutputValue: " << m_OutputValue << std::endl;
}


template <class TInputImage, class TOutput, class TFunctor, class TCoordRep>
typename FilterByEigenvaluesVesselnessImageFunction<TInputImage,TOutput,TFunctor,TCoordRep>::OutputType
FilterByEigenvaluesVesselnessImageFunction<TInputImage,TOutput,TFunctor,TCoordRep>
::EvaluateVesselness( const HessianTensorType & hessian ) const
{
  typedef typename HessianTensorType::EigenValuesArrayType   EigenValuesArrayType;
  EigenValuesArrayType   eigenValues;
  
  // Calculate eigenvalues of Hessian matrix at the current location
  hessian.ComputeEigenValues( eigenValues );
  
  if( m_Functor( eigenValues ) )
    return m_OutputValue;
  else
    return 0.0;
}

} // end namespace ivan

#endif
