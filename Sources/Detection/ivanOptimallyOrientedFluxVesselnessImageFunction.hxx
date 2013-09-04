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
// File: ivanOptimallyOrientedFluxVesselnessImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/01/18

#ifndef __ivanOptimallyOrientedFluxVesselnessImageFunction_hxx
#define __ivanOptimallyOrientedFluxVesselnessImageFunction_hxx

#include "ivanOptimallyOrientedFluxVesselnessImageFunction.h"

#include "itkVector.h"


namespace ivan
{

/** Set the Input Image */
template <class TInputImage, class TVectorField, class TEigenvalueFunctor, class TOutput, class TCoordRep>
OptimallyOrientedFluxVesselnessImageFunction<TInputImage,TVectorField,TEigenvalueFunctor,TOutput,TCoordRep>
::OptimallyOrientedFluxVesselnessImageFunction()
{

}


/** Print self method */
template <class TInputImage, class TVectorField, class TEigenvalueFunctor, class TOutput, class TCoordRep>
void
OptimallyOrientedFluxVesselnessImageFunction<TInputImage,TVectorField,TEigenvalueFunctor,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
  
}


template <class TInputImage, class TVectorField, class TEigenvalueFunctor, class TOutput, class TCoordRep>
typename OptimallyOrientedFluxVesselnessImageFunction
<TInputImage,TVectorField,TEigenvalueFunctor,TOutput,TCoordRep>::OutputType
OptimallyOrientedFluxVesselnessImageFunction<TInputImage,TVectorField,TEigenvalueFunctor,TOutput,TCoordRep>
::EvaluateVesselness( const FluxMatrixType & fluxMatrix, const PointType & point ) const
{
  EigenVectorsMatrixType   eigenVectors;
  EigenValuesArrayType     eigenValues;
  
  // We need the arrays in the form of an itk::Vector
  //VectorType  firstEigenVector, secondEigenVector, thirdEigenVector;
  
  // Calculate eigenvalues and eigenvectors of flux matrix at the current location
  fluxMatrix.ComputeEigenAnalysis( eigenValues, eigenVectors );
  
  // Now rely on the eigenvalue functor to provide a response
  return this->m_EigenvalueFunctor.Evaluate( eigenValues, eigenVectors );
}

} // end namespace ivan

#endif
