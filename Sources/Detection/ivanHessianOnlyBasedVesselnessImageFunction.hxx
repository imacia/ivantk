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
// File: ivanHessianOnlyBasedVesselnessImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/19

#ifndef __ivanHessianOnlyBasedVesselnessImageFunction_hxx
#define __ivanHessianOnlyBasedVesselnessImageFunction_hxx

#include "ivanHessianOnlyBasedVesselnessImageFunction.h"


namespace ivan
{

/** Set the Input Image */
template <class TInputImage, class TOutput, class TCoordRep>
HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::HessianOnlyBasedVesselnessImageFunction()
{
  
}


/** Print self method */
template <class TInputImage, class TOutput, class TCoordRep>
void
HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
}


template <class TInputImage, class TOutput, class TCoordRep>
typename HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateVesselness( const HessianTensorType & hessian, const PointType & point ) const
{
  return this->EvaluateVesselness( hessian );
}


template <class TInputImage, class TOutput, class TCoordRep>
typename HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateVesselnessAtIndex( const HessianTensorType & hessian, const IndexType & index ) const
{
  return this->EvaluateVesselness( hessian );
}


template <class TInputImage, class TOutput, class TCoordRep>
typename HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateVesselnessAtContinuousIndex( const HessianTensorType & hessian, 
  const ContinuousIndexType & index ) const
{
  return this->EvaluateVesselness( hessian );
}

} // end namespace ivan

#endif
