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
// File: ivanMultiscaleHessianBasedVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanFluxBasedVesselnessImageFunction_hxx
#define __ivanFluxBasedVesselnessImageFunction_hxx

#include "ivanFluxBasedVesselnessImageFunction.h"


namespace ivan
{
  
template <class TInputImage, class TOutput, class TCoordRep>
FluxBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::FluxBasedVesselnessImageFunction() :
  m_RadiusFactor( 1.7320508 ) // = sqrt(3.0)
{
  m_GradientFunction = GradientFunctionType::New();
  m_GradientFunction->NormalizeAcrossScaleOn();
  m_GradientFunction->UseImageSpacingOn();
  m_GradientFunction->SetInterpolationMode( GradientFunctionType::LinearInterpolation );
  m_GradientFunction->SetSigma( this->m_Sigma );
}


template <class TInputImage, class TOutput, class TCoordRep>
FluxBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::~FluxBasedVesselnessImageFunction()
{
  
}


/** Print self method */
template <class TInputImage, class TOutput, class TCoordRep>
void
FluxBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
  os << indent << "RadiusFactor: " << m_RadiusFactor << std::endl;
}


template <class TInputImage, class TOutput, class TCoordRep>
void
FluxBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::SetSigma( double sigma )
{ 
  if( this->m_Sigma == sigma )
    return;
    
  Superclass::SetSigma( sigma );
    
  this->m_GradientFunction->SetSigma( sigma );
}


/** Set the input image */
template <class TInputImage, class TOutput, class TCoordRep>
void
FluxBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::SetInputImage( const InputImageType * ptr )
{
  Superclass::SetInputImage( ptr );
  
  this->m_GradientFunction->SetInputImage( ptr ); 
}


template <class TInputImage, class TOutput, class TCoordRep>
void
FluxBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::Initialize()
{
  Superclass::Initialize();
  
  this->m_GradientFunction->Initialize();
}
  
} // end namespace ivan

#endif
