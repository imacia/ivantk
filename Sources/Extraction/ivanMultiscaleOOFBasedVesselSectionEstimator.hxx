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
// File: ivanMultiscaleOOFBasedVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2012/02/07



#ifndef __ivanMultiscaleOOFBasedVesselSectionEstimator_hxx
#define __ivanMultiscaleOOFBasedVesselSectionEstimator_hxx


#include "ivanMultiscaleOOFBasedVesselSectionEstimator.h"
#include "ivanDiscreteGradientGaussianImageFunction.h"

#define DEFAULT_OOF_GRADIENT_SIGMA 1.0


namespace ivan
{

template <class TImage, class TVectorField, class TCenterline, class TMetricsCalculator>
MultiscaleOOFBasedVesselSectionEstimator<TImage,TVectorField,TCenterline,TMetricsCalculator>
::MultiscaleOOFBasedVesselSectionEstimator()
{
  
}


template <class TImage, class TVectorField, class TCenterline, class TMetricsCalculator>
MultiscaleOOFBasedVesselSectionEstimator<TImage,TVectorField,TCenterline, TMetricsCalculator>
::~MultiscaleOOFBasedVesselSectionEstimator()
{

}


template <class TImage, class TVectorField, class TCenterline, class TMetricsCalculator>
void 
MultiscaleOOFBasedVesselSectionEstimator<TImage,TVectorField,TCenterline,TMetricsCalculator>
::InitializeScaledFunction( ScaledImageFunctionType *scaledImageFunction, double scale )
{
  // For now we will provide this as default, which assumes a DiscreteGradientImageFunction or similar
  // We should provide a functor for this task too
  
  VectorFieldPointer vectorField = scaledImageFunction->GetVectorField();
  vectorField->SetInputImage( this->m_Image );
  vectorField->SetSigma( DEFAULT_OOF_GRADIENT_SIGMA );
  vectorField->NormalizeAcrossScaleOn();
  vectorField->UseImageSpacingOn();
  vectorField->Initialize();
  
  scaledImageFunction->SetRadius( scale );
  scaledImageFunction->SetInputImage( this->m_Image );
  scaledImageFunction->Initialize(); // this computes the sphere grid
}


template <class TImage, class TVectorField, class TCenterline, class TMetricsCalculator>
void 
MultiscaleOOFBasedVesselSectionEstimator<TImage,TVectorField,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  

}

} // end namespace ivan

#endif // __ivanMultiscaleOOFBasedVesselSectionEstimator_hxx
