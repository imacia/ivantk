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
// File: ivanMultiscaleHessianBasedVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2012/02/03



#ifndef __ivanMultiscaleHessianBasedVesselSectionEstimator_hxx
#define __ivanMultiscaleHessianBasedVesselSectionEstimator_hxx

#include "ivanMultiscaleHessianBasedVesselSectionEstimator.h"

#include "itkLinearInterpolateImageFunction.h"

#include <vcl_cassert.h>

#define SCALE_TOLERANCE 1e-2
#define HESSIAN_OPERATOR_MAX_ERROR 0.025


namespace ivan
{

template <class TImage, class TCenterline, class TMetricsCalculator>
MultiscaleHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::MultiscaleHessianBasedVesselSectionEstimator()
{
  
}


template <class TImage, class TCenterline, class TMetricsCalculator>
MultiscaleHessianBasedVesselSectionEstimator<TImage,TCenterline, TMetricsCalculator>
::~MultiscaleHessianBasedVesselSectionEstimator()
{

}


template <class TImage, class TCenterline, class TMetricsCalculator>
void 
MultiscaleHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::InitializeScaledFunction( ScaledImageFunctionType *scaledImageFunction, double scale )
{
  scaledImageFunction->SetMaximumError( HESSIAN_OPERATOR_MAX_ERROR );
  scaledImageFunction->SetSigma( scale );
  scaledImageFunction->SetInputImage( this->m_Image );
  scaledImageFunction->SetNormalizeAcrossScale( true );
  scaledImageFunction->SetUseImageSpacing( true );
  scaledImageFunction->Initialize();
}


template <class TImage, class TCenterline, class TMetricsCalculator>
void 
MultiscaleHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  

}

} // end namespace ivan

#endif // __ivanMultiscaleHessianBasedVesselSectionEstimator_hxx
