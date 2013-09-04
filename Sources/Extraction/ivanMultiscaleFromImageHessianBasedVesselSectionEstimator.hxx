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
// File: ivanMultiscaleFromImageHessianBasedVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/06/22


#ifndef __ivanMultiscaleFromImageHessianBasedVesselSectionEstimator_hxx
#define __ivanMultiscaleFromImageHessianBasedVesselSectionEstimator_hxx

#include "itkMultiscaleFromImageHessianBasedVesselSectionEstimator.h"


namespace ivan
{

namespace Vessel
{

template <class TImage, class TScaleImage, class TCenterline, class TMetricsCalculator>
MultiscaleFromImageHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::MultiscaleFromImageHessianBasedVesselSectionEstimator()
{
  m_ScaleInterpolator = ScaleImageInterpolatorType::New();
}


template <class TImage, class TScaleImage, class TCenterline, class TMetricsCalculator>
MultiscaleFromImageHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::~MultiscaleFromImageHessianBasedVesselSectionEstimator()
{

}


template <class TImage, class TScaleImage, class TCenterline, class TMetricsCalculator>
void
MultiscaleFromImageHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::SetScaleImage( const ScaleImageType *scaleImage )
{
  if( m_ScaleImage.GetPointer() != scaleImage )
  {
    m_ScaleImage = scaleImage;
    m_ScaleInterpolator->SetInputImage( m_ScaleImage );
  }
}


template <class TImage, class TScaleImage, class TCenterline, class TMetricsCalculator>
double
MultiscaleFromImageHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::GetScaleAt( unsigned int centerlineIdx, const PointType & point )
{
  vcl_assert( m_ScaleImage.IsNotNull() );
  
  if( m_ScaleInterpolator->IsInsideBuffer( point ) )
    return m_ScaleInterpolator->Evaluate( point );
  else
    return 0.0;
}


template <class TImage, class TScaleImage, class TCenterline, class TMetricsCalculator>
void 
MultiscaleFromImageHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
 
  os << indent << "ScaleImage: " << m_ScaleImage.GetPointer() << std::endl;
}

} // end namespace Vessel

} // end namespace ivan

#endif // __ivanMultiscaleFromImageHessianBasedVesselSectionEstimator_hxx
