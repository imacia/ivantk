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
// File: ivanMultiscaleFromRadiusHessianBasedVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/06/22


#ifndef __ivanMultiscaleFromRadiusHessianBasedVesselSectionEstimator_hxx
#define __ivanMultiscaleFromRadiusHessianBasedVesselSectionEstimator_hxx

#include "itkMultiscaleFromRadiusHessianBasedVesselSectionEstimator.h"


namespace ivan
{

namespace Vessel
{

template <class TImage, class TScaleImage, class TRadiusFunctor, class TCenterline, class TMetricsCalculator>
MultiscaleFromRadiusHessianBasedVesselSectionEstimator<TImage,TScaleImage,TRadiusFunctor,TCenterline,TMetricsCalculator>
::MultiscaleFromRadiusHessianBasedVesselSectionEstimator()
{
  
}


template <class TImage, class TScaleImage, class TRadiusFunctor, class TCenterline, class TMetricsCalculator>
MultiscaleFromRadiusHessianBasedVesselSectionEstimator<TImage,TScaleImage,TRadiusFunctor,TCenterline,TMetricsCalculator>
::~MultiscaleFromRadiusHessianBasedVesselSectionEstimator()
{

}


template <class TImage, class TScaleImage, class TRadiusFunctor, class TCenterline, class TMetricsCalculator>
double
MultiscaleFromRadiusHessianBasedVesselSectionEstimator<TImage,TScaleImage,TRadiusFunctor,TCenterline,TMetricsCalculator>
::GetScaleAt( unsigned int centerlineIdx, const PointType & point )
{
  SectionTypePointer currentSection = this->GetCenterline()->at( centerlineIdx );
  return this->m_ScaleFunctor( currentSection->GetRadius() ); // use the overloaded () operator
}



template <class TImage, class class TScaleImage, class TRadiusFunctor, class TCenterline, class TMetricsCalculator>
void 
MultiscaleFromRadiusHessianBasedVesselSectionEstimator<TImage,TScaleImage,TRadiusFunctor,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
 
}

} // end namespace Vessel

} // end namespace ivan

#endif // __ivanMultiscaleFromRadiusHessianBasedVesselSectionEstimator_hxx
