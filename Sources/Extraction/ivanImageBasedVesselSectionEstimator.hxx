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
// File: ivanImageBasedVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: generic node of an acyclic graph structure
// Date: 2011/11/04


#ifndef __ivanImageBasedVesselSectionEstimator_hxx
#define __ivanImageBasedVesselSectionEstimator_hxx

#include "ivanImageBasedVesselSectionEstimator.h"


namespace ivan
{

template <class TImage, class TCenterline, class TMetricsCalculator>
ImageBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::ImageBasedVesselSectionEstimator()
{

}


template <class TImage, class TCenterline, class TMetricsCalculator>
ImageBasedVesselSectionEstimator<TImage,TCenterline, TMetricsCalculator>
::~ImageBasedVesselSectionEstimator()
{

}


template <class TImage, class TCenterline, class TMetricsCalculator>
void
ImageBasedVesselSectionEstimator<TImage,TCenterline, TMetricsCalculator> 
::SetImage( ImageType *image )
{
  if( m_Image.GetPointer() != image )
    this->m_Image = image;
}


template <class TImage, class TCenterline, class TMetricsCalculator>
void 
ImageBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Image: " << m_Image.GetPointer() << std::endl;  
}

} // end namespace ivan

#endif // __ivanImageBasedVesselSectionEstimator_hxx
