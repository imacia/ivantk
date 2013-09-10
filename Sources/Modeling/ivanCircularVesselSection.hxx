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
// File: ivanVesselSection.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanVesselSection_hxx
#define __ivanVesselSection_hxx

#include "ivanVesselSection.h"


namespace ivan
{

template <unsigned int VDimension, class TCurveMetrics, class TSectionMetrics>
CircularVesselSection<VDimension,TCurveMetrics,TSectionMetrics>::CircularVesselSection() :
  m_Radius(0.0),
  m_Scale(0.0),
  m_Arclength(0.0)
{

}


template <unsigned int VDimension, class TCurveMetrics, class TSectionMetrics>
CircularVesselSection<VDimension,TCurveMetrics,TSectionMetrics>::~CircularVesselSection()
{

}


template <unsigned int VDimension, class TCurveMetrics, class TSectionMetrics>
void CircularVesselSection<VDimension,TCurveMetrics,TSectionMetrics>::PrintSelf
  ( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Normal: " << m_Normal << std::endl;
  os << indent << "Radius: " << m_Radius << std::endl;
  os << indent << "Scale: " << m_Scale << std::endl;
  os << indent << "Arclength: " << m_Arclength << std::endl;
}

} // end namespace ivan

#endif // __ivanVesselSection_hxx
