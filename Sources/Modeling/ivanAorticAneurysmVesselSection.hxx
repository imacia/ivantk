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
// File: ivanAorticAneurysmVesselSection.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanAorticAneurysmVesselSection_hxx
#define __ivanAorticAneurysmVesselSection_hxx

#include "ivanAorticAneurysmVesselSection.h"


namespace ivan
{

template <unsigned int VDimension>
AorticAneurysmVesselSection<VDimension>::AorticAneurysmVesselSection() :
  m_MaxAneurysmDiameter(0.0)
{
  m_LumenContour = RadialContourType::New();
  m_ThrombusContour = RadialContourType::New();
}


template <unsigned int VDimension>
AorticAneurysmVesselSection<VDimension>::~AorticAneurysmVesselSection()
{

}


template <unsigned int VDimension>
void AorticAneurysmVesselSection<VDimension>::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Center2: " << m_Center2 << std::endl;
  os << indent << "AneurysmCenter: " << m_AneurysmCenter << std::endl;
  os << indent << "MaxAneurysmDiameter: " << m_MaxAneurysmDiameter << std::endl;

  m_LumenContour->Print( os, indent.GetNextIndent() );
  m_ThrombusContour->Print( os, indent.GetNextIndent() );
}

} // end namespace ivan

#endif // __ivanAorticAneurysmVesselSection_hxx
