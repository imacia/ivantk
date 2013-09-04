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
// File: ivanVesselnessBasedVesselTrackerEndCondition.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/06/05

#ifndef __ivanVesselnessBasedVesselTrackerEndCondition_hxx
#define __ivanVesselnessBasedVesselTrackerEndCondition_hxx

#include "ivanVesselnessBasedVesselTrackerEndCondition.h"

#include "itkNumericTraits.h"


namespace ivan
{

template <class TVesselnessFunction, class TValueFunctor>
VesselnessBasedVesselTrackerEndCondition<TVesselnessFunction,TValueFunctor>
::VesselnessBasedVesselTrackerEndCondition()
{
  this->m_VesselnessFunction = VesselnessFunctionType::New();
  this->m_Position.Fill(0.0);
}


template <class TVesselnessFunction, class TValueFunctor>
bool 
VesselnessBasedVesselTrackerEndCondition<TVesselnessFunction,TValueFunctor>
::Finished()
{
  return ( this->m_ValueFunctor( this->m_VesselnessFunction->Evaluate( this->m_Position ) ) );
}
	

template <class TVesselnessFunction, class TValueFunctor>
void 
VesselnessBasedVesselTrackerEndCondition<TVesselnessFunction,TValueFunctor>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Position: " << this->m_Position << std::endl;
}

} // end namespace ivan

#endif // __ivanVesselnessBasedVesselTrackerEndCondition_hxx
