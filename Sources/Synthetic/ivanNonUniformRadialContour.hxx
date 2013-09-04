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
// File: ivanNonUniformRadialContour.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: generic node of an acyclic graph structure
// Date: 2009/02/06


#ifndef __ivanNonUniformRadialContour_hxx
#define __ivanNonUniformRadialContour_hxx

#include "ivanNonUniformRadialContour.h"

#include "itkNumericTraits.h"


namespace ivan
{

template <class TPoint, typename TAngle>
NonUniformRadialContour<TPoint,TAngle>::NonUniformRadialContour()
{

}


template <class TPoint,TAngle>
NonUniformRadialContour<TPoint>::~NonUniformRadialContour()
{

}


template <class TPoint,TAngle>
void NonUniformRadialContour<TPoint>::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "ContourId: " << m_ContourId << std::endl;
}

} // end namespace ivan

#endif // __ivanNonUniformRadialContour_hxx
