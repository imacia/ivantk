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
// File: ivanLocalCurveMetricsCalculator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06

#ifndef __ivanLocalCurveMetricsCalculator_hxx
#define __ivanLocalCurveMetricsCalculator_hxx

#include "ivanLocalCurveMetricsCalculator.h"


namespace ivan
{

template <class TSection>
LocalCurveMetricsCalculator<TSection>::LocalCurveMetricsCalculator()
{

}


template <class TSection>
LocalCurveMetricsCalculator<TSection>::~LocalCurveMetricsCalculator()
{

}


template <class TSection>
void LocalCurveMetricsCalculator<TSection>::operator()
{
  if( this->m_Centerline->IsNull() )
  {
    itkWarningMacro( "Cannot compute metrics. Please, set centerline first." );
    return; 
  }
  
  CurveMetricsType & metrics = 
}


template <class TSection>
void LocalCurveMetricsCalculator<TSection>::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );

}

} // end namespace ivan

#endif // __ivanLocalCurveMetricsCalculator_hxx
