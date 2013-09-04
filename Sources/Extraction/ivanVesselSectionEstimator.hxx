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
// File: ivanVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: generic node of an acyclic graph structure
// Date: 2010/05/28


#ifndef __ivanVesselSectionEstimator_hxx
#define __ivanVesselSectionEstimator_hxx

#include "ivanVesselSectionEstimator.h"


namespace ivan
{

template <class TCenterline, class TMetrics>
VesselSectionMetricsCalculator<TCenterline,TMetrics>
::VesselSectionMetricsCalculator()
{
  this->m_SectionRange.Fill(0);
}


template <class TCenterline, class TMetrics>
VesselSectionMetricsCalculator<TCenterline,TMetrics>
::~VesselSectionMetricsCalculator()
{
  this->m_SectionRange.Fill(0);
}

  
template <class TCenterline, class TMetrics>
void 
VesselSectionMetricsCalculator<TCenterline,TMetrics>
::SetSectionRange( RangeValueType first, RangeValueType last )
{
  // Do not perform checks. This is done in the VesselSectionEstimator
  this->m_SectionRange[0] = first;
  this->m_SectionRange[1] = last;  
}


template <class TCenterline, class TMetrics>
void 
VesselSectionMetricsCalculator<TCenterline,TMetrics>
::SetSection( RangeValueType section )
{
  // Do not perform checks. This is done in the VesselSectionEstimator
  this->_SectionRange[0] = this->m_SectionRange[1] = section;
}
  

template <class TCenterline, class TMetrics>
void
VesselSectionMetricsCalculator<TCenterline,TMetrics>
::SetSectionRangeToAll()
{
  if( this->m_Centerline.IsNull() )
  {
    itkWarningMacro( "Cannot set section range to all sections. Set centerline first." );
    return;
  }
  
  this->m_SectionRange[0] = 0;
  this->m_SectionRange[1] = this->m_Centerline->GetNumberOfSections()-1;
}


template <class TCenterline, class TMetrics>
void 
VesselSectionMetricsCalculator<TCenterline,TMetrics>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Centerline: " << this->m_Centerline.GetPointer() << std::endl;
  os << indent << "SectionRange: ( " << this->m_SectionRange[0] << ", " << this->m_SectionRange[1] << " )" << std::endl;
}  


template <class TCenterline, class TMetricsCalculator>
VesselSectionEstimator<TCenterline,TMetricsCalculator>
::VesselSectionEstimator() :
  m_CurrentScale( 0.0 )
{
  this->m_MeasurementCalculator = MetricsCalculatorType::New();
  this->m_SectionRange.Fill(0);
}


template <class TCenterline, class TMetricsCalculator>
VesselSectionEstimator<TCenterline,TMetricsCalculator>
::~VesselSectionEstimator()
{

}


template <class TCenterline, class TMetricsCalculator>
void 
VesselSectionEstimator<TCenterline,TMetricsCalculator>
::SetCenterline( CenterlineType * centerline )
{ 
  this->m_Centerline = centerline;
  this->m_MeasurementCalculator->SetCenterline( centerline );
  
  // Check coherency for section range
  if( this->m_Centerline.IsNotNull() )
    this->CheckSectionRangeIsValid();  
}


template <class TCenterline, class TMetricsCalculator>
void 
VesselSectionEstimator<TCenterline,TMetricsCalculator>
::SetSectionRange( RangeValueType first, RangeValueType last )
{
  this->m_SectionRange[0] = first;
  
  if( last > first )
  {
    itkWarningMacro( "Last section index should be larger than first. Setting last to first." );
    this->m_SectionRange[1] = this->m_SectionRange[0];
  }
  else
    this->m_SectionRange[1] = last;
  
  if( m_Centerline.IsNotNull() )
    this->CheckSectionRangeIsValid();
}


template <class TCenterline, class TMetricsCalculator>
void 
VesselSectionEstimator<TCenterline,TMetricsCalculator>
::SetSection( RangeValueType section )
{
  this->m_SectionRange[0] = this->m_SectionRange[1] = section;
    
  if( this->m_Centerline.IsNotNull() )
    this->CheckSectionRangeIsValid();
}
  

template <class TCenterline, class TMetricsCalculator>
void
VesselSectionEstimator<TCenterline,TMetricsCalculator>
::SetSectionRangeToAll()
{
  if( this->m_Centerline.IsNull() )
  {
    itkWarningMacro( "Cannot set section range to all sections. Set centerline first." );
    return;
  }
  
  this->m_SectionRange[0] = 0;
  this->m_SectionRange[1] = this->m_Centerline->GetNumberOfSections()-1;
  this->m_MeasurementCalculator->SetSectionRange( this->m_SectionRange[0], this->m_SectionRange[1] );
}


template <class TCenterline, class TMetricsCalculator>
void 
VesselSectionEstimator<TCenterline,TMetricsCalculator>
::CheckSectionRangeIsValid()
{
  assert( this->m_Centerline.IsNotNull() );
  
  if( !this->m_Centerline->GetNumberOfSections() )
  {
    this->m_SectionRange.Fill(0);
    this->m_MeasurementCalculator->SetSectionRange( 0, 0 );
    return;
  }
  
  if( this->m_SectionRange[0] > this->m_Centerline->GetNumberOfSections() )
  {
    this->m_SectionRange[0] = this->m_Centerline->GetNumberOfSections()-1;
    this->m_SectionRange[1] = this->m_Centerline->GetNumberOfSections()-1;
  }
  
  if( this->m_SectionRange[1] > this->m_Centerline->GetNumberOfSections() )
  {
    this->m_SectionRange[1] = this->m_Centerline->GetNumberOfSections()-1;
    
    if( this->m_SectionRange[0] > this->m_SectionRange[1] )
      this->m_SectionRange[0] = this->m_SectionRange[1];
  }
  
  this->m_MeasurementCalculator->SetSectionRange( this->m_SectionRange[0], this->m_SectionRange[1] );
}


template <class TCenterline, class TMetricsCalculator>
void 
VesselSectionEstimator<TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Centerline: " << this->m_Centerline.GetPointer() << std::endl;
  os << indent << "SectionRange: ( " << this->m_SectionRange[0] << ", " << this->m_SectionRange[1] << " )" << std::endl;
  os << indent << "CurrentScale: " << this->m_CurrentScale << std::endl;
}

} // end namespace ivan

#endif // __ivanVesselSectionEstimator_hxx
