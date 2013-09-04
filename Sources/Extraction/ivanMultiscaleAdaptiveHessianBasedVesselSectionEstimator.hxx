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
// File: ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/06/22


#ifndef __ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimator_hxx
#define __ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimator_hxx

#include "ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimator.h"

#include <algorithm>

namespace ivan
{

template <class TImage, class TMetricFunction, class TCenterline, class TMetricsCalculator>
MultiscaleAdaptiveHessianBasedVesselSectionEstimator<TImage,TMetricFunction,TCenterline,TMetricsCalculator>
::MultiscaleAdaptiveHessianBasedVesselSectionEstimator()
{
  this->m_MetricFunctionInitializer = MetricFunctionInitializerType::New(); // provide at least default implementation
}


template <class TImage, class TMetricFunction, class TCenterline, class TMetricsCalculator>
MultiscaleAdaptiveHessianBasedVesselSectionEstimator<TImage,TMetricFunction,TCenterline,TMetricsCalculator>
::~MultiscaleAdaptiveHessianBasedVesselSectionEstimator()
{

}


template <class TImage, class TMetricFunction, class TCenterline, class TMetricsCalculator>
void
MultiscaleAdaptiveHessianBasedVesselSectionEstimator<TImage,TMetricFunction,TCenterline,TMetricsCalculator>
::Initialize()
{
  Superclass::Initialize();
  
  MetricFunctionPointer metricFunction;
  
  for( unsigned int i=0; i<this->m_Scales.size(); ++i )
  {
    metricFunction = MetricFunctionType::New();
    this->m_MetricFunctionInitializer->Initialize( metricFunction.GetPointer(), this->m_Image, this->m_Scales[i] );
    this->m_MetricFunctionContainer.push_back( metricFunction );   
  }  
}


template <class TImage, class TMetricFunction, class TCenterline, class TMetricsCalculator>
double
MultiscaleAdaptiveHessianBasedVesselSectionEstimator<TImage,TMetricFunction,TCenterline,TMetricsCalculator>
::GetScaleAt( unsigned int centerlineIdx, const PointType & point )
{
  assert( this->m_Scales.size() );
  
  SectionPointer currentSection = this->GetCenterline()->at( centerlineIdx );
  SectionPointer previousSection;
  
  double currentScale;
  int idx = 0;
  
  if( centerlineIdx > 0 )
  {
    previousSection = this->GetCenterline()->at( centerlineIdx-1 );
    currentScale = previousSection->GetScale();
    
    // Get the closest scale from our group of scales
    if( previousSection->GetScale() < this->m_Scales[0] )
      currentScale = *( this->m_Scales.begin() );
    else
    {
      double scale = previousSection->GetScale(); 
      // imacia: I dont know why but need to create the variable or VS2008 compiler complains on next line
      ScaleVectorType::iterator it = std::lower_bound
        ( this->m_Scales.begin(), this->m_Scales.end(), scale );
      if( it == this->m_Scales.begin() )
        currentScale = *( this->m_Scales.begin() );
      else
        currentScale = *( --it );
        
      idx = (unsigned int)( it - this->m_Scales.begin() ); // CHECK THIS
      assert( idx >= 0 && idx < this->m_Scales.size() );
    }
  }
  else
    currentScale = *( this->m_Scales.begin() ); // always take the smallest if possible since it is faster to calculate

  double currentValue, nextValue, previousValue;
  bool maxFound = false;

  if( this->m_MetricFunctionContainer.size() <= 1 )
    return this->m_Scales[0];

  currentValue = this->m_MetricFunctionContainer[idx]->Evaluate( point );

  nextValue = this->m_MetricFunctionContainer[idx+1]->Evaluate( point );

  if( !idx )
    previousValue = itk::NumericTraits<double>::NonpositiveMin();
  else
    previousValue = this->m_MetricFunctionContainer[idx-1]->Evaluate( point );
  
  // WARNING: we are assuming that there is a single maximum accross scales!!!

  while( !maxFound )
  {
    if( currentValue >= previousValue && currentValue >= nextValue && currentValue != 0.0 )
      maxFound = true;
    else if( currentValue < previousValue )
    {
      assert( idx != 0 );

      nextValue = currentValue;
      currentValue = previousValue;
      --idx;
      
      if( idx > 0 )
        previousValue = this->m_MetricFunctionContainer[idx-1]->Evaluate( point );
      else
        previousValue = itk::NumericTraits<double>::NonpositiveMin();
    }
    else if( currentValue <= nextValue )
    {
      previousValue = currentValue;
      currentValue = nextValue; 
      ++idx;

      if( idx+1 > this->m_Scales.size()-1 )
        nextValue = itk::NumericTraits<double>::NonpositiveMin(); // set again a minimum value
      else
        nextValue = this->m_MetricFunctionContainer[idx+1]->Evaluate( point );
    }
    else if( currentValue >= previousValue && currentValue >= nextValue && currentValue == 0.0 )
    {
      // This might happen for example when all the values are 0.0, such as in out of bounds condition
      itkWarningMacro( "The medialness at the current position is zero." );
      return this->m_Scales[idx];
    }
    else 
    {
      assert(0); // OOOOOPS
    }

    if( idx == this->m_MetricFunctionContainer.size() )
      return this->m_Scales[idx-1];
  }
    
  return this->m_Scales[idx];
}


template <class TImage, class TMetricFunction, class TCenterline, class TMetricsCalculator>
void 
MultiscaleAdaptiveHessianBasedVesselSectionEstimator<TImage,TMetricFunction,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
 
  os << indent << "MetricFunction: " << std::endl;
  for( unsigned int i=0; i<this->m_MetricFunctionContainer.size(); ++i )
  {
    os << indent << "[" << i << "]" << std::endl;
    this->m_MetricFunctionContainer[i]->Print( os, indent.GetNextIndent() );
  }
}

} // end namespace ivan

#endif // __ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimator_hxx
