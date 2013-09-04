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
// File: ivanMultiscaleVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2012/02/03


#ifndef __ivanMultiscaleVesselSectionEstimator_hxx
#define __ivanMultiscaleVesselSectionEstimator_hxx


#include "ivanMultiscaleVesselSectionEstimator.h"

#include <vcl_cassert.h>



namespace ivan
{

template <class TImage, class TScaledImageFunction, class TCenterline, class TMetricsCalculator>
MultiscaleVesselSectionEstimator<TImage,TScaledImageFunction,TCenterline,TMetricsCalculator>
::MultiscaleVesselSectionEstimator() :
  m_OptimizeScale( false ),
  m_MinimumScale( 1.0 ),
  m_MaximumScale( 3.0 ),
  m_NumberOfScales( 5 ),
  m_PowerFactor( 1.5 ),
  m_ScaleStepMethod( LogarithmicScaleSteps )
{
  
}


template <class TImage, class TScaledImageFunction, class TCenterline, class TMetricsCalculator>
MultiscaleVesselSectionEstimator<TImage,TScaledImageFunction,TCenterline, TMetricsCalculator>
::~MultiscaleVesselSectionEstimator()
{

}


template <class TImage, class TScaledImageFunction, class TCenterline, class TMetricsCalculator>
void
MultiscaleVesselSectionEstimator<TImage,TScaledImageFunction,TCenterline, TMetricsCalculator>
::Initialize()
{
  if( this->GetNumberOfScales() == 0 )
  {
    itkWarningMacro( "NumberOfScales is zero. Setting to one." );
    this->m_NumberOfScales = 1;
  }
  
  this->m_ScaledImageFunctionContainer.clear();
  this->m_Scales.clear();
  
  double stepSize = 0.0, scaleValue = 0.0;
  
  if( this->GetNumberOfScales() > 1 )
  {
    switch( this->m_ScaleStepMethod )
    {
      case Self::EquispacedScaleSteps:
      {
        stepSize = vnl_math_max( 1e-10, ( this->m_MaximumScale - this->m_MinimumScale ) / ( this->m_NumberOfScales - 1 ) );
        
        for( unsigned int i=0; i < this->GetNumberOfScales(); ++i )
        {
          scaleValue = this->m_MinimumScale + stepSize * (double)i;
          
          ScaledImageFunctionPointer scaledFunction = ScaledImageFunctionType::New();
          this->InitializeScaledFunction( scaledFunction.GetPointer(), scaleValue );
                    
          this->m_ScaledImageFunctionContainer.push_back( scaledFunction );
          this->m_Scales.push_back( scaleValue );
        }
        
        break;
      }
      case Self::LogarithmicScaleSteps:
      {
        stepSize = vnl_math_max( 1e-10, ( vcl_log( this->m_MaximumScale ) - vcl_log( this->m_MinimumScale ) ) / 
          ( this->m_NumberOfScales - 1 ) );
          
        for( unsigned int i=0; i < this->GetNumberOfScales(); ++i )
        {
          scaleValue = vcl_exp( vcl_log ( this->m_MinimumScale ) + stepSize * (double)i );
          
          ScaledImageFunctionPointer scaledFunction = ScaledImageFunctionType::New();
          this->InitializeScaledFunction( scaledFunction.GetPointer(), scaleValue );
          
          this->m_ScaledImageFunctionContainer.push_back( scaledFunction );
          this->m_Scales.push_back( scaleValue );
        }
                 
        break;
      }
      case Self::PowerFactorScaleSteps:
      {
        double scaleValue = this->m_MinimumScale;
        
        while( scaleValue < this->m_MaximumScale )
        {
          ScaledImageFunctionPointer scaledFunction = ScaledImageFunctionType::New();
          this->InitializeScaledFunction( scaledFunction.GetPointer(), scaleValue );
          
          this->m_ScaledImageFunctionContainer.push_back( scaledFunction );
          this->m_Scales.push_back( scaleValue );
          
          scaleValue *= this->m_PowerFactor;          
        }
        
        break; 
      }
      default:
        itkExceptionMacro( "Invalid SigmaStepMethod." );
        break;
    }
  }
}


template <class TImage, class TScaledImageFunction, class TCenterline, class TMetricsCalculator>
void 
MultiscaleVesselSectionEstimator<TImage,TScaledImageFunction,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "OptimizeScale: " << this->m_OptimizeScale << std::endl;
  os << indent << "MinimumScale: " << this->m_MinimumScale << std::endl;
  os << indent << "MaximumScale: " << this->m_MaximumScale << std::endl;
  os << indent << "NumberOfScales: " << this->m_NumberOfScales << std::endl;
  os << indent << "PowerFactor: " << this->m_PowerFactor << std::endl;
  os << indent << "ScaleStepMethod: " << this->m_ScaleStepMethod << std::endl;
  os << indent << "ScaledImageFunctionContainer:" << std::endl;
  for( typename ScaledImageFunctionContainerType::const_iterator it = this->m_ScaledImageFunctionContainer.begin();
    it != this->m_ScaledImageFunctionContainer.end(); ++it )
    (*it)->Print( os, indent.GetNextIndent() );
  os << indent << "Scales: ";
  for( unsigned int i=0; i<this->m_Scales.size(); ++i )
    os << this->m_Scales[i] << " ";
  os << std::endl;
}

} // end namespace ivan

#endif // __ivanMultiscaleVesselSectionEstimator_hxx
