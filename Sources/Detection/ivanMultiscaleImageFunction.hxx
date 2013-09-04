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
// File: MultiscaleImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanMultiscaleImageFunction_hxx
#define __ivanMultiscaleImageFunction_hxx


#include "ivanMultiscaleImageFunction.h"


namespace ivan
{

/** Set the Input Image */
template <class TScaledImageFunction, class TInputImage, class TOutput, class TCoordRep>
MultiscaleImageFunction<TScaledImageFunction,TInputImage,TOutput,TCoordRep>
::MultiscaleImageFunction() :
  m_MinimumScale( 1.0),
  m_MaximumScale( 5.0 ),
  m_NumberOfScales( 5 ),
  m_ScaleStepMethod( LogarithmicScaleSteps )
{
  this->m_ScaledImageFunctionInitializer = ScaledImageFunctionInitializerType::New(); // provide at least default implementation
}



template <class TScaledImageFunction, class TInputImage, class TOutput, class TCoordRep>
void
MultiscaleImageFunction<TScaledImageFunction,TInputImage,TOutput,TCoordRep>
::SetInputImage( const InputImageType *inputImage )
{ 
  this->m_InputImage = inputImage;
}


template <class TScaledImageFunction, class TInputImage, class TOutput, class TCoordRep>
void
MultiscaleImageFunction<TScaledImageFunction,TInputImage,TOutput,TCoordRep>
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
            
          this->m_ScaledImageFunctionInitializer->Initialize
            ( scaledFunction.GetPointer(), this->m_InputImage, scaleValue );                    
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
            
          this->m_ScaledImageFunctionInitializer->Initialize
            ( scaledFunction.GetPointer(), this->m_InputImage, scaleValue );                    
          this->m_ScaledImageFunctionContainer.push_back( scaledFunction );
          this->m_Scales.push_back( scaleValue );
        }
                 
        break;
      }
      default:
        itkExceptionMacro( "Invalid SigmaStepMethod." );
        break;
    }
  }
}


/** Evaluate the function at the specifed index */
template <class TScaledImageFunction, class TInputImage, class TOutput, class TCoordRep>
typename MultiscaleImageFunction<TScaledImageFunction,TInputImage,TOutput,TCoordRep>::OutputType
MultiscaleImageFunction<TScaledImageFunction,TInputImage,TOutput,TCoordRep>
::EvaluateAtIndex( const IndexType& index ) const
{
  // For now we simply select the maximum

  OutputType value = itk::NumericTraits<OutputType>::min();
  OutputType tempValue;

  for( unsigned int i=0; i < this->m_ScaledImageFunctionContainer.size(); ++i )
  {
    tempValue = this->m_ScaledImageFunctionContainer[i]->EvaluateAtIndex( index );
  
    if( tempValue > value )
      value = tempValue;
  }

  return value;
}


/** Evaluate the function at the specifed point */
template <class TScaledImageFunction, class TInputImage, class TOutput, class TCoordRep>
typename MultiscaleImageFunction<TScaledImageFunction,TInputImage,TOutput,TCoordRep>::OutputType
MultiscaleImageFunction<TScaledImageFunction,TInputImage,TOutput,TCoordRep>
::Evaluate( const PointType& point ) const
{
  // For now we simply select the maximum

  OutputType value = itk::NumericTraits<OutputType>::min();
  OutputType tempValue;

  for( unsigned int i=0; i < this->m_ScaledImageFunctionContainer.size(); ++i )
  {
    tempValue = this->m_ScaledImageFunctionContainer[i]->Evaluate( point );
  
    if( tempValue > value )
      value = tempValue;
  }

  return value;
}


/** Evaluate the function at specified ContinousIndex position.*/
template <class TScaledImageFunction, class TInputImage, class TOutput, class TCoordRep>
typename MultiscaleImageFunction<TScaledImageFunction,TInputImage,TOutput,TCoordRep>::OutputType
MultiscaleImageFunction<TScaledImageFunction,TInputImage,TOutput,TCoordRep>
::EvaluateAtContinuousIndex( const ContinuousIndexType & cindex ) const
{
  // For now we simply select the maximum

  OutputType value = itk::NumericTraits<OutputType>::min();
  OutputType tempValue;

  for( unsigned int i=0; i < this->m_ScaledImageFunctionContainer.size(); ++i )
  {
    tempValue = this->m_ScaledImageFunctionContainer[i]->EvaluateAtContinuousIndex( cindex );
  
    if( tempValue > value )
      value = tempValue;
  }

  return value;
}


/** Print self method */
template <class TScaledImageFunction, class TInputImage, class TOutput, class TCoordRep>
void
MultiscaleImageFunction<TScaledImageFunction,TInputImage,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
  os << indent << "InputImage: " << this->m_InputImage.GetPointer() << std::endl;
  os << indent << "MinimumScale: " << this->m_MinimumScale << std::endl;
  os << indent << "MaximumScale: " << this->m_MaximumScale << std::endl;
  os << indent << "NumberOfScales: " << this->m_NumberOfScales << std::endl;
  os << indent << "ScaleStepMethod: " << this->m_ScaleStepMethod << std::endl;
  os << indent << "ScaledImageFunctionContainer:" << std::endl;
  
  os << indent << "Scales: ";
  for( unsigned int i=0; i < this->m_Scales.size(); ++i )
    os << this->m_Scales[i] << " ";
    
  for( typename ScaledImageFunctionContainerType::const_iterator it = m_ScaledImageFunctionContainer.begin();
    it != m_ScaledImageFunctionContainer.end(); ++it )
    (*it)->Print( os, indent.GetNextIndent() );
  
  os << std::endl;    
}

} // end namespace ivan

#endif
