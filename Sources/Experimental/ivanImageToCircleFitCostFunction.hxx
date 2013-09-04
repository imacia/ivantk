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
// File: ivanImageToCircleFitCostFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: cost function to fit a circle to a 2D image

#ifndef __ivanImageToCircleFitCostFunction_hxx
#define __ivanImageToCircleFitCostFunction_hxx

#include "vnl/vnl_cross.h"

namespace ivan
{



template<class TImage>
ImageToCircleFitCostFunction<TImage>
::ImageToCircleFitCostFunction() :
	m_SymmetryCoefficient(1.0)
{
	m_Center.Fill(0.0);  
}



template<class TImage>
ImageToCircleFitCostFunction<TImage>
::~ImageToCircleFitCostFunction()
{

}



template <class TImage> 
void
ImageToCircleFitCostFunction<TImage>
::Initialize(void) throw ( ExceptionObject )
{

  if( !m_Interpolator )
    {
    itkExceptionMacro(<<"Interpolator is not present");
    }

  if( !m_Image )
    {
    itkExceptionMacro(<<"Image is not present");
    }

  // If no region is assigned set default to requested region  
  
  const typename RegionType::SizeType &size = m_Region.GetSize();
  
  for( unsigned int i=0; i<ImageDimension; ++i )
  {
  	if( size[i] == 0 )
  	{
  		m_Region = m_Image->GetRequestedRegion();
  		break;  		
  	}
  }    
        
  // If the image is provided by a source, update the source.
  if( m_Image->GetSource() )
    {
    m_Image->GetSource()->Update();
    }

  m_Interpolator->SetInputImage( m_Image );
 
 
}





template<class TImage>
unsigned int
ImageToCircleFitCostFunction<TImage>
::GetNumberOfParameters(void) const
{
  return 1;
}




template<class TImage>
typename ImageToCircleFitCostFunction<TImage>::MeasureType
ImageToCircleFitCostFunction<TImage>
::GetValue( const ParametersType & parameters ) const
{
  if ( m_Image.IsNull() )
    {
    itkExceptionMacro(<<"Image is null");
    }

  MeasureType value = 0.0;
  double radius = parameters[0];


	// Do not allow the radius to be bigger than the image region or to be negative
	
	if( radius <= 0.0 )
		return itk::NumericTraits<MeasureType>::Zero;
			
	// Do not allow the radius to be larger than half the minimum size in all directions

	typename RegionType::SizeType::SizeValueType minSize = 
		itk::NumericTraits<typename RegionType::SizeType::SizeValueType>::max();
	
	typename RegionType::SizeType size = m_Region.GetSize();

  for( unsigned int i=0; i<ImageDimension; ++i )
	{
		if( size[i] < minSize )
			minSize = size[i];
	}

	if( radius >= minSize / 2.0 )
		return itk::NumericTraits<MeasureType>::Zero;

  // Do not allow radius to be smaller than half the minimum spacing in all directions
  
	const typename TImage::SpacingType &spacing = m_Image->GetSpacing();

	double maxSpacing = 0.0;

	for( unsigned int i=0; i<ImageDimension; ++i )
	{
		if( spacing[i] > maxSpacing )
			maxSpacing = spacing[i];
	}

	if( radius <= maxSpacing / 2.0 )
		return itk::NumericTraits<MeasureType>::Zero;
	
	
  // Set the number of samples in a circle depending on the scale
	double tempSamples = 2.0f * vnl_math::pi * radius + 1.0f;
	unsigned int samples = static_cast<unsigned int>( std::floor( tempSamples + 0.5 ) );

	if( samples < 4 )
		samples = 4; // take at least four samples
	
	// Set the angle increment 
	double angleInc = vnl_math::pi * 2.0f / static_cast<double>( samples ); 
	
	double angle = 0.0, sinAngle, cosAngle; // current angle and its sin and cos
	PointType circlePoint;
	
	itk::Array<double>		firstBaseVector; 
	itk::Array<double>	 	secondBaseVector;
	firstBaseVector.SetSize(ImageDimension);
	secondBaseVector.SetSize(ImageDimension);
	
	// !!! Specific for 2D
	firstBaseVector.Fill(0.0);
	firstBaseVector[0] = 1.0;
	secondBaseVector.Fill(0.0);
	secondBaseVector[1] = 1.0;
	
	// Calculate points in a circle with the given radius
	for ( unsigned int i=0; i<samples; ++i )
	{
		cosAngle = cos( angle );
		sinAngle = sin( angle );
	  
		// Evaluate and store the value of the gradient in a circle (rotate 90� and calculate 4 points each time)
		
		for( int k=0; k<ImageDimension; ++k )
			circlePoint[k] = m_Center[k] + radius * ( cosAngle * firstBaseVector[k] + sinAngle * secondBaseVector[k] );

		if( m_Interpolator->IsInsideBuffer( circlePoint ) )
			value += m_Interpolator->Evaluate( circlePoint );
		  			
		angle += angleInc;
		
	}
	
	value /= samples;

	std::cout << "Radius: " << radius << "  Value: " << value << std::endl;
  
  return value;
  
}



template <class TImage> 
void
ImageToCircleFitCostFunction<TImage>
::GetDerivative( const ParametersType & parameters, DerivativeType & derivative ) const
{
	if ( m_Image.IsNull() )
    {
    itkExceptionMacro(<<"Image is null");
    }
    
  const unsigned int ParametersDimension = this->GetNumberOfParameters();
  derivative = DerivativeType( ParametersDimension );
  derivative.Fill( NumericTraits<ITK_TYPENAME DerivativeType::ValueType>::Zero );
    
  double radius = parameters[0];
 
 
  // Set the number of samples in a circle depending on the scale
	double tempSamples = 2.0f * vnl_math::pi * radius + 1.0f;
	unsigned int samples = static_cast<unsigned int>( std::floor( tempSamples + 0.5 ) );
	
	if( samples < 4 )
		samples = 4; // take at least four samples
	
	// Set the angle increment 
	double angleInc = vnl_math::pi * 2.0f / static_cast<double>( samples ); 
	
	MeasureType tempDerivative;
	double angle = 0.0, delta = 0.5, sinAngle, cosAngle; // current angle and its sin and cos
	PointType circlePointBack, circlePointFord;
	
	itk::Array<double>		firstBaseVector; 
	itk::Array<double>	 	secondBaseVector;
	firstBaseVector.SetSize(ImageDimension);
	secondBaseVector.SetSize(ImageDimension);
	
	// !!! Specific for 2D
	firstBaseVector.Fill(0.0);
	firstBaseVector[0] = 1.0;
	secondBaseVector.Fill(0.0);
	secondBaseVector[1] = 1.0;
	
	// Calculate points in a circle with the given radius
	for ( unsigned int i=0; i<samples; ++i )
	{
		cosAngle = cos( angle );
		sinAngle = sin( angle );
	  
		// Evaluate and store the value of the gradient in a circle (rotate 90� and calculate 4 points each time)
		
		for( int k=0; k<ImageDimension; ++k )
		{
			circlePointBack[k] = m_Center[k] + radius * ( 1.0 - delta ) * 
				( cosAngle * firstBaseVector[k] + sinAngle * secondBaseVector[k] );
			circlePointFord[k] = m_Center[k] + radius * ( 1.0 + delta ) * 
				( cosAngle * firstBaseVector[k] + sinAngle * secondBaseVector[k] );
			
		}

		if( m_Interpolator->IsInsideBuffer( circlePointBack ) && m_Interpolator->IsInsideBuffer( circlePointFord ) )
		{
			tempDerivative = 0.0;
			tempDerivative += m_Interpolator->Evaluate( circlePointFord );
			tempDerivative -= m_Interpolator->Evaluate( circlePointBack );
			tempDerivative /= 2.0 * delta;
			derivative[0] += tempDerivative;
		}
		  			
		angle += angleInc;		
	}
	
	derivative[0] /= samples;

  std::cout << "Derivative:  " << derivative[0] << std::endl;  
}



template <class TImage> 
void
ImageToCircleFitCostFunction<TImage>
::GetValueAndDerivative( const ParametersType & parameters, 
  MeasureType & value, DerivativeType  & derivative ) const
{
  value = this->GetValue( parameters );
  this->GetDerivative( parameters, derivative );
}



} // end namespace ivan

#endif
