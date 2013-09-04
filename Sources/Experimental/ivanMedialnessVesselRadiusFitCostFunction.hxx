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
// File: ivanMedialnessVesselRadiusFitCostFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: cost function to fit a circle to a 2D image

#ifndef __ivanMedialnessVesselRadiusFitCostFunction_hxx
#define __ivanMedialnessVesselRadiusFitCostFunction_hxx

#include "vnl/vnl_cross.h"


namespace ivan
{



template<class TImage,class TScale>
MedialnessVesselRadiusFitCostFunction<TImage,TScale>
::MedialnessVesselRadiusFitCostFunction() :
  m_SamplingDistance(0.0),
  m_SymmetryCoefficient(1.0),
  m_CurrentScale(0.0),
  m_ScaleTolerance(0.5)
{
  m_Center.Fill(0.0);
  
  m_Gradient = GradientFunctionType::New();
  m_Gradient->SetInterpolationMode( GradientFunctionType::LinearInterpolation );
  m_Gradient->SetNormalizeAcrossScale( true );
  m_Gradient->SetUseImageSpacing( true );
  
  m_FirstBaseVector.SetSize(ImageDimension);
	m_SecondBaseVector.SetSize(ImageDimension);
		
	m_FirstBaseVector.Fill(0.0);
	m_FirstBaseVector[0] = 1.0;
	m_SecondBaseVector.Fill(0.0);
	m_SecondBaseVector[1] = 1.0;
}



template<class TImage,class TScale>
MedialnessVesselRadiusFitCostFunction<TImage,TScale>
::~MedialnessVesselRadiusFitCostFunction()
{

}


template<class TImage,class TScale>
void
MedialnessVesselRadiusFitCostFunction<TImage,TScale>
::SetImage( const ImageType * image )
{
	m_Image = image;
	m_Gradient->SetInputImage( image );
}


template<class TImage,class TScale>
void
MedialnessVesselRadiusFitCostFunction<TImage,TScale>
::SetNormal( const NormalVectorType & normal )
{
  itk::Array<double>  normal2; 
  normal2.SetSize(ImageDimension);
  normal2[0] = normal[0];
  normal2[1] = normal[1];
  normal2[2] = normal[2];
  normal2.normalize();
	
	// Calculate an orthonormal base of vectors in the plane of the section.
  // The plane equation can be expressed as :
  // a*(x-x0) + b*(y-y0) + c(z-z0) = 0 where (a,b,c) is the plane normal and (x0,y0,z0) is a point in the 
  // plane, for example the center. We are interested in calculating a unit vector ( (x-x0),(y-y0),(z-z0) ).
  // From the equations :
  // 1) a*(x-x0) + b*(y-y0) + c(z-z0) = 0
  // 2) (x-x0)^2 + (y-y0)^2 + (z-z0)^2 = 1 (unit vector)
  // 3) a^2 + b^2 + c^2 = 1 (normal is unit vector)
  // we get
  // (z-z0) = ( -2bc(y-y0) +- sqrt( 4 * (b^2+c^2-1) * ((y-y0)^2+b^2-1) ) ) / 2(1-b^2)
  // We arbitrarily choose y-y0 = 1-b^2 to make the discriminant zero resulting (z-z0) = -b*c / (y-y0)
  // and (x-x0) = ( -b(y-y0) -c(z-z0) ) / a 
    
  m_FirstBaseVector[1] = sqrt( 1.0 - normal[1]*normal[1] );
  m_FirstBaseVector[2] = ( - normal[1]*normal[2] ) / m_FirstBaseVector[1];
  m_FirstBaseVector[0] = ( - normal[1]*m_FirstBaseVector[1] - normal[2]*m_FirstBaseVector[2] ) / normal[0];
  
  m_SecondBaseVector = vnl_cross_3d( m_FirstBaseVector, normal2 );
}


template <class TImage,class TScale> 
void
MedialnessVesselRadiusFitCostFunction<TImage,TScale>
::Initialize(void) throw ( ExceptionObject )
{
  if( !m_Image )
    {
    itkExceptionMacro(<<"Image is not present");
    }
    
  // If no region is assigned set default to requested region  

  const SizeType& size = m_Region.GetSize();

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
    
  // If the sampling distance is not set initialize it to the minimum image spacing
  if( !m_SamplingDistance )
  {
    const SpacingType& spacing = m_Image->GetSpacing();
    m_SamplingDistance = itk::NumericTraits<double>::max();
  
    for( unsigned int i=0; i<ImageDimension; ++i )
    {
      if( spacing[i] < m_SamplingDistance )
        m_SamplingDistance = spacing[i];
    }
  }
}


template<class TImage,class TScale>
unsigned int
MedialnessVesselRadiusFitCostFunction<TImage,TScale>
::GetNumberOfParameters(void) const
{
  return ImageDimension+1;
}


template<class TImage,class TScale>
typename MedialnessVesselRadiusFitCostFunction<TImage,TScale>::MeasureType
MedialnessVesselRadiusFitCostFunction<TImage,TScale>
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

	RegionType::SizeType::SizeValueType minSize = 
		itk::NumericTraits<RegionType::SizeType::SizeValueType>::max();
	
	RegionType::SizeType size = m_Region.GetSize();

  for( unsigned int i=0; i<ImageDimension; ++i )
	{
		if( size[i] < minSize )
			minSize = size[i];
	}

	if( radius >= minSize / 2.0 )
		return itk::NumericTraits<MeasureType>::Zero;

  // Do not allow radius to be smaller than half the minimum spacing in all directions
  
	const ImageType::SpacingType &spacing = m_Image->GetSpacing();

	double maxSpacing = 0.0;

	for( unsigned int i=0; i<ImageDimension; ++i )
	{
		if( spacing[i] > maxSpacing )
			maxSpacing = spacing[i];
	}

	if( radius <= maxSpacing / 2.0 )
		return itk::NumericTraits<MeasureType>::Zero;
      
  
  // Set current scale
  //m_Gradient->SetSigma( parameters[0] / 1.732050807 ); // this would use adaptive scaling
  
  
  /*if( m_CurrentScale == 0.0 )
  {
    m_CurrentScale = parameters[0] / 1.732050807;
    m_Gradient->SetSigma( m_CurrentScale );
  }
  else if( vcl_abs( m_CurrentScale - parameters[0] / 1.732050807 ) > m_ScaleTolerance )
  {
    m_CurrentScale = parameters[0] /  1.732050807;
    m_Gradient->SetSigma( m_CurrentScale );
  } */
   

  // Set the number of samples in a circle depending on the scale
	double tempSamples = 2.0f * vnl_math::pi * radius + 1.0f;
	unsigned int samples = static_cast<unsigned int>( std::floor( tempSamples + 0.5 ) );

	if( samples < 4 )
		samples = 4; // take at least four samples
	
	// Set the angle increment 
	double angleInc = vnl_math::pi * 2.0f / static_cast<double>( samples ); 
	
	double angle = 0.0, sinAngle, cosAngle; // current angle and its sin and cos
	PointType circlePoint;
	
	// Calculate points in a circle with the given radius
	for ( unsigned int i=0; i<samples; ++i )
	{
		cosAngle = cos( angle );
		sinAngle = sin( angle );
	  
		// Evaluate and store the value of the gradient in a circle (rotate 90� and calculate 4 points each time)
		
		for( int k=0; k<ImageDimension; ++k )
			circlePoint[k] = m_Center[k] + radius * ( cosAngle * firstBaseVector[k] + sinAngle * secondBaseVector[k] );

		if( m_Gradient->IsInsideBuffer( circlePoint ) )
			value += m_Gradient->Evaluate( circlePoint );
		  			
		angle += angleInc;
	}
	
	value /= samples;

	std::cout << "Radius: " << radius << "  Value: " << value << std::endl;
  
  return value; 
}


template <class TImage,class TScale> 
void
MedialnessVesselRadiusFitCostFunction<TImage,TScale>
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
 
  // Set current scale
  //m_Gradient->SetSigma( parameters[0] / 1.732050807 ); // this would use adaptive scaling
  
  /*if( m_CurrentScale == 0.0 )
  {
    m_CurrentScale = parameters[0] / 1.732050807;
    m_Gradient->SetSigma( m_CurrentScale );
  }
  else if( vcl_abs( m_CurrentScale - parameters[0] / 1.732050807 ) > m_ScaleTolerance )
  {
    m_CurrentScale = parameters[0] /  1.732050807;
    m_Gradient->SetSigma( m_CurrentScale );
  } */
 
  // Set the number of samples in a circle depending on the scale
	double tempSamples = 2.0f * vnl_math::pi * radius + 1.0f;
	unsigned int samples = static_cast<unsigned int>( std::floor( tempSamples + 0.5 ) );
	
	if( samples < 4 )
		samples = 4; // take at least four samples
	
	// Set the angle increment 
	double angleInc = vnl_math::pi * 2.0f / static_cast<double>( samples ); 
	
	MeasureType tempDerivative;
	double angle = 0.0, delta = 0.001, sinAngle, cosAngle; // current angle and its sin and cos
	PointType circlePointBack, circlePointFord;
	
	// Calculate points in a circle with the given radius
	for ( unsigned int i=0; i<samples; ++i )
	{
		cosAngle = cos( angle );
		sinAngle = sin( angle );
	  
		// Evaluate and store the value of the gradient in a circle (rotate 90� and calculate 4 points each time)
		
		for( int k=0; k<ImageDimension; ++k )
		{
			circlePointBack[k] = m_Center[k] + radius * ( 1.0 - delta ) * 
				( cosAngle * m_FirstBaseVector[k] + sinAngle * m_SecondBaseVector[k] );
			circlePointFord[k] = m_Center[k] + radius * ( 1.0 + delta ) * 
				( cosAngle * m_FirstBaseVector[k] + sinAngle * m_SecondBaseVector[k] );
			
		}

		if( m_Gradient->IsInsideBuffer( circlePointBack ) && m_Gradient->IsInsideBuffer( circlePointFord ) )
		{
			tempDerivative = 0.0;
			tempDerivative += m_Gradient->Evaluate( circlePointFord );
			tempDerivative -= m_Gradient->Evaluate( circlePointBack );
			tempDerivative /= 2.0 * delta;
			derivative[0] += tempDerivative;
		}
		  			
		angle += angleInc;
		
	}
	
	derivative[0] /= samples;

  std::cout << "Derivative:  " << derivative[0] << std::endl;  
}



template <class TImage,class TScale> 
void
MedialnessVesselRadiusFitCostFunction<TImage,TScale>
::GetValueAndDerivative( const ParametersType & parameters, 
  MeasureType & value, DerivativeType  & derivative ) const
{
  value = this->GetValue( parameters );
  this->GetDerivative( parameters, derivative );
}



} // end namespace ivan

#endif
