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
// File: ivanMedialnessVesselSectionAndRadiusFitCostFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: cost function to fit a circle to a 2D image

#ifndef __ivanMedialnessVesselSectionAndRadiusFitCostFunction_hxx
#define __ivanMedialnessVesselSectionAndRadiusFitCostFunction_hxx

#include "vnl_cross.h"

namespace ivan
{



template<class TImage,class TScale>
MedialnessVesselSectionAndRadiusFitCostFunction<TImage,TScale>
::MedialnessVesselSectionAndRadiusFitCostFunction() :
  m_SamplingDistance(0.5),
  m_SymmetryCoefficient(1.0),
  m_CurrentScale(0.0),
  m_ScaleTolerance(0.5)
{
  m_Center.Fill(0.0);
  
  m_Gradient = GradientFunctionType::New();
  m_Gradient->SetInterpolationMode( GradientFunctionType::LinearInterpolation );
  m_Gradient->SetNormalizeAcrossScale( true );
  m_Gradient->SetUseImageSpacing( true );
}



template<class TImage,class TScale>
MedialnessVesselSectionAndRadiusFitCostFunction<TImage,TScale>
::~MedialnessVesselSectionAndRadiusFitCostFunction()
{

}


template<class TImage,class TScale>
void
MedialnessVesselSectionAndRadiusFitCostFunction<TImage,TScale>
::SetImage( const ImageType * image )
{
	m_Image = image;
	m_Gradient->SetInputImage( image );
}


template <class TImage,class TScale> 
void
MedialnessVesselSectionAndRadiusFitCostFunction<TImage,TScale>
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
MedialnessVesselSectionAndRadiusFitCostFunction<TImage,TScale>
::GetNumberOfParameters(void) const
{
  return ImageDimension+1;
}


template<class TImage,class TScale>
typename MedialnessVesselSectionAndRadiusFitCostFunction<TImage,TScale>::MeasureType
MedialnessVesselSectionAndRadiusFitCostFunction<TImage,TScale>
::GetValue( const ParametersType & parameters ) const
{
  if ( m_Image.IsNull() )
    {
    itkExceptionMacro(<<"Image is null");
    }

  double radius = parameters[0];
  double a, b, c;
  
  itk::Array<double>  normal; 
  normal.SetSize(ImageDimension);
    
  normal[0] = parameters[1];
  normal[1] = parameters[2];
  normal[2] = parameters[3];

  normal.normalize();

  a = normal[0];
  b = normal[1];
  c = normal[2];
   

  // Do not allow radius to be larger than the image region or to be negative
  
  if( radius <= 0.0 )
    return itk::NumericTraits<MeasureType>::Zero;

  // Do not allow radius to be larger than half the minimum size in all directions
  
  typename RegionType::SizeType::SizeValueType minSize = 
    itk::NumericTraits<typename RegionType::SizeType::SizeValueType>::max();
  
  const SizeType &size = m_Region.GetSize();

  for( unsigned int i=0; i<ImageDimension; ++i )
  {
    if( size[i] < minSize )
      minSize = size[i];
  }

  if( radius >= minSize / 2.0 )
    return itk::NumericTraits<MeasureType>::Zero;

  // Do not allow radius to be smaller than half the minimum spacing in all directions

  const SpacingType &spacing = m_Image->GetSpacing();
  double maxSpacing = 0.0;

  for( unsigned int i=0; i<ImageDimension; ++i )
  {
    if( spacing[i] > maxSpacing )
      maxSpacing = spacing[i];
  }

  if( radius <= maxSpacing / 2.0 )
    return itk::NumericTraits<MeasureType>::Zero;
      
  
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
  
    
  itk::Array<double>    firstBaseVector; 
  itk::Array<double>    secondBaseVector;
  firstBaseVector.SetSize(ImageDimension);
  secondBaseVector.SetSize(ImageDimension);
  
  firstBaseVector[1] = sqrt( 1.0 - b*b );
  firstBaseVector[2] = ( - b*c ) / firstBaseVector[1];
  firstBaseVector[0] = ( - b*firstBaseVector[1] - c*firstBaseVector[2] ) / a;
  
  secondBaseVector = vnl_cross_3d( firstBaseVector, normal );
    
  
  // Set current scale
  //m_Gradient->SetSigma( parameters[0] / 1.732050807 ); // this would use adaptive scaling
  

  // Set the number of samples in a circle depending on the scale
  double tempSamples = 2.0f * vnl_math::pi * radius + 1.0f;
  unsigned int samples = static_cast<unsigned int>( std::floor( tempSamples + 0.5 ) );

  if( samples < 4 )
    samples = 4; // take at least four samples
  
  // Set the angle increment 
  double angleInc = vnl_math::pi * 2.0f / static_cast<double>( samples ); 
  
  
  MeasureType value = 0.0, tempValue, tempValue2; // cost function values
  PointType currentPoint, currentPoint2, circlePoint, circlePoint2;
  unsigned int calculatedPoints = 0, calculatedSamples, calculatedSamples2;
  bool isOutOfBounds = false;
  
  // Variables for medialness calculations
  double averageRadialMedialness, averageRadialMedialness2; // measure of average medialness without weighting
  double weightExponent; // exponent of weighting factors for medialness calculations
  
  // Arrays used for calculations
  itk::Array<double>  radialMedialness, radialMedialness2; // this stores all values of gradient in the circle with given radius
  itk::Array<double>  radialMedialnessWeights, radialMedialnessWeights2;  // this stores all weighting factors in the circle with given radius
  radialMedialness.SetSize( samples );
  radialMedialness2.SetSize( samples );
  radialMedialnessWeights.SetSize( samples );
  radialMedialnessWeights2.SetSize( samples );
  
  
  // Precalculate sin and cos values as these calculations are costly
  
  itk::Array<double>  sinArray; // store sin values
  itk::Array<double>  cosArray; // store cos values
  sinArray.SetSize( samples );
  conArray.SetSize( samples );
    
  double angle = 0.0;
  
  for ( unsigned int i=0; i<samples; ++i )
  {
    sinArray[i] = sin( angle );
    cosArray[i] = cos( angle );
    angle += angleInc;
  }    
    
  // Initialize the currentPoint and invertedPoint to the center
  currentPoint = currentPoint2 = m_Center;
  
  while( !isOutOfBounds )
  {
    calculatedSamples  = 0;
    calculatedSamples2 = 0;
    tempValue  = 0.0;
    tempValue2 = 0.0;
    radialMedialness.Fill(0.0);
    radialMedialness2.Fill(0.0);
    radialMedialnessWeights.Fill(0.0);
    radialMedialnessWeights2.Fill(0.0);
    
    // Calculate points in a circle with the given radius
    for ( unsigned int i=0; i<samples; ++i )
    {
      // Evaluate and store the value of the gradient in a circle (rotate 90� and calculate 4 points each time)
      // The base vectors are the same adivancing in both directions as we move in the normal of the section plane
      
      for( int k=0; k<ImageDimension; ++k )
      {
        circlePoint[k] = currentPoint[k] + radius * ( cosArray[i] * firstBaseVector[k] 
          + sinArray[i] * secondBaseVector[k] );
        if( calculatedPoints ) // do not duplicate the center point
          circlePoint2[k] = currentPoint2[k] + radius * ( cosArray[i] * firstBaseVector[k] 
          + sinArray[i] * secondBaseVector[k] );
      }
  
      if( gradient->IsInsideBuffer( circlePoint ) )
      {
        calculatedSamples++;
        radialMedialness[i] = gradient->Evaluate( circlePoint );
        //tempValue += gradient->Evaluate( circlePoint );
      }
            
      if( gradient->IsInsideBuffer( circlePoint2 ) && calculatedPoints )
      {
        calculatedSamples2++;
        radialMedialness2[i] = gradient->Evaluate( circlePoint2 );
        //tempValue2 += gradient->Evaluate( circlePoint2 );
      }
              
      angle += angleInc;
      
    }
    
    // Now sum the values and calculate weighting factors if necessary (when symmetry coefficient is not 1.0)
        
    if( calculatedSamples )
    {
      if( m_SymmetryCoefficient == 1.0 ) // speed-up calculations for the simplest (default) case
      {
        for ( unsigned int i=0; i<samples; ++i )
          tempValue += radialMedialness[i];
      }
      else
      {
        averageRadialMedialness = radialMedialness.one_norm() / static_cast<double>( calculatedSamples );
        
        for ( unsigned int i=0; i<samples; ++i )
        {
          if( !radialMedialness[i] )
            continue;
          
          // Calculate b coefficients
          weightExponent = ( 1.0 - radialMedialness[i] / averageRadialMedialness ) / m_SymmetryCoefficient;
          radialMedialnessWeights[i] = exp( -0.5 * weightExponent * weightExponent );
          tempValue += radialMedialnessWeights[i] * radialMedialness[i];
        }
        
      }
      
      tempValue /= calculatedSamples;
    }
    else
      tempValue = 0.0;


    if( calculatedSamples2 )
    { 
      if( m_SymmetryCoefficient == 1.0 ) // speed-up calculations for the simplest (default) case
      {
        for ( unsigned int i=0; i<samples; ++i )
          tempValue2 += radialMedialness2[i];
      }
      else
      {
        averageRadialMedialness2 = radialMedialness2.one_norm() / static_cast<double>( calculatedSamples2 );
        
        for ( unsigned int i=0; i<samples; ++i )
        {
          if( !radialMedialness2[i] )
            continue;
          
          // Calculate b coefficients
          weightExponent = ( 1.0 - radialMedialness2[i] / averageRadialMedialness2 ) / m_SymmetryCoefficient;
          radialMedialnessWeights2[i] = exp( -0.5 * weightExponent * weightExponent );
          tempValue2 += radialMedialnessWeights2[i] * radialMedialness2[i];
        }
      }
            
      tempValue2 /= calculatedSamples2;
    }
    else
      tempValue2 = 0.0;
    
    value += tempValue + tempValue2;
    
    calculatedPoints++;
    
          
    // Move the points in the direction of the section normal and check the finishing condition
    
    currentPoint[0] += m_SamplingDistance * normal[0];
    currentPoint[1] += m_SamplingDistance * normal[1];
    currentPoint[2] += m_SamplingDistance * normal[2];
    
    currentPoint2[0] -= m_SamplingDistance * normal[0];
    currentPoint2[1] -= m_SamplingDistance * normal[1];
    currentPoint2[2] -= m_SamplingDistance * normal[2];
    
    if( !gradient->IsInsideBuffer( currentPoint ) || !gradient->IsInsideBuffer( currentPoint2 ) )
      isOutOfBounds = true;
      
  }
  
  // In fact we have calculated two points in each iteration except for the first iteration that was only one
  calculatedPoints = 2 * ( calculatedPoints - 1 ) + 1;
  
  // Normalize the value
  if( calculatedPoints )
    value /= calculatedPoints;
  else
    value = 0.0;

  return value; 
}



template <class TImage,class TScale> 
void
MedialnessVesselSectionAndRadiusFitCostFunction<TImage,TScale>
::GetDerivative( const ParametersType & parameters, DerivativeType & derivative ) const
{
  if ( m_Image.IsNull() )
    {
    itkExceptionMacro(<<"Image is null");
    }
    
  const unsigned int ParametersDimension = this->GetNumberOfParameters();
  derivative = DerivativeType( ParametersDimension );
  derivative.Fill( NumericTraits<ITK_TYPENAME DerivativeType::ValueType>::Zero );
    
  double delta = 0.5;
  double radius = parameters[0];
  double a, b, c;
  
  // Normal and displaced normals for partial derivatives
  itk::Array<double>  normal, normaldxyzFord[ImageDimension], normaldxyzBack[ImageDimension]; 
  normal.SetSize(ImageDimension);
      
  a = normal[0] = parameters[1];
  b = normal[1] = parameters[2];
  c = normal[2] = parameters[3];
  
  for( unsigned int i=0; i<ImageDimension; ++i )
  {
    normaldxyzFord[i].SetSize(ImageDimension);
    normaldxyzBack[i].SetSize(ImageDimension);
    
    normaldxyzFord[i] = normal;
    normaldxyzBack[i] = normal;
    
    // Displace normals in x,y,z directions
    normaldxyzFord[i][i] += delta;
    normaldxyzBack[i][i] -= delta;
  }
  
 
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
  // and (x-x0) = sqrt( 1-(y-y0)^2-(z-z0)^2 ) 
  
  // Arrays for calculations
  itk::Array<double>  firstBaseVector; 
  itk::Array<double>  secondBaseVector;

  firstBaseVector.SetSize(ImageDimension);
  secondBaseVector.SetSize(ImageDimension);
  
  firstBaseVector[1] = sqrt( 1.0 - b*b );
  firstBaseVector[2] = ( - b*c ) / firstBaseVector[1];
  firstBaseVector[0] = sqrt( 1.0 - firstBaseVector[1]*firstBaseVector[1] - firstBaseVector[2]*firstBaseVector[2] );
  
  secondBaseVector = vnl_cross_3d( firstBaseVector, normal );
  
  
  // Set current scale
  //m_Gradient->SetSigma( parameters[0] / 1.732050807 ); // this would use adaptive scaling
  
    
  // Set the number of samples in a circle depending on the scale
  double tempSamples = 2.0f * vnl_math::pi * radius + 1.0f;
  unsigned int samples = static_cast<unsigned int>( std::floor( tempSamples + 0.5 ) );
  
  if( samples < 4 )
    samples = 4; // take at least four samples
  
  // Set the angle increment 
  double angleInc = vnl_math::pi * 2.0f / static_cast<double>( samples ); 
  
  // Precalculate sin and cos values as these calculations are costly
  
  itk::Array<double>  sinArray; // store sin values
  itk::Array<double>  cosArray; // store cos values
  sinArray.SetSize( samples );
  conArray.SetSize( samples );
    
  double angle = 0.0;
  
  for ( unsigned int i=0; i<samples; ++i )
  {
    sinArray[i] = sin( angle );
    cosArray[i] = cos( angle );
    angle += angleInc;
  }  
  
  //////////////////////////////////////////////////////////////////////////////////
  // Calculate derivative with respect to radius. 
  // This derivative is different because the position of the central point does not 
  // need to be recalculated (we are using the same normal to the section plane).
    
    
  // Points used to calculate the partial derivative with respect to the radius
  PointType currentPointdr, currentPointdr2;
  PointType circlePointdrBack, circlePointdrFord, circlePointdrBack2, circlePointdrFord2;
  unsigned int calculatedSamplesdr, calculatedSamplesdr2;
  
  // Used for partial calculations
  DerivativeType tempDerivative = DerivativeType( ParametersDimension );
  tempDerivative.Fill( NumericTraits<ITK_TYPENAME DerivativeType::ValueType>::Zero );
  DerivativeType tempDerivative2 = DerivativeType( ParametersDimension );
  tempDerivative2.Fill( NumericTraits<ITK_TYPENAME DerivativeType::ValueType>::Zero );
  MeasureType tempDerivative3 = NumericTraits<MeasureType>::Zero;
  
  bool isOutOfBounds = false;
  unsigned int calculatedPoints = 0; 
  
  // Initialize the central points to the center of the region
  currentPointdr = currentPointdr2 = m_Center;
    
  
  
  while( !isOutOfBounds )
  {
    angle = 0.0;
    calculatedSamplesdr  = 0;
    calculatedSamplesdr2 = 0;
    tempDerivative[0]  = NumericTraits<ITK_TYPENAME DerivativeType::ValueType>::Zero;
    tempDerivative2[0] = NumericTraits<ITK_TYPENAME DerivativeType::ValueType>::Zero;
      
    // Calculate points in a circle with the given radius
    for ( unsigned int i=0; i<samples; ++i )
    {
      // Evaluate and store the value of the gradient in a circle (rotate 90� and calculate 4 points each time)
      // The base vectors are the same adivancing in both directions as we move in the normal to the section plane
      
      for( unsigned int k=0; k<ImageDimension; ++k )
      {
        circlePointdrBack[k] = currentPointdr[k] + radius * ( 1.0 - delta ) *
            ( cosArray[i] * firstBaseVector[k] + sinArray[i] * secondBaseVector[k] );
        circlePointdrFord[k] = currentPointdr[k] + radius * ( 1.0 + delta ) *
            ( cosArray[i] * firstBaseVector[k] + sinArray[i] * secondBaseVector[k] );
                    
        if( calculatedPoints )
        {
          circlePointdrBack2[k] = currentPointdr2[k] + radius * ( 1.0 - delta ) *
            ( cosArray[i] * firstBaseVector[k] + sinArray[i] * secondBaseVector[k] );
          circlePointdrFord2[k] = currentPointdr2[k] + radius * ( 1.0 + delta ) *
            ( cosArray[i] * firstBaseVector[k] + sinArray[i] * secondBaseVector[k] );
        }
                  
      }
            
      if( gradient->IsInsideBuffer( circlePointdrBack ) && gradient->IsInsideBuffer( circlePointdrFord ) )
      {
        calculatedSamplesdr++;
        tempDerivative3 = 0.0;
        tempDerivative3 += gradient->Evaluate( circlePointdrFord );
        tempDerivative3 -= gradient->Evaluate( circlePointdrBack );
        tempDerivative3 /= 2.0 * delta;
        tempDerivative[0]  += tempDerivative3;
      }
      
      if( gradient->IsInsideBuffer( circlePointdrBack2 ) && 
        gradient->IsInsideBuffer( circlePointdrFord2 ) && calculatedPoints )
      {
        calculatedSamplesdr2++;
        tempDerivative3 = 0.0;
        tempDerivative3 += gradient->Evaluate( circlePointdrFord2 );
        tempDerivative3 -= gradient->Evaluate( circlePointdrBack2 );
        tempDerivative3 /= 2.0 * delta;
        tempDerivative2[0] += tempDerivative3;
      }
                    
      angle += angleInc;
      
    }
    
    if( calculatedSamplesdr )
      tempDerivative[0] /= calculatedSamplesdr;
    else
      tempDerivative[0] = 0.0;

    if( calculatedSamplesdr2 )
      tempDerivative2[0] /= calculatedSamplesdr2;
    else
      tempDerivative2[0] = 0.0;

    derivative[0] += tempDerivative[0] + tempDerivative2[0];
        
    calculatedPoints++;
      
      
    // Move the points in the direction of the section normal and check the finishing condition
    
    currentPointdr[0] += m_SamplingDistance * normal[0];
    currentPointdr[1] += m_SamplingDistance * normal[1];
    currentPointdr[2] += m_SamplingDistance * normal[2];
    
    currentPointdr2[0] -= m_SamplingDistance * normal[0];
    currentPointdr2[1] -= m_SamplingDistance * normal[1];
    currentPointdr2[2] -= m_SamplingDistance * normal[2];
    
    if( !gradient->IsInsideBuffer( currentPointdr ) || !gradient->IsInsideBuffer( currentPointdr2 ) )
      isOutOfBounds = true;
      
  }
  
  // In fact we have calculated two points in each iteration except for the first iteration that was only one
  calculatedPoints = 2 * ( calculatedPoints - 1 ) + 1;
  
  // Normalize the derivative
  if( calculatedPoints )
    derivative[0] /= calculatedPoints;
  else
    derivative[0] = 0.0;
  
  
  ////////////////////////////////////////////////////////
  // Calculate partial derivatives with respect to x,y,z. 
  // The central point must be recalculated (we make a 
  // displacement of the normal to the section plane).
  
  
  // Points used to calculate the partial derivative with respect to x,y,z
  PointType currentPointdxyzBack[ImageDimension], currentPointdxyzFord[ImageDimension], 
    currentPointdxyzBack2[ImageDimension], currentPointdxyzFord2[ImageDimension];
  PointType circlePointdxyzBack[ImageDimension], circlePointdxyzFord[ImageDimension], 
    circlePointdxyzBack2[ImageDimension], circlePointdxyzFord2[ImageDimension];
  
  itk::Array<unsigned int>    calculatedSamplesdxyz;
  itk::Array<unsigned int>    calculatedSamplesdxyz2;
  calculatedSamplesdxyz.SetSize(ImageDimension);
  calculatedSamplesdxyz2.SetSize(ImageDimension);
  
  isOutOfBounds = false;
  calculatedPoints = 0; 
  
  // Initialize the central points to the center of the region
  for( unsigned int i=0; i<ImageDimension; ++i )
  {
    currentPointdxyzBack[i] = currentPointdxyzFord[i] = 
      currentPointdxyzBack2[i] = currentPointdxyzFord2[i] = m_Center;
  }
  
  
  while( !isOutOfBounds )
  {
    angle = 0.0;
    calculatedSamplesdxyz.Fill(0);
    calculatedSamplesdxyz2.Fill(0);
    tempDerivative.Fill( NumericTraits<ITK_TYPENAME DerivativeType::ValueType>::Zero );
    tempDerivative2.Fill( NumericTraits<ITK_TYPENAME DerivativeType::ValueType>::Zero );
    
    // Calculate points in a circle with the given radius
    for ( unsigned int i=0; i<samples; ++i )
    {
      // Evaluate and store the value of the gradient in a circle (rotate 90� and calculate 4 points each time)
      // The base vectors are the same adivancing in both directions as we move in the normal to the section plane
      
      for( unsigned int j=0; j<ImageDimension; ++j )
      {
        for( unsigned int k=0; k<ImageDimension; ++k )
        {
          circlePointdxyzBack[j][k] = currentPointdxyzBack[j][k] + radius * 
            ( cosArray[i] * firstBaseVector[k] + sinArray[i] * secondBaseVector[k] );
          circlePointdxyzFord[j][k] = currentPointdxyzFord[j][k] + radius * 
            ( cosArray[i] * firstBaseVector[k] + sinArray[i] * secondBaseVector[k] );
            
          if( calculatedPoints )
          {
            circlePointdxyzBack2[j][k] = currentPointdxyzBack2[j][k] + radius * 
              ( cosArray[i] * firstBaseVector[k] + sinArray[i] * secondBaseVector[k] );
            circlePointdxyzFord2[j][k] = currentPointdxyzFord2[j][k] + radius * 
              ( cosArray[i] * firstBaseVector[k] + sinArray[i] * secondBaseVector[k] );
          }
          
        } // end for k
                  
        if( gradient->IsInsideBuffer( circlePointdxyzBack[j] ) && 
            gradient->IsInsideBuffer( circlePointdxyzFord[j] ) )
        {
          calculatedSamplesdxyz[j]++;
          tempDerivative3 = 0.0;
          tempDerivative3 += gradient->Evaluate( circlePointdrFord );
          tempDerivative3 -= gradient->Evaluate( circlePointdrBack );
          tempDerivative3 /= 2.0 * delta;
          tempDerivative[j+1]  += tempDerivative3;
        }
        
        if( gradient->IsInsideBuffer( circlePointdxyzBack2[j] ) && 
            gradient->IsInsideBuffer( circlePointdxyzFord2[j] ) && calculatedPoints )
        {
          calculatedSamplesdxyz2[j]++;
          tempDerivative3 = 0.0;
          tempDerivative3 += gradient->Evaluate( circlePointdrFord2 );
          tempDerivative3 -= gradient->Evaluate( circlePointdrBack2 );
          tempDerivative3 /= 2.0 * delta;
          tempDerivative2[j+1] += tempDerivative3;
        }
        
      } // end for j
                      
      angle += angleInc;
      
    } // end for i
    
    for( unsigned int j=0; j<ImageDimension; ++j )
    {
      if( calculatedSamplesdxyz[j] )
        tempDerivative[j+1] /= calculatedSamplesdxyz[j];
      else
        tempDerivative[j+1] = 0.0;

      if( calculatedSamplesdxyz2[j] )
        tempDerivative2[j+1] /= calculatedSamplesdxyz2[j];
      else
        tempDerivative2[j+1] = 0.0;
    
      derivative[j+1] += tempDerivative[j+1] + tempDerivative2[j+1];
    }
        
    calculatedPoints++;
      
      
    // Move the points in the direction of the section displaced normals and check the finishing condition
    
    for( unsigned int j=0; j<ImageDimension; ++j )
    {
      for( unsigned int i=0; i<ImageDimension; ++i )
      {
        currentPointdxyzBack[j][i] += m_SamplingDistance * normaldxyzBack[j][i];
        currentPointdxyzFord[j][i] += m_SamplingDistance * normaldxyzFord[j][i];
        currentPointdxyzBack2[j][i] -= m_SamplingDistance * normaldxyzBack[j][i];
        currentPointdxyzFord2[j][i] -= m_SamplingDistance * normaldxyzFord[j][i];
      }
      
      if( !gradient->IsInsideBuffer( currentPointdxyzBack[j] ) || 
          !gradient->IsInsideBuffer( currentPointdxyzFord[j] ) ||
          !gradient->IsInsideBuffer( currentPointdxyzBack2[j] ) ||
          !gradient->IsInsideBuffer( currentPointdxyzFord2[j] ) )
        isOutOfBounds = true;
    
    }
          
  }
  
  // In fact we have calculated two points in each iteration except for the first iteration that was only one
  calculatedPoints = 2 * ( calculatedPoints - 1 ) + 1;
  
  // Normalize the derivatives
  if( calculatedPoints )
  {
    derivative[1] /= calculatedPoints;
    derivative[2] /= calculatedPoints;
    derivative[3] /= calculatedPoints;
  }
  else
  {
    derivative[1] = 0.0;
    derivative[2] = 0.0;
    derivative[3] = 0.0;
  }
  
}



template <class TImage,class TScale> 
void
MedialnessVesselSectionAndRadiusFitCostFunction<TImage,TScale>
::GetValueAndDerivative( const ParametersType & parameters, 
  MeasureType & value, DerivativeType  & derivative ) const
{
  value = this->GetValue( parameters );
  this->GetDerivative( parameters, derivative );
}



} // end namespace ivan

#endif
