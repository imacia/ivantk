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
// File: ivanPolarProfileVesselnessImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/08/17

#ifndef __ivanPolarProfileVesselnessImageFunction_hxx
#define __ivanPolarProfileVesselnessImageFunction_hxx

#include "ivanPolarProfileVesselnessImageFunction.h"

#include "itkVector.h"


namespace ivan
{

/** Set the Input Image */
template <class TInputImage, class TOutput, class TCoordRep>
PolarProfileVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::PolarProfileVesselnessImageFunction() : 
  m_SpatialWeightingType( UniformSpatialWeighting ),
  m_Beta( 0.01 ),
  m_Sigma( 1.0 ),
  m_Tau( 1.0 ),
  m_Gamma( 3.0 ) // 0.5
{
  this->m_RadialResolution = 5; // for this filter only
  
  this->m_Interpolator = InterpolatorType::New();
}


/** Print self method */
template <class TInputImage, class TOutput, class TCoordRep>
void
PolarProfileVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
  os << indent << "SpatialWeightingType: " << this->m_SpatialWeightingType << std::endl;
  os << indent << "Beta: " << this->m_Beta << std::endl;
  os << indent << "Sigma: " << this->m_Sigma << std::endl;
  os << indent << "Tau: " << this->m_Tau << std::endl;
  os << indent << "Gamma: " << this->m_Gamma << std::endl;
}


/** Set the input image */
template <class TInputImage, class TOutput, class TCoordRep>
void
PolarProfileVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::SetInputImage( const InputImageType * ptr )
{
  Superclass::SetInputImage( ptr );
  
  this->m_Interpolator->SetInputImage( ptr );
}


/** Evaluate the function at the specifed index */
template <class TInputImage, class TOutput, class TCoordRep>
typename PolarProfileVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
PolarProfileVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateAtIndex( const IndexType& index ) const
{
  PointType point;
  this->GetInputImage()->TransformIndexToPhysicalPoint( index, point );
  
  return this->Evaluate( point ); 
}


/** Evaluate the function at the specifed point */
template <class TInputImage, class TOutput, class TCoordRep>
typename PolarProfileVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
PolarProfileVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::Evaluate( const PointType& point ) const
{
  assert( this->m_Sphere.IsNotNull() );
  
  typename SphereType::PointsContainer *spherePoints = this->m_Sphere->GetPoints();
  typename SphereType::PointsContainerIterator pointIterator = spherePoints->Begin();
  typename SphereType::PointsContainerIterator minPointIterator = pointIterator;
    
  std::vector<TOutput> sectorValues( spherePoints->Size(), 0.0 );
  std::vector<TOutput> entropyValues( spherePoints->Size(), 0.0 );
    
  const double weightingValue = 1.0; // TO MODIFY DEPENDING ON m_SpatialWeightingType
  const double samplingDistance = this->m_Radius / this->m_RadialResolution;
  
  double centerValue = this->m_Interpolator->Evaluate( point );
  double sampleValue = 0.0;
  double sectorValuesTotal = 0.0;
  
  typedef itk::Vector<double,3>   VectorType;
  VectorType radialVector;
  PointType  samplePoint;
  
  // Variables used to calculate the sectors with minimum and maximum deviation
  unsigned int sectorNum = 0, maxDevIdx = 0, minDevIdx = 0;    
  double maxDev = itk::NumericTraits<double>::min(), minDev = itk::NumericTraits<double>::max();
  std::vector<TOutput> minIntensityValues( this->m_RadialResolution, 0.0 );
  
  while( pointIterator != spherePoints->End() )
  {
    for( unsigned int dim = 0; dim < TInputImage::GetImageDimension(); ++dim )
      radialVector[dim] = pointIterator.Value()[dim];
    
    radialVector.Normalize();
    
    for( unsigned int i=1; i < this->m_RadialResolution + 1; ++i )
    {      
      // Add the corresponding radial distance to the sampling point      
      for( unsigned int dim = 0; dim < TInputImage::GetImageDimension(); ++dim )
        samplePoint[dim] = point[dim] + radialVector[dim] * (double)i * samplingDistance;

      if( m_Interpolator->IsInsideBuffer( samplePoint ) )
        sampleValue = m_Interpolator->Evaluate( samplePoint );
      else
        sampleValue = 0.0;
      
      sectorValues[sectorNum] += ( sampleValue - centerValue ) * ( sampleValue - centerValue );
    }
    
    if( sectorValues[sectorNum] > maxDev )
    {
      maxDev = sectorValues[sectorNum];
      maxDevIdx = sectorNum;      
    }
      
    if( sectorValues[sectorNum] < minDev )
    {
      minDev = sectorValues[sectorNum];
      minDevIdx = sectorNum;
      minPointIterator = pointIterator;
    }
    
    // Now reuse the vector to calculate probability
    sectorValues[sectorNum] = vcl_exp( - m_Beta * sectorValues[sectorNum] / ( m_Sigma * m_Sigma ) );
    sectorValuesTotal += sectorValues[sectorNum];
    
    ++pointIterator;
    ++sectorNum;
  }
  
  // Recover the intensity values within the region with minimum deviation and calculate their mean
  
  for( unsigned int dim = 0; dim < TInputImage::GetImageDimension(); ++dim )
    radialVector[dim] = minPointIterator.Value()[dim];
    
  radialVector.Normalize();
  
  minDev = 0.0;
  
  for( unsigned int i=1; i < minIntensityValues.size(); ++i )
  {      
    // Add the corresponding radial distance to the sampling point      
    for( unsigned int dim = 0; dim < TInputImage::GetImageDimension(); ++dim )
      samplePoint[dim] = point[dim] + radialVector[dim] * (double)i * samplingDistance;

    if( m_Interpolator->IsInsideBuffer( samplePoint ) )
      minIntensityValues[i-1] = m_Interpolator->Evaluate( samplePoint );
    else
      minIntensityValues[i-1] = 0.0;
      
    minDev += minIntensityValues[i-1];      
  }
  
  // Calculate mean    
  minDev /= (double)minIntensityValues.size();
  
  
  // Now we can normalize the probability distribution so all values sum exactly one
  
  double entropy = 0.0;
  
  for( unsigned int i=0; i<sectorValues.size(); ++i )
  {
    entropyValues[i] = sectorValues[i] * vcl_log( sectorValues[i] );
	  
	if( sectorValuesTotal > 1e-6 )
      sectorValues[i] /= sectorValuesTotal;
    else
      sectorValues[i] = 0.0;

	if( sectorValues[i] > 1e-6 )
      entropy += sectorValues[i] * vcl_log( sectorValues[i] );
      // The VTK implementation uses log but shouldn't be log10???
      //entropy += sectorValues[i] * vcl_log10( sectorValues[i] ); 
  }

  double tightness = vcl_exp( m_Tau * entropy );
  //double brightness = 1.0 / ( 1.0 + vcl_exp( - m_Gamma * ( maxDev - minDev ) ) );
  double brightness = 1.0 - vcl_exp( - 0.5 * minDev * minDev / ( m_Sigma * m_Sigma * m_Gamma ) );
  
  double vesselness = tightness * brightness;
  //double vesselness = brightness;
  
  return vesselness;
}


/** Evaluate the function at specified ContinousIndex position.*/
template <class TInputImage, class TOutput, class TCoordRep>
typename PolarProfileVesselnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
PolarProfileVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateAtContinuousIndex( const ContinuousIndexType & cindex ) const
{
  PointType point;
  this->GetInputImage()->TransformContinuousIndexToPhysicalPoint( cindex, point );
  
  return this->Evaluate( point ); 
}

} // end namespace ivan

#endif
