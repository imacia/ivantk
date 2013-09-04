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
// File: ivanCenterlineBasedGaussianTubeGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a curved Gaussian tube of the given size, stddev and peak intensity value
// following the provided centerline.
// Date: 2010/10/01


#ifndef __ivanCenterlineBasedGaussianTubeGenerator_h_
#define __ivanCenterlineBasedGaussianTubeGenerator_h_

#include "ivanCircularGaussianTubeGenerator.h"

#include "itkGaussianSpatialFunction.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkVectorContainer.h"
#include "itkImageFileWriter.h"

//#define DISTANCE_TOL_FACTOR 4.0 // this means exp(-16/2) = exp(-8) = 0.000335462628 for the min Gaussian value for this tolerance
#define DISTANCE_TOL_FACTOR 3.0 // this means exp(-9/2) = exp(-4.5) = 0.011 for the min Gaussian value for this tolerance

namespace ivan
{

template <class TPixel>
class CenterlineBasedGaussianTubeGenerator :
  public CircularGaussianTubeGenerator<TPixel>
{
public:

  typedef CenterlineBasedGaussianTubeGenerator<TPixel>  Self;
  typedef CircularGaussianTubeGenerator<TPixel>         Superclass;

  typedef TPixel                         PixelType;
  typedef itk::Image<PixelType,3>        ImageType;
  typedef typename ImageType::Pointer    ImagePointer;
  typedef typename ImageType::PointType  PointType;
  typedef typename ImageType::IndexType  IndexType;
    
  typedef itk::VectorContainer<unsigned int,PointType>  PointContainerType;
  typedef typename PointContainerType::Pointer          PointContainerPointer;

public:

  CenterlineBasedGaussianTubeGenerator();
  ~CenterlineBasedGaussianTubeGenerator() {};
  
  /** Set point vector defining the centerline. */
  virtual void SetCenterline( PointContainerType *centerline )
    { m_Centerline = centerline; }
    
  PointContainerType * GetCenterline() 
    { return m_Centerline; }
  const PointContainerType * GetCenterline() const 
    { return m_Centerline; }  

  virtual ImagePointer Create();
  
protected:
  
  /** Create and allocate the image with the appropiate dimensions, spacing... */
  virtual ImagePointer CreateEmptyImage();
  
  /** Actually draw/fill the image from the centerline, once the image structure is created. */
  void DrawFromCenterline( ImagePointer emptyImage );
  
  unsigned int GetClosestCenterlinePoint( const PointType & point, double & distance );

protected:

  PointContainerPointer   m_Centerline;
};


template <class TPixel>
CenterlineBasedGaussianTubeGenerator<TPixel>::CenterlineBasedGaussianTubeGenerator()
{
 
}


template <class TPixel>
unsigned int 
CenterlineBasedGaussianTubeGenerator<TPixel>
::GetClosestCenterlinePoint( const PointType & point, double & distance )
{
  double sqrDist = itk::NumericTraits<double>::max();
  double current;
  unsigned int idx = m_Centerline->size();
  
  for( unsigned int i=0; i<m_Centerline->size(); ++i )
  {
    current = 0.0;

    for( unsigned int dim = 0; dim<3; ++dim )
      current += ( point[dim] - m_Centerline->at(i)[dim] ) * ( point[dim] - m_Centerline->at(i)[dim] );
      
    if( current < sqrDist )
    {
      idx = i;
      sqrDist = current; 
    } 
  }
  
  distance = sqrt( sqrDist );
  
  return idx;
}


template <class TPixel>
typename CenterlineBasedGaussianTubeGenerator<TPixel>::ImagePointer
CenterlineBasedGaussianTubeGenerator<TPixel>::CreateEmptyImage()
{
  ImagePointer tubeImage = ImageType::New();
  
  typename ImageType::SpacingType spacing;
  spacing.Fill( this->m_ImageSpacing );
  
  tubeImage->SetSpacing( spacing );
  
  typename ImageType::PointType origin;
  origin.Fill( 0.0 );
  
  tubeImage->SetOrigin( origin );
  
  double totalHeight = this->m_Height + 2.0 * this->m_Offset;
  
  typename ImageType::RegionType::SizeType size;
  size[0] = this->m_SectionImageSize[0];
  size[1] = this->m_SectionImageSize[1];
  size[2] = (unsigned long) floor( ( totalHeight / this->m_ImageSpacing ) + 0.5 );
  
  typename ImageType::RegionType::IndexType index;
  index.Fill(0);
  
  typename ImageType::RegionType region;
  region.SetSize( size );
  region.SetIndex( index );
  
  tubeImage->SetRegions( region );
  
  tubeImage->Allocate();
  tubeImage->FillBuffer(0);
    
  return tubeImage;
}


template <class TPixel>
void
CenterlineBasedGaussianTubeGenerator<TPixel>::DrawFromCenterline( ImagePointer emptyImage )
{
  typedef itk::GaussianSpatialFunction<double, 1>   GaussianFunctionType;
  typedef typename GaussianFunctionType::InputType  GaussianFunctionPositionType;
  typename GaussianFunctionType::Pointer gaussian = GaussianFunctionType::New();
   
  GaussianFunctionType::ArrayType sigma;
  sigma.Fill( this->m_Sigma );

  GaussianFunctionType::ArrayType mean;
  mean.Fill( 0.0 );
   
  gaussian->SetSigma( sigma );
  gaussian->SetMean( mean );
  gaussian->SetNormalized( this->m_Normalize );
  
  double rescaleFactor = 1.0;
  double value;
  
  typename GaussianFunctionType::InputType gaussianPoint; // this is a 1-D point representing the distance
  
  if( this->m_Rescale && !this->m_Normalize )
  {
    // Compute rescaling factor from central value of the Gaussian 
    gaussianPoint[0] = 0.0;
    value = gaussian->Evaluate( gaussianPoint );
    
    rescaleFactor = this->m_MaxValue / value;
  }   
  
  typedef itk::ImageRegionIteratorWithIndex<ImageType> IteratorType;
  
  IteratorType outputIt( emptyImage, emptyImage->GetRequestedRegion() );
  outputIt.GoToBegin();
  
  PointType point;
  double    distance;
  unsigned int z = 0;
   
  while( !outputIt.IsAtEnd() )
  {
    emptyImage->TransformIndexToPhysicalPoint( outputIt.GetIndex(), point );
    point[2] -= this->m_Offset;

    //if( outputIt.GetIndex()[0] == 133 && outputIt.GetIndex()[1] == 150 && outputIt.GetIndex()[2] == 0 )
      //std::cout << outputIt.GetIndex();
    
    if( outputIt.GetIndex()[2] > z )
    {
      z = outputIt.GetIndex()[2];
      std::cout << "Z = " << z << std::endl;
    }

    if( this->GetClosestCenterlinePoint( point, distance ) < m_Centerline->size() )
    {
      if( distance < DISTANCE_TOL_FACTOR * this->m_Sigma )
      {
        gaussianPoint[0] = distance;
        value = gaussian->Evaluate( gaussianPoint );
        value *= rescaleFactor;
        
        // Set the pixel value to the function value
        outputIt.Set( (PixelType) value );        
      }   
    }
    
    ++outputIt; 
  }  
}


template <class TPixel>
typename CenterlineBasedGaussianTubeGenerator<TPixel>::ImagePointer
CenterlineBasedGaussianTubeGenerator<TPixel>::Create()
{
  ImagePointer tubeImage = this->CreateEmptyImage();
  this->DrawFromCenterline( tubeImage );
  
  return tubeImage;    
}

} // end namespace ivan

#endif // __CenterlineBasedGaussianTubeGenerator_h_

