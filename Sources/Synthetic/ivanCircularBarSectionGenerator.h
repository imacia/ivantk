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
// File: ivanCircularBarSectionGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a Bar image of the given size, stddev and peak intensity value
// Date: 2010/09/17


#ifndef __ivanCircularBarSectionGenerator_h_
#define __ivanCircularBarSectionGenerator_h_

#include "itkImage.h"
#include "itkImageRegionIterator.h"
#include "itkImageRegionIteratorWithIndex.h"
//#include "itkDiscreteBarImageFilter.h"
//#include "itkRescaleIntensityImageFilter.h"
//#include "itkCastImageFilter.h"


namespace ivan
{

template <class TPixel>
class CircularBarSectionGenerator
{
public:

  typedef TPixel                        PixelType;
  typedef itk::Image<PixelType,2>       ImageType;
  typedef typename ImageType::Pointer   ImagePointer;
  typedef typename ImageType::PointType PointType;

public:

  CircularBarSectionGenerator();
  ~CircularBarSectionGenerator() {};
  
  void SetImageSize( unsigned long size )
    { m_ImageSize = size; }
  unsigned long GetImageSize() const
    { return m_ImageSize; }
    
  void SetImageSpacing( double spacing )
    { m_ImageSpacing = spacing; }
  double GetImageSpacing() const
    { return m_ImageSpacing; }
  
  void SetRadius( double tubeRadius )
    { m_Radius = tubeRadius; }
  double GetRadius() const
    { return m_Radius; }

  PointType GetCenter(  )
    { return m_Center; }
    
  void SetMaxValue( PixelType value )
    { m_MaxValue = value; }
  PixelType GetMaxValue() const
    { return m_MaxValue; }

  ImagePointer Create();

private: 
  
  /** Image size in pixels (same for height and width). */
  unsigned long   m_ImageSize;
  
  /** Image spacing. */
  double          m_ImageSpacing;
  
  /** Standard deviation of the Gaussian in both directions. */
  double          m_Radius;
  
  /** Maximum intensity value under rescaling. Default value is 255. */
  PixelType       m_MaxValue;

  /** Center of the section. */
  PointType       m_Center;
};


template <class TPixel>
CircularBarSectionGenerator<TPixel>::CircularBarSectionGenerator() :
  m_ImageSpacing( 1.0 ),
  m_ImageSize( 50 ),
  m_Radius( 2.0 ),
  m_MaxValue( 255.0 )
{
  m_Center.Fill( 0 );

}


template <class TPixel>
typename CircularBarSectionGenerator<TPixel>::ImagePointer
CircularBarSectionGenerator<TPixel>::Create()
{
  //typedef double   RealPixelType;
  typedef itk::Image<TPixel, 2>              RealImageType;

  typename RealImageType::Pointer sectionImage = RealImageType::New();
    
  typename RealImageType::SpacingType spacing;
  spacing.Fill( this->m_ImageSpacing );
  sectionImage->SetSpacing( spacing );
  
  typename RealImageType::RegionType region;
    
  typename RealImageType::RegionType::SizeType size;    
  size.Fill( this->m_ImageSize );
  region.SetSize( size );
  
  typename RealImageType::RegionType::IndexType index;    
  index.Fill(0);
  region.SetIndex( index );
  
  sectionImage->SetRegions( region );
  sectionImage->Allocate();
  sectionImage->FillBuffer(0);
  
  // Taking into account the aliasing
  typedef itk::ImageRegionIterator<ImageType>   IteratorType;
  
  IteratorType it( sectionImage, sectionImage->GetRequestedRegion() );
  it.GoToBegin();
  
  typename ImageType::IndexType currentIndex;
  typename ImageType::PointType currentPoint;
  
  this->m_Center[0] = ( size[0] - 1 ) * spacing[0] / 2.0;
  this->m_Center[1] = ( size[1] - 1 ) * spacing[1] / 2.0;
  
  double squaredDist;
  double diff;
  double squaredRadius = this->GetRadius() * this->GetRadius();
  
  while( !it.IsAtEnd() )
  {
    currentIndex = it.GetIndex();
    currentPoint[0] = currentIndex[0] * this->GetImageSpacing();
    currentPoint[1] = currentIndex[1] * this->GetImageSpacing();
    
    squaredDist = ( currentPoint[0] - this->m_Center[0] ) * ( currentPoint[0] - this->m_Center[0] ) + 
      ( currentPoint[1] - this->m_Center[1] ) * ( currentPoint[1] - this->m_Center[1] );
      
    // Perform some aliasing when the distance exceeds the radius only by a quantity smaller than the spacing
      
    diff = sqrt( squaredDist ) - this->GetRadius();
          
    if( diff > 0.0 && diff < this->GetImageSpacing() )
    {
      it.Set( this->GetMaxValue() * ( this->m_ImageSpacing - diff ) / this->GetImageSpacing() );
    }
    else if( squaredDist <= squaredRadius )
    {
      it.Set( this->GetMaxValue() );
    }
          
    ++it; 
  }
      
  return sectionImage;
}

} // end namespace ivan

#endif // __ivanCircularGaussianSectionGenerator_h_

