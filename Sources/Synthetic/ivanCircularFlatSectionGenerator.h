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
// File: ivanCircularFlatSectionGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a Gaussian image of the given size, stddev and peak intensity value
// Date: 2010/09/17


#ifndef __ivanCircularFlatSectionGenerator_h_
#define __ivanCircularFlatSectionGenerator_h_

#include "itkImage.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkFixedArray.h"

namespace ivan
{

template <class TPixel>
class CircularFlatSectionGenerator
{
public:

  typedef TPixel                        PixelType;
  typedef itk::Image<PixelType,2>       ImageType;
  typedef typename ImageType::Pointer   ImagePointer;
  
  typedef itk::FixedArray<unsigned long,2>   SectionSizeType;

public:

  CircularFlatSectionGenerator();
  ~CircularFlatSectionGenerator() {};

  void SetImageSize( unsigned long sizeX, unsigned long sizeY )
    { m_ImageSize[0] = sizeX; m_ImageSize[1] = sizeY; }
  void SetImageSize( unsigned long size )
    { m_ImageSize[0] = size; m_ImageSize[1] = size; }
  void SetImageSize( const SectionSizeType & size )
    { m_ImageSize = size;  }
  const SectionSizeType & GetImageSize() const
    { return m_ImageSize; }
    
  void SetImageSpacing( double spacing )
    { m_ImageSpacing = spacing; }
  double GetImageSpacing() const
    { return m_ImageSpacing; }

  void SetRadius( double radius )
    { m_Radius = radius; }
  double GetRadius() const
    { return m_Radius; }

  void SetMaxValue( PixelType value )
    { m_MaxValue = value; }
  PixelType GetMaxValue() const
    { return m_MaxValue; }

  ImagePointer Create();

private: 
  
  /** Image size (same for height and width). */
  SectionSizeType m_ImageSize;
  
  /** Image spacing. */
  double          m_ImageSpacing;
  
  /** Radius of the section. */
  double          m_Radius;
  
  /** Maximum intensity value. */
  PixelType       m_MaxValue;
};


template <class TPixel>
CircularFlatSectionGenerator<TPixel>::CircularFlatSectionGenerator() :
  m_ImageSpacing( 1.0 ),
  m_Radius( 2.0 ),
  m_MaxValue( 255.0 )
{
  m_ImageSize.Fill( 50 );
}


template <class TPixel>
typename CircularFlatSectionGenerator<TPixel>::ImagePointer
CircularFlatSectionGenerator<TPixel>::Create()
{
  ImagePointer sectionImage = ImageType::New();
  
  typename ImageType::SpacingType spacing;
  spacing.Fill( this->m_ImageSpacing );
  
  sectionImage->SetSpacing( spacing );
  
  typename ImageType::PointType origin;
  origin.Fill( 0.0 );
  
  sectionImage->SetOrigin( origin );
  
  typename ImageType::RegionType::SizeType size;
  size[0] = this->m_ImageSize[0];
  size[1] = this->m_ImageSize[1];
    
  typename ImageType::RegionType::IndexType index;
  index.Fill(0);
  
  typename ImageType::RegionType region;
  region.SetSize( size );
  region.SetIndex( index );
  
  sectionImage->SetRegions( region );
  
  sectionImage->Allocate();
  sectionImage->FillBuffer(0);
  
  
  typedef itk::ImageRegionIterator<ImageType>   IteratorType;
  
  IteratorType it( sectionImage, sectionImage->GetRequestedRegion() );
  it.GoToBegin();
  
  ImageType::IndexType currentIndex;
  ImageType::PointType currentPoint;
  ImageType::PointType center;
  
  center[0] = ( size[0] - 1 ) * spacing[0] / 2.0;
  center[1] = ( size[1] - 1 ) * spacing[1] / 2.0;
  
  double squaredDist;
  double diff;
  
  while( !it.IsAtEnd() )
  {
    currentIndex = it.GetIndex();
    currentPoint[0] = currentIndex[0] * spacing[0];
    currentPoint[1] = currentIndex[1] * spacing[1];
    
    squaredDist = ( currentPoint[0] - center[0] ) * ( currentPoint[0] - center[0] ) + 
      ( currentPoint[1] - center[1] ) * ( currentPoint[1] - center[1] );
      
    // Perform some aliasing when the distance exceeds the radius only by a quantity smaller than the spacing
      
    diff = sqrt( squaredDist ) - this->m_Radius;
          
    if( diff > 0.0 && diff < this->m_ImageSpacing )
      it.Set( this->m_MaxValue * ( this->m_ImageSpacing - diff ) / this->m_ImageSpacing );
    else if( squaredDist <= this->m_Radius * this->m_Radius )
      it.Set( this->m_MaxValue );
          
    ++it; 
  }
   
  return sectionImage;    
}

} // end namespace ivan

#endif // __ivanCircularFlatSectionGenerator_h_
