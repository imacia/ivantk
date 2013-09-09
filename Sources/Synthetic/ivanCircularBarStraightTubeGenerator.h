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
// File: ivanCircularBarStraightTubeGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a Bar straight tube of the given size, stddev and peak intensity value
// Date: 2010/09/17


#ifndef __ivanCircularBarStraightTubeGenerator_h_
#define __ivanCircularBarStraightTubeGenerator_h_

#include "ivanCircularBarTubeGenerator.h"
#include "itkVectorContainer.h"

namespace ivan
{

template <class TPixel>
class CircularBarStraightTubeGenerator : public CircularBarTubeGenerator<TPixel>
{
public:

  typedef CircularBarStraightTubeGenerator<TPixel>    Self;
  typedef CircularBarTubeGenerator<TPixel>            Superclass;

  typedef TPixel                        PixelType;
  typedef itk::Image<PixelType,3>       ImageType;
  typedef typename ImageType::Pointer   ImagePointer;
  typedef typename ImageType::PointType PointType;
  typedef typename itk::Image<PixelType,2>::PointType   SectionPointType;

  typedef itk::VectorContainer<unsigned int,PointType>  CenterlineType;
  typedef typename CenterlineType::Pointer              CenterlinePointer;
  
public:

  CircularBarStraightTubeGenerator();
  ~CircularBarStraightTubeGenerator() {};
  
  CenterlinePointer GetCenterline()
  { return this->m_CenterlinePointer; }

  SectionPointType GetCenter()
  { return this->m_Center; }

  virtual ImagePointer Create();

private:
  CenterlinePointer                   m_CenterlinePointer;
  SectionPointType                    m_Center;
  
};


template <class TPixel>
CircularBarStraightTubeGenerator<TPixel>::CircularBarStraightTubeGenerator()
{
  m_CenterlinePointer = CenterlineType::New();
  
}


template <class TPixel>
typename CircularBarStraightTubeGenerator<TPixel>::ImagePointer
CircularBarStraightTubeGenerator<TPixel>::Create()
{
  ImagePointer tubeImage = ImageType::New();
  
  typename ImageType::SpacingType spacing;
  spacing[0] = this->m_ImageSpacing;
  spacing[1] = this->m_ImageSpacing;
  spacing[2] = this->m_ImageSpacing; // Before, it was fixed to 1.0
  
  tubeImage->SetSpacing( spacing );
  
  typename ImageType::PointType origin;
  origin.Fill( 0.0 );
  
  tubeImage->SetOrigin( origin );

  double imageSizeZ = vcl_floor( ( this->m_Height + 2*this->m_ZOffset ) / this->m_ImageSpacing );
  
  typename ImageType::RegionType::SizeType size;
  size[0] = this->m_SectionImageSize;
  size[1] = this->m_SectionImageSize;
  size[2] = vcl_floor( imageSizeZ );
  
  typename ImageType::RegionType::IndexType index;
  index.Fill(0);
  
  typename ImageType::RegionType region;
  region.SetSize( size );
  region.SetIndex( index );
  
  tubeImage->SetRegions( region );
  
  tubeImage->Allocate();
  tubeImage->FillBuffer(0);
  
  // Create Bar section
  
  typedef CircularBarSectionGenerator<TPixel>         SectionGeneratorType;
  typedef typename SectionGeneratorType::ImageType    SectionImageType;
    
  SectionGeneratorType sectionGenerator;
  sectionGenerator.SetImageSize( this->m_SectionImageSize );
  sectionGenerator.SetImageSpacing( this->m_ImageSpacing );
  sectionGenerator.SetRadius( this->m_TubeRadius );
  sectionGenerator.SetMaxValue( this->m_MaxValue );
  
  typename SectionImageType::Pointer sectionImage = sectionGenerator.Create();

  // Get center

  this->m_Center[0] = sectionGenerator.GetCenter()[0];
  this->m_Center[1] = sectionGenerator.GetCenter()[1];
  
  typedef itk::ImageRegionConstIterator<SectionImageType>  ConstIteratorType;
  typedef itk::ImageRegionIterator<ImageType>              IteratorType;
  
  ConstIteratorType inputIt( sectionImage, sectionImage->GetRequestedRegion() );
  IteratorType outputIt;
  
  size[2] = 1; // copy a single slice each time
  region.SetSize( size );

  unsigned long firstSlice = vcl_floor( this->m_ZOffset / this->m_ImageSpacing );
  unsigned long lastSlice = imageSizeZ - firstSlice ;
  
  int i = 0;

  ImageType::PointType centerlinePoint;
  centerlinePoint[0] = this->m_Center[0];
  centerlinePoint[1] = this->m_Center[1];

  unsigned int numPoints = lastSlice - firstSlice ;

  this->m_CenterlinePointer->resize( numPoints );

  for( unsigned long z = firstSlice; z < lastSlice ; ++z )
  {
    
    index[2] = z; // current slice ( index units )
    region.SetIndex( index );
    
    outputIt = IteratorType( tubeImage, region );
    
    inputIt.GoToBegin();
    outputIt.GoToBegin();

    // Create centerline

    centerlinePoint[2] = origin[2] + index[2] * this->m_ImageSpacing; // Convert to physical units.

    (*this->m_CenterlinePointer)[i] = centerlinePoint;
    i++;
    
    while( !inputIt.IsAtEnd() )
    {
      outputIt.Set( inputIt.Get() );
      ++inputIt;
      ++outputIt;
    }
  }
  
  return tubeImage;    
}

} // end namespace ivan

#endif // __ivanCircularBarStraightTubeGenerator_h_

