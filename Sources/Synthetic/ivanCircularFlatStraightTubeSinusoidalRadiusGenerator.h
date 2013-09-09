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
// File: ivanCircularBarStraightTubeSinuosidalRadiusGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a straight tube with bar profile of the given size, 
// stddev and peak intensity value and with radius following a sinusoid.
// Date: 2012/04/03


#ifndef __ivanCircularBarStraightTubeSinuosidalRadiusGenerator_h_
#define __ivanCircularBarStraightTubeSinuosidalRadiusGenerator_h_

#include "ivanCircularBarTubeGenerator.h"

namespace ivan
{

template <class TPixel>
class CircularBarStraightTubeSinuosidalRadiusGenerator : public CircularBarTubeGenerator<TPixel>
{
public:

  typedef CircularBarStraightTubeSinuosidalRadiusGenerator<TPixel>  Self;
  typedef CircularBarTubeGenerator<TPixel>                          Superclass;

  typedef TPixel                        PixelType;
  typedef itk::Image<PixelType,3>       ImageType;
  typedef typename ImageType::Pointer   ImagePointer;
  
  typedef typename Superclass::SectionSizeType   SectionSizeType;
  
public:

  CircularBarStraightTubeSinuosidalRadiusGenerator();
  ~CircularBarStraightTubeSinuosidalRadiusGenerator() {};
  
  virtual ImagePointer Create();
  
  void SetMinRadius( double minRadius )
    { this->m_MinRadius = minRadius; }
  double GetMinRadius() const
    { return this->m_MinRadius; }

  void SetSemiPeriod( double semiPeriod )
    { this->m_SemiPeriod = semiPeriod; }
  double GetSemiPeriod() const
    { return this->m_SemiPeriod; }
 
  void SetMaxRadius( double maxRadius )
    { this->SetRadius( maxRadius ); }
  double GetMaxRadius() const 
    { return this->GetSigma(); }

protected: 
  
  double    m_MinRadius;
  double    m_SemiPeriod;
};


template <class TPixel>
CircularBarStraightTubeSinuosidalRadiusGenerator<TPixel>
::CircularBarStraightTubeSinuosidalRadiusGenerator() :
  m_MinRadius( 1.0 ),
  m_SemiPeriod( 20 )
{
  
}


template <class TPixel>
typename CircularBarStraightTubeSinuosidalRadiusGenerator<TPixel>::ImagePointer
CircularBarStraightTubeSinuosidalRadiusGenerator<TPixel>::Create()
{
  ImagePointer tubeImage = ImageType::New();
  
  typename ImageType::SpacingType spacing;
  spacing[0] = this->m_ImageSpacing;
  spacing[1] = this->m_ImageSpacing;
  spacing[2] = 1.0; // !!! CURRENTLY A FIXED VALUE
  
  tubeImage->SetSpacing( spacing );
  
  typename ImageType::PointType origin;
  origin.Fill( 0.0 );
  
  tubeImage->SetOrigin( origin );
  
  typename ImageType::RegionType::SizeType size;
  size[0] = this->m_SectionImageSize[0];
  size[1] = this->m_SectionImageSize[1];
  size[2] = this->m_Height;
  
  typename ImageType::RegionType::IndexType index;
  index.Fill(0);
  
  typename ImageType::RegionType region;
  region.SetSize( size );
  region.SetIndex( index );
  
  tubeImage->SetRegions( region );
  
  tubeImage->Allocate();
  tubeImage->FillBuffer(0);
  
  // Create Bar section
  
  typedef CircularBarSectionGenerator<TPixel>    SectionGeneratorType;
  typedef typename SectionGeneratorType::ImageType    SectionImageType;
    
  SectionGeneratorType sectionGenerator;
  sectionGenerator.SetImageSize( this->m_SectionImageSize[0] ); // !!!
  sectionGenerator.SetImageSpacing( this->m_ImageSpacing );
  //sectionGenerator.SetSigma( this->m_Sigma );
  sectionGenerator.SetMaxValue( this->m_MaxValue );
  
  typename SectionImageType::Pointer sectionImage;
  
  typedef itk::ImageRegionConstIterator<SectionImageType>  ConstIteratorType;
  typedef itk::ImageRegionIterator<ImageType>              IteratorType;
  
  size[2] = 1; // copy a single slice each time
  region.SetSize( size );
  
  double maxRadius = this->m_Radius;
  double minRadius = this->m_MinRadius;
  
  if( maxRadius < minRadius )
  {
    maxRadius = this->m_MinRadius;
    minRadius = this->m_Radius; 
  }
  
  for( unsigned long z=0; z < this->m_Height; ++z )
  {
    // Set the radius for the current point
    
    double sinPos = ( z * this->m_ImageSpacing ) / this->m_SemiPeriod;
    sinPos = sinPos - floor( sinPos );
    double angle = vnl_math::pi * sinPos;         
    double sinValue = minRadius + ( maxRadius - minRadius ) * sin( angle );
    
    sectionGenerator.SetRadius( sinValue );
        
    sectionImage = sectionGenerator.Create();

    ConstIteratorType inputIt( sectionImage, sectionImage->GetRequestedRegion() );
    IteratorType outputIt;
    
    index[2] = z; // current slice
    region.SetIndex( index );
    
    outputIt = IteratorType( tubeImage, region );
    
    inputIt.GoToBegin();
    outputIt.GoToBegin();
    
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

#endif // __ivanCircularBarStraightTubeSinuosidalRadiusGenerator_h_

