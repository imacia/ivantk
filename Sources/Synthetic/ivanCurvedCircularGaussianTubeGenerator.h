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
// File: ivanCurvedCurvedCircularGaussianTubeGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a curved Gaussian tube of the given size, stddev and peak intensity value
// The tube is aligned with the Z-axis, and is normal to it at the middle slice. The maximum x
// deviation at the first and last slice must be specified. This indirectly defines the radius
// of curvature
// Date: 2010/09/30


#ifndef __ivanCurvedCircularGaussianTubeGenerator_h_
#define __ivanCurvedCircularGaussianTubeGenerator_h_

#include "ivanCircularGaussianTubeGenerator.h"
#include "ivanGaussianBallGenerator.h"

#include "itkImageRegionIterator.h"
#include "itkNeighborhoodIterator.h"
#include "itkImageFileWriter.h"


namespace ivan
{

template <class TPixel>
class CurvedCircularGaussianTubeGenerator : 
  public CircularGaussianTubeGenerator<TPixel>
{
public:

  typedef TPixel                         PixelType;
  typedef itk::Image<PixelType,3>        ImageType;
  typedef typename ImageType::Pointer    ImagePointer;
  typedef typename ImageType::PointType  PointType;
  typedef typename ImageType::IndexType  IndexType;

public:

  CurvedCircularGaussianTubeGenerator();
  ~CurvedCircularGaussianTubeGenerator() {};
  
  /** Set the maximum x offset. The radius of curvature is calculated accordingly. */
  void SetMaxXOffset( double maxXOffset );
  double GetMaxXOffset() const
    { return m_MaxXOffset; }  
    
  /** Set the radius of curvature. The maximum x offset is calculateda ccordingly. */
  void SetRadiusOfCurvature( double radiusOfCurvature );
  double GetRadiusOfCurvature() const
    { return m_RadiusOfCurvature; }  
  
  ImagePointer Create();

protected:

  void CopyGaussianBall( ImageType *destImage, ImageType *gaussianBall, 
    const PointType & ballCenter ) const;

protected: 
  
  /** Indirectly defines curvature. This is the x offset for the first and last
    * slice. The tube is normal to the z plane exactly at the middle. */
  double   m_MaxXOffset;
  
  /** Alternatively the radius of curvature can be provided directly. */
  double   m_RadiusOfCurvature;
};


template <class TPixel>
CurvedCircularGaussianTubeGenerator<TPixel>::CurvedCircularGaussianTubeGenerator() :
  m_MaxXOffset( 0.0 ),
  m_RadiusOfCurvature( 0.0 )
{
 
}

template <class TPixel>
void CurvedCircularGaussianTubeGenerator<TPixel>::SetRadiusOfCurvature( double radiusOfCurvature )
{
  // The maximum X offset, at H/2 from the middle, where H is the tube height, can be calculated as
  // Xmax = R - sqrt( R^2 - (H/2)^2 )
  
  this->m_RadiusOfCurvature = radiusOfCurvature;
  
  this->m_MaxXOffset = this->m_RadiusOfCurvature - sqrt( this->m_RadiusOfCurvature * 
    this->m_RadiusOfCurvature - 0.25 * this->m_ImageHeight * this->m_ImageHeight );
    
  if( this->m_MaxXOffset > this->m_ImageHeight * 0.5 )
  {
    std::cerr << "Maximum X offset should be at least half the tube height. "
      "Setting it to half the tube height" << std::endl;
    this->SetMaxXOffset( this->m_ImageHeight * 0.5 );
  }
}


template <class TPixel>
void CurvedCircularGaussianTubeGenerator<TPixel>::SetMaxXOffset( double maxXOffset )
{
  // Following the above formula, we can calculate the radius of curvature as
  // R = 0.5 * Xmax + 0.125 * H^2 / Xmax
  
  this->m_MaxXOffset = maxXOffset;
  
  if( this->m_MaxXOffset > this->m_Height * 0.5 )
  {
    std::cerr << "Maximum X offset should be at least half the tube height. "
      "Setting it to half the tube height" << std::endl; 
    this->m_MaxXOffset = this->m_Height * 0.5;
  }
  
  this->m_RadiusOfCurvature = 0.5 * this->m_MaxXOffset
    + 0.125 * this->m_Height * this->m_Height / this->m_MaxXOffset;  
}


template <class TPixel>
typename CurvedCircularGaussianTubeGenerator<TPixel>::ImagePointer
CurvedCircularGaussianTubeGenerator<TPixel>::Create()
{
  ImagePointer tubeImage = ImageType::New();
  
  typename ImageType::SpacingType spacing;
  spacing.Fill( this->m_ImageSpacing );
  
  tubeImage->SetSpacing( spacing );
  
  typename ImageType::PointType origin;
  origin.Fill( 0.0 );
  
  tubeImage->SetOrigin( origin );
  
  typename ImageType::RegionType::SizeType size;
  size[0] = this->m_SectionImageSize[0];
  size[1] = this->m_SectionImageSize[1];
  size[2] = (unsigned long) floor( ( this->m_Height / this->m_ImageSpacing ) + 0.5 );
  
  // Recalculate height to get an integer size in Z
  this->m_Height = (double)size[2] * this->m_ImageSpacing;  
  
  typename ImageType::RegionType::IndexType index;
  index.Fill(0);
  
  typename ImageType::RegionType region;
  region.SetSize( size );
  region.SetIndex( index );
  
  tubeImage->SetRegions( region );
  
  tubeImage->Allocate();
  tubeImage->FillBuffer(0);
  
  // Create Gaussian ball
  
  typedef GaussianBallGenerator<TPixel>                  GaussianBallGeneratorType;
  typedef typename GaussianBallGeneratorType::ImageType  GaussianBallImageType;
    
  GaussianBallGeneratorType ballGenerator;
  
  unsigned long ballImageSize = this->m_Sigma * 5.0 / this->m_ImageSpacing;
  
  if( ballImageSize % 2 == 0 )
    ballImageSize += 1;
    
  ballGenerator.SetImageSpacing( this->m_ImageSpacing );
  ballGenerator.SetImageSize( ballImageSize );
  ballGenerator.SetSigma( this->m_Sigma );
  ballGenerator.SetMaxValue( this->m_MaxValue );
  
  typename GaussianBallImageType::Pointer ballImage = ballGenerator.Create();
  
  /*typedef itk::ImageFileWriter<GaussianBallImageType> WriterType;
  typename WriterType::Pointer writer = WriterType::New();
  writer->SetInput( ballImage );
  writer->SetFileName( "ballImage.mhd" );
  writer->Update();*/
  
  typedef itk::ImageRegionConstIterator<GaussianBallImageType>  ConstIteratorType;
  typedef itk::ImageRegionIterator<ImageType>                   IteratorType;
  
  ConstIteratorType inputIt( ballImage, ballImage->GetRequestedRegion() );
  IteratorType outputIt;
  
  double currentHeight;
  double currentXOffset;
  
  PointType centerPoint;
    
  for( unsigned long z=0; z < size[2]; ++z )
  {
    index[2] = z; // current slice
    region.SetIndex( index );
    
    currentHeight = vnl_math_abs( (int)z - (int)( size[2]/2 ) ) * tubeImage->GetSpacing()[2];
    currentHeight = vnl_math_abs( currentHeight );
    currentXOffset = this->m_RadiusOfCurvature - sqrt( this->m_RadiusOfCurvature * 
      this->m_RadiusOfCurvature - currentHeight * currentHeight );
    currentXOffset -= this->m_MaxXOffset * 0.5; 
      //- ballImageSize * this->m_ImageSpacing * 0.5; // to center
      
    centerPoint[0] = 0.5 * size[0] * spacing[0] + currentXOffset;
    centerPoint[1] = 0.5 * size[1] * spacing[1];
    centerPoint[2] = z * spacing[2];
      
    this->CopyGaussianBall( tubeImage, ballImage, centerPoint );
    
    /*outputIt = IteratorType( tubeImage, region );
    
    inputIt.GoToBegin();
    outputIt.GoToBegin();
    
    while( !inputIt.IsAtEnd() )
    {
      outputIt.Set( inputIt.Get() );
      ++inputIt;
      ++outputIt;
    }*/
  }
  
  return tubeImage;    
}


template <class TPixel>
void 
CurvedCircularGaussianTubeGenerator<TPixel>
::CopyGaussianBall( ImageType *destImage, ImageType *gaussianBall, 
  const PointType & ballCenter ) const
{
  IndexType ballIndex;
  
  typedef itk::ImageRegionConstIterator<ImageType>  ConstIteratorType;
  typedef itk::NeighborhoodIterator<ImageType>      NeighborhoodIteratorType;
    
  unsigned int radius = 0.5 * gaussianBall->GetLargestPossibleRegion().GetSize()[0] 
    / gaussianBall->GetSpacing()[0];
    
  if( radius % 2 == 0 )
    radius += 1;
    
  typename NeighborhoodIteratorType::RadiusType ballRadius;
  ballRadius.Fill( gaussianBall->GetLargestPossibleRegion().GetSize()[0] / 2 );
    
  ConstIteratorType it( gaussianBall, gaussianBall->GetRequestedRegion() );
  NeighborhoodIteratorType neighIt( ballRadius, destImage, destImage->GetRequestedRegion() );
  typename NeighborhoodIteratorType::Iterator internalIt; 
  
  if( destImage->TransformPhysicalPointToIndex( ballCenter, ballIndex ) )
  {
    // This is the simple case of marking points
    //destImage->SetPixel( ballIndex, REPLACE_VALUE );
    
    neighIt.SetLocation( ballIndex );
    
    if( !neighIt.InBounds() )
      return;
    
    internalIt = neighIt.Begin();
    it.GoToBegin();      
    
    while( !it.IsAtEnd() )
    {
      if( *(*internalIt) < it.Get() )
        *(*internalIt) = it.Get();
      
      ++internalIt;
      ++it;
    }
  }  
}

} // end namespace ivan

#endif // __CurvedCircularGaussianTubeGenerator_h_

