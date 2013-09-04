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
// File: ivanCircularGaussianSectionGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a Gaussian image of the given size, stddev and peak intensity value
// Date: 2010/09/17


#ifndef __ivanCircularGaussianSectionGenerator_h_
#define __ivanCircularGaussianSectionGenerator_h_

#include "itkImage.h"
#include "itkDiscreteGaussianImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"


namespace ivan
{

template <class TPixel>
class CircularGaussianSectionGenerator
{
public:

  typedef TPixel                        PixelType;
  typedef itk::Image<PixelType,2>       ImageType;
  typedef typename ImageType::Pointer   ImagePointer;

public:

  CircularGaussianSectionGenerator();
  ~CircularGaussianSectionGenerator() {};
  
  void SetImageSize( unsigned long size )
    { m_ImageSize = size; }
  unsigned long GetImageSize() const
    { return m_ImageSize; }
    
  void SetImageSpacing( double spacing )
    { m_ImageSpacing = spacing; }
  double GetImageSpacing() const
    { return m_ImageSpacing; }
  
  void SetSigma( double sigma )
    { m_Sigma = sigma; }
  double GetSigma() const
    { return m_Sigma; }
    
  void SetRescale( bool rescale )
    { m_Rescale = rescale; }
  void RescaleOn()
    { this->SetRescale( true ); }
  void RescaleOff()
    { this->SetRescale( false ); }
  bool GetRescale() const
    { return m_Rescale; }
    
  void SetNormalize( bool normalize )
    { m_Normalize = normalize; }
  void NormalizeOn()
    { this->SetNormalize( true ); }
  void NormalizeOff()
    { this->SetNormalize( false ); }
  bool GetNormalize() const
    { return m_Normalize; }

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
  double          m_Sigma;
  
  /** Normalize by dividing by sqrt(2*pi)*sigma. This is useful for theoretical studies. If normalization
    * is applied then rescaling does not take effect. */
  bool            m_Normalize;
  
  /** Rescale to desired max value. */
  bool            m_Rescale;
  
  /** Maximum intensity value under rescaling. Default value is 255. */
  PixelType       m_MaxValue;
};


template <class TPixel>
CircularGaussianSectionGenerator<TPixel>::CircularGaussianSectionGenerator() :
  m_ImageSpacing( 1.0 ),
  m_ImageSize( 50 ),
  m_Sigma( 2.0 ),
  m_Normalize( true ),
  m_Rescale( true ),
  m_MaxValue( 255.0 )
{

}


template <class TPixel>
typename CircularGaussianSectionGenerator<TPixel>::ImagePointer
CircularGaussianSectionGenerator<TPixel>::Create()
{
  typedef double   RealPixelType;
  typedef itk::Image<RealPixelType, 2>              RealImageType;
  /*typedef itk::GaussianImageSource<RealImageType>   GaussianSourceType;
  
  GaussianSourceType::Pointer gaussianSource = GaussianSourceType::New();
    
  GaussianSourceType::SpacingType spacing;
  spacing[0] = this->m_ImageSpacing;
  spacing[1] = this->m_ImageSpacing;
  
  gaussianSource->SetSpacing( spacing );
  
  GaussianSourceType::SizeType size;
  size[0] = this->m_ImageSize;
  size[1] = this->m_ImageSize;

  gaussianSource->SetSize( size );
  gaussianSource->SetSpacing( spacing );
  gaussianSource->SetScale( 1.0 );
  gaussianSource->SetNormalized( true );

  GaussianSourceType::ArrayType sigma;
  sigma.Fill( this->m_Sigma );
  gaussianSource->SetSigma( sigma );

  GaussianSourceType::ArrayType mean;
  mean[0] = 0.5 * ( size[0] - 1 ) * spacing[0];
  mean[1] = 0.5 * ( size[1] - 1 ) * spacing[1];
  gaussianSource->SetMean( mean );
  
  try
  {
    gaussianSource->Update();
  }
  catch ( itk::ExceptionObject &err )
  {
    std::cout << "ExceptionObject caught !" << std::endl; 
    std::cout << err << std::endl; 
    return 0;
  }*/

  /*typedef itk::ImageRegionIterator<RealImageType> IteratorType;
  IteratorType it( gaussianSource->GetOutput(), gaussianSource->GetOutput()->GetLargestPossibleRegion() );

  it.GoToBegin();

  while( !it.IsAtEnd() )
  {
    std::cout << "Index: " << it.GetIndex() << " Value: " << it.Get() << std::endl;
    ++it;
  }*/
  
  RealImageType::Pointer pulseImage = RealImageType::New();
    
  RealImageType::SpacingType spacing;
  spacing.Fill( this->m_ImageSpacing );
  pulseImage->SetSpacing( spacing );
  
  RealImageType::RegionType region;
    
  RealImageType::RegionType::SizeType size;    
  size.Fill( this->m_ImageSize );
  region.SetSize( size );
  
  RealImageType::RegionType::IndexType index;    
  index.Fill(0);
  region.SetIndex( index );
  
  pulseImage->SetRegions( region );
  pulseImage->Allocate();
  pulseImage->FillBuffer(0);
  
  // Set the central pixel to 1 (pulse)
  RealImageType::IndexType centralIndex;
  centralIndex.Fill( ( this->m_ImageSize - 1 ) / 2 );
  
  double centerValue = 1.0;
  
  pulseImage->SetPixel( centralIndex, 1 );
      
  ImagePointer gaussianImage;
  
  if( this->m_Rescale )
  {
    typedef itk::DiscreteGaussianImageFilter<RealImageType,RealImageType>  GaussianFilterType;
    
    GaussianFilterType::Pointer gaussian = GaussianFilterType::New();
    gaussian->SetInput( pulseImage );
    gaussian->SetVariance( this->m_Sigma * this->m_Sigma );

    try
    {
      gaussian->Update();
    }
    catch ( itk::ExceptionObject &err )
    {
      std::cout << "ExceptionObject caught !" << std::endl; 
      std::cout << err << std::endl; 
      return 0;
    }
    
    RealImageType::Pointer gaussianOutput = gaussian->GetOutput();
    gaussianOutput->DisconnectPipeline();
    pulseImage = 0; // free some memory

    typedef itk::RescaleIntensityImageFilter<RealImageType, ImageType>  RescaleFilterType;
    
    RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
    rescaler->SetInput( gaussianOutput );
    rescaler->SetOutputMinimum( 0 );
    rescaler->SetOutputMaximum( this->m_MaxValue );
  
    try
    {
      rescaler->Update();
    }
    catch ( itk::ExceptionObject &err)
    {
      std::cout << "ExceptionObject caught !" << std::endl; 
      std::cout << err << std::endl; 
      return 0;
    }
    
    gaussianImage = rescaler->GetOutput();
  }
  else
  {
    // !!! BE CAREFUL. FLOAT PIXEL TYPES SHOULD BE USED ON OUTPUT FOR THIS
    typedef itk::DiscreteGaussianImageFilter<RealImageType,ImageType>  GaussianFilterType;
    
    GaussianFilterType::Pointer gaussian = GaussianFilterType::New();
    gaussian->SetInput( pulseImage );
    gaussian->SetVariance( this->m_Sigma * this->m_Sigma );

    try
    {
      gaussian->Update();
    }
    catch ( itk::ExceptionObject &err )
    {
      std::cout << "ExceptionObject caught !" << std::endl; 
      std::cout << err << std::endl; 
      return 0;
    }

    gaussianImage = gaussian->GetOutput();
  }
  
  gaussianImage->DisconnectPipeline();  
  
  return gaussianImage;
    
  
  /*if( this->m_Normalize || !this->m_Rescale )
  {
    // Simply return the output of the Gaussian as it is

    typedef itk::CastImageFilter<RealImageType,ImageType>  CastFilterType;
    
    CastFilterType::Pointer caster = CastFilterType::New();
    caster->SetInput( gaussianSource->GetOutput() );

    try
    {
      caster->Update();
    }
    catch ( itk::ExceptionObject &err )
    {
      std::cout << "ExceptionObject caught !" << std::endl; 
      std::cout << err << std::endl; 
      return 0;
    }
    
    ImagePointer gaussianImage = caster->GetOutput();
    gaussianImage->DisconnectPipeline();
  
    return gaussianImage;    
  }

  typedef itk::RescaleIntensityImageFilter<RealImageType, ImageType>  RescaleFilterType;
  
  RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
  rescaler->SetInput( gaussianSource->GetOutput() );
  rescaler->SetOutputMinimum( 0 );
  rescaler->SetOutputMaximum( this->m_MaxValue );

  try
  {
    rescaler->Update();
  }
  catch ( itk::ExceptionObject &err)
  {
    std::cout << "ExceptionObject caught !" << std::endl; 
    std::cout << err << std::endl; 
    return 0;
  }
  
  ImagePointer gaussianImage = rescaler->GetOutput();
  gaussianImage->DisconnectPipeline();
  */
  
  return gaussianImage;    
}

} // end namespace ivan

#endif // __ivanCircularGaussianSectionGenerator_h_

