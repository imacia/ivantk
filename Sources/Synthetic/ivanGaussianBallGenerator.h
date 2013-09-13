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
// File: ivanGaussianBallGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a Gaussian ball of the given size, stddev and peak intensity value
// Date: 2010/09/17


#ifndef __ivanGaussianBallGenerator_h_
#define __ivanGaussianBallGenerator_h_

#include "itkImage.h"
#include "itkGaussianImageSource.h"
#include "itkRescaleIntensityImageFilter.h"

namespace ivan
{

template <class TPixel>
class GaussianBallGenerator
{
public:

  typedef TPixel                        PixelType;
  typedef itk::Image<PixelType,3>       ImageType;
  typedef typename ImageType::Pointer   ImagePointer;

public:

  GaussianBallGenerator();
  ~GaussianBallGenerator() {};

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
  
  /** Maximum intensity value. */
  PixelType       m_MaxValue;
};


template <class TPixel>
GaussianBallGenerator<TPixel>::GaussianBallGenerator() :
  m_ImageSize( 50 ),
  m_ImageSpacing( 1.0 ),
  m_Sigma( 2.0 ),
  m_MaxValue( 255.0 )
{
 
}


template <class TPixel>
typename GaussianBallGenerator<TPixel>::ImagePointer
GaussianBallGenerator<TPixel>::Create()
{
  typedef double   RealPixelType;
  typedef itk::Image<RealPixelType, 3>              RealImageType;
  typedef itk::GaussianImageSource<RealImageType>   GaussianSourceType;
  
  GaussianSourceType::Pointer gaussianSource = GaussianSourceType::New();
    
  GaussianSourceType::SpacingType spacing;
  spacing[0] = this->m_ImageSpacing;
  spacing[1] = this->m_ImageSpacing;
  spacing[2] = this->m_ImageSpacing;
  
  gaussianSource->SetSpacing( spacing );
  
  GaussianSourceType::SizeType size;
  size[0] = this->m_ImageSize;
  size[1] = this->m_ImageSize;
  size[2] = this->m_ImageSize;

  gaussianSource->SetSize( size );
  gaussianSource->SetScale( 1.0 );
  gaussianSource->SetNormalized( true ); // IMPORTANT

  GaussianSourceType::ArrayType sigma;
  sigma.Fill( this->m_Sigma );
  gaussianSource->SetSigma( sigma );

  GaussianSourceType::ArrayType mean;
  mean[0] = ( size[0] - 1 ) * spacing[0] / 2.0;
  mean[1] = ( size[1] - 1 ) * spacing[1] / 2.0;
  mean[2] = ( size[2] - 1 ) * spacing[2] / 2.0;
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
  }

  typedef itk::RescaleIntensityImageFilter<RealImageType, ImageType>  RescaleFilterType;
  
  typename RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
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
  
  return gaussianImage;    
}

} // end namespace ivan

#endif // __GaussianBallGenerator_h_

