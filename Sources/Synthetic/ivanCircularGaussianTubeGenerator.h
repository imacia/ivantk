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
// File: ivanCircularGaussianTubeGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a generic Gaussian tube of the given size, stddev and peak intensity value.
//   Subclasses must define the shape of the tube.
// Date: 2010/09/17


#ifndef __ivanCircularGaussianTubeGenerator_h_
#define __ivanCircularGaussianTubeGenerator_h_

#include "ivanCircularGaussianSectionGenerator.h"

#include "itkImageRegionIterator.h"
#include "itkFixedArray.h"

namespace ivan
{

template <class TPixel>
class CircularGaussianTubeGenerator
{
public:

  typedef TPixel                        PixelType;
  typedef itk::Image<PixelType,3>       ImageType;
  typedef typename ImageType::Pointer   ImagePointer;
  
  typedef itk::FixedArray<unsigned long,2>   SectionSizeType;
  
public:

  CircularGaussianTubeGenerator();
  ~CircularGaussianTubeGenerator() {};
  
  void SetSectionImageSize( unsigned long sizeX, unsigned long sizeY )
    { m_SectionImageSize[0] = sizeX; m_SectionImageSize[1] = sizeY; }
  void SetSectionImageSize( unsigned long size )
    { m_SectionImageSize[0] = size; m_SectionImageSize[1] = size; }
  const SectionSizeType & GetSectionImageSize() const
    { return m_SectionImageSize; }
  
  void SetImageSpacing( double spacing )
    { m_ImageSpacing = spacing; }
  double GetImageSpacing() const
    { return m_ImageSpacing; }

  void SetSigma( double sigma )
    { m_Sigma = sigma; }
  double GetSigma() const
    { return m_Sigma; }
    
  /** This is the tube/shape height in pixels. The image height is the tube height plus 
    * twice the (Z)offset. */
  void SetHeight( unsigned long height )
    { m_Height = height; }
  unsigned long GetHeight() const
    { return m_Height; }
    
  /** Add an offset so we don't cut the tube at the Z boundaries. */
  void SetOffset( double offset )
    { m_Offset = offset; }
  double GetOffset() const
    { return m_Offset; }
  
  /** Set/Get rescale to the desired max value. Default is 255. Does not take effect when normalizing. */
  void SetRescale( bool rescale )
    { m_Rescale = rescale; }
  void RescaleOn()
    { this->SetRescale( true ); }
  void RescaleOff()
    { this->SetRescale( false ); }
  bool GetRescale() const
    { return m_Rescale; }
    
  /** Normalize by dividing by sqrt(2*pi)*sigma for the Gaussian section. */
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

  virtual ImagePointer Create() = 0;

protected: 
  
  /** Image size for the section (same for width and depth). */
  SectionSizeType  m_SectionImageSize;
  
  /** Image spacing. */
  double           m_ImageSpacing;
  
  /** Standard deviation of the Gaussian in both directions. */
  double           m_Sigma;
  
  /** Tube/shape height in pixels. Subclasses may use this as the height occupied by the tube/shape without
    * taking to account the (z-)offset. The total image height (in Z) would be m_Height + 2 * m_Offset. 
    * The interpretation of this height depends on the subclass. */
  unsigned long    m_Height;
  
  /** Offset used so we don't cut the shape at the (Z) boundaries. */
  double           m_Offset;
  
  /** Normalize by dividing by sqrt(2*pi)*sigma. This is useful for theoretical studies. If normalization
    * is applied then rescaling does not take effect. */
  bool             m_Normalize;
  
  /** Rescale to desired max value. */
  bool             m_Rescale;
    
  /** Maximum intensity value. */
  PixelType        m_MaxValue;
};


template <class TPixel>
CircularGaussianTubeGenerator<TPixel>::CircularGaussianTubeGenerator() :
  m_ImageSpacing( 1.0 ),
  m_Sigma( 2.0 ),
  m_Height( 100 ),
  m_Offset( 0.0 ),
  m_Normalize( true ),
  m_Rescale( true ),
  m_MaxValue( 255.0 )
{
  m_SectionImageSize.Fill( 50 );
}

} // end namespace ivan

#endif // __CircularGaussianTubeGenerator_h_

