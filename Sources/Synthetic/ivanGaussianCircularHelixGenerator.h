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
// File: ivanCircularGaussianHelixGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a circular helix with Gaussian section of the given size, stddev and peak intensity value
// Date: 2010/08/24


#ifndef __ivanCircularGaussianHelixGenerator_h_
#define __ivanCircularGaussianHelixGenerator_h_

#include "ivanCircularGaussianToroidGenerator.h"
#include "ivanCircularHelixCenterlineGenerator.h"

#include "itkGaussianSpatialFunction.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkVectorContainer.h"

namespace ivan
{

template <class TPixel>
class CircularGaussianHelixGenerator :
  public CircularGaussianToroidGenerator<TPixel>
{
public:
  
  typedef CircularGaussianHelixGenerator<TPixel>   Self;
  typedef CircularGaussianToroidGenerator<TPixel>  Superclass;

  typedef TPixel                             PixelType;
  typedef itk::Image<PixelType,3>            ImageType;
  typedef typename Superclass::ImagePointer  ImagePointer;
  typedef typename Superclass::PointType     PointType;
  typedef typename Superclass::IndexType     IndexType;
    
  typedef typename Superclass::PointContainerType        PointContainerType;
  typedef typename Superclass::PointContainerPointer     PointContainerPointer;
    
  typedef typename Superclass::PointType  CenterPointType;
    
public:

  CircularGaussianHelixGenerator();
  ~CircularGaussianHelixGenerator() {};
  
  void SetZeroAngleStartsAtBottom( bool atBottom )
    { m_ZeroAngleStartsAtBottom = atBottom; }
  bool GetZeroAngleStartsAtBottom() const
    { return m_ZeroAngleStartsAtBottom; }

  /** Set the pitch of the helix, that is, what the helix adivances in axial direction per turn (360� = 2pi rad). */
  void SetUnitPitch( double pitch )
    { m_UnitPitch = pitch; }
  double GetUnitPitch() const
    { return m_UnitPitch; }
  
  virtual double GetTotalHeight() const
    { return ( 8.0 * this->m_Sigma + this->GetAngleInterval() * this->GetUnitPitch() ); }
    
protected:
  
  /** Create the toroid centerline with the given center point. */
  virtual void CreateCenterline();
  
  /** Computes where the center of the toroid should be located depending on the calculated image area. */
  virtual void ComputeCenter( const ImageType *image );
    
protected:

  /** Choose whether to start the zero angle of the helix at the bottom or at the center of the image.
      The second may be useful when the helix starts at negative angles. */
  bool        m_ZeroAngleStartsAtBottom;

  /** Set the unit step. This is what the vertical distance between points per turn of the helix. */
  double      m_UnitPitch;  
};


template <class TPixel>
CircularGaussianHelixGenerator<TPixel>::CircularGaussianHelixGenerator() :
  m_UnitPitch( 100.0 ),
  m_ZeroAngleStartsAtBottom( true )
{
  
}


template <class TPixel>
void
CircularGaussianHelixGenerator<TPixel>::CreateCenterline()
{
  CircularHelixCenterlineGenerator<> generator;
  generator.SetDirection( this->m_Direction );
  generator.SetRadius( this->m_Radius );
  generator.SetCenter( this->m_Center );
  generator.SetStartAngle( this->m_StartAngle );
  generator.SetEndAngle( this->m_EndAngle );
  generator.SetUnitPitch( this->m_UnitPitch );
  
  if( this->m_AutoComputeNumberOfPoints || this->m_NumberOfPoints == 0 )
  {
    // We choose two times the length divided by the spacing
    // The length is abs( startAngle - endAngle ) * pi
    this->m_NumberOfPoints = 2 * static_cast<unsigned int>( vcl_floor
      ( vcl_abs( this->m_EndAngle - this->m_StartAngle ) * this->m_Radius / this->m_ImageSpacing ) );
  }
  
  generator.SetNumberOfPoints( this->m_NumberOfPoints );
  
  this->m_Centerline = generator.Create();
}


template <class TPixel>
void CircularGaussianHelixGenerator<TPixel>::ComputeCenter( const ImageType *image )
{
  Superclass::ComputeCenter( image );
    
  if( this->m_ZeroAngleStartsAtBottom )
    this->m_Center[this->m_Direction] = 4.0 * this->m_Sigma;
  else  
    this->m_Center[this->m_Direction] = 0.5 *
      image->GetLargestPossibleRegion().GetSize()[this->m_Direction] * image->GetSpacing()[this->m_Direction];
}

} // end namespace ivan

#endif // __CircularGaussianHelixGenerator_h_
