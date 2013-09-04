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
// File: ivanGaussianCircularToroidGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a toroid with Gaussian section of the given size, stddev and peak intensity value
// Date: 2010/10/01


#ifndef __ivanGaussianCircularToroidGenerator_h_
#define __ivanGaussianCircularToroidGenerator_h_

#include "ivanCenterlineBasedGaussianTubeGenerator.h"
#include "ivanCircularCenterlineGenerator.h"

#include "itkGaussianSpatialFunction.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkVectorContainer.h"

namespace ivan
{

template <class TPixel>
class GaussianCircularToroidGenerator :
  public CenterlineBasedGaussianTubeGenerator<TPixel>
{
public:
  
  typedef GaussianCircularToroidGenerator<TPixel>        Self;
  typedef CenterlineBasedGaussianTubeGenerator<TPixel>   Superclass;

  typedef TPixel                             PixelType;
  typedef itk::Image<PixelType,3>            ImageType;
  typedef typename Superclass::ImagePointer  ImagePointer;
  typedef typename Superclass::PointType     PointType;
  typedef typename Superclass::IndexType     IndexType;
    
  typedef typename Superclass::PointContainerType      PointContainerType;
  typedef typename Superclass::PointContainerPointer   PointContainerPointer;
    
  typedef typename CircularCenterlineGenerator<>::PointType  CenterPointType;
  
  enum ImageAreaType
  {
    FullCircle, // 0 to 360�, toroid center on center
    TopHalf, // 0 to 180�, toroid center on bottom-center
    BottomHalf, // 180 to 360�, toroid center on top-center
    LeftHalf, // 90 to 270�, toroid center on center-right
    RightHalf, // 270 to 90�, toroid center on center-left
    FirstQuarter, // 0 to 90�, toroid center on bottom-left
    SecondQuarter, // 90 to 180�, toroid center on bottom-right
    ThirdQuarter, // 180 to 270�, toroid center on top-left
    FourthQuarter // 270 to 360�, toroid center on top-right
  };
  
public:

  GaussianCircularToroidGenerator();
  ~GaussianCircularToroidGenerator() {};
  
  void SetDirection( unsigned int direction )
    { 
      this->m_Direction = direction;
      this->m_Modified = true;
    }
  unsigned int GetDirection() const 
    { return this->m_Direction; }
    
  void SetRadius( double radius )
    { 
      this->m_Radius = radius;
      this->m_Modified = true;
    }
  double GetRadius() const 
    { return this->m_Radius; }
    
  void SetNumberOfPoints( unsigned long numPoints )
    { 
      this->m_NumberOfPoints = numPoints;
      this->m_Modified = true;
      this->m_AutoComputeNumberOfPoints = false;
    }
  unsigned long GetNumberOfPoints() const 
    { return this->m_NumberOfPoints; }
    
  void SetStartAngle( double angle )
    { 
      this->m_StartAngle = angle;
      this->ComputeAngleInterval();
    }
  double GetStartAngle() const
    { return this->m_StartAngle; }
    
  void SetEndAngle( double angle )
    { 
      this->m_EndAngle = angle;
      this->ComputeAngleInterval();
    }
  double GetEndAngle() const
    { return this->m_EndAngle; }
    
  double GetAngleInterval() const
    { return this->m_AngleInterval; }
    
  void SetAutoComputeNumberOfPoints( bool compute )
    { this->m_AutoComputeNumberOfPoints = compute; }
  bool GetAutoComputeNumberOfPoints() const
    { return this->m_AutoComputeNumberOfPoints; }
    
  void SetAlwaysImageAreaFullCircle( bool always )
    { this->m_AlwaysImageAreaFullCircle = always; }
  bool GetAlwaysImageAreaFullCircle() const
    { return this->m_AlwaysImageAreaFullCircle; }
    
  virtual double GetTotalWidth() const
    //{ return ( 2.0 * this->m_Radius + 6.0 * this->m_Sigma ); } // with this would be enough but we left here some more margin
    { return ( 2.0 * this->m_Radius + 8.0 * this->m_Sigma ); }
    
  virtual double GetTotalHeight() const
    //{ return ( 6.0 * this->m_Sigma ); } // with this would be enough but we left here some more margin
    { return ( 8.0 * this->m_Sigma ); }
    
  /** Get the toroid center on the image. This is only valid after calling Create(). */
  CenterPointType GetCenter() const
    { return m_Center; }
    
  virtual ImagePointer Create();
  
protected:
  
  /** Set point vector defining the centerline. Disabled in this implementation since the
    * centerline is created internally. */
  virtual void SetCenterline( PointContainerType *centerline ) {} // purposely empty
  
  /** Create and allocate the image with the appropiate dimensions, spacing... */
  virtual ImagePointer CreateEmptyImage();
  
  /** Create the toroid centerline with the given center point. */
  virtual void CreateCenterline();
  
  /** Computes the image area that minimizes the volume depending on the start and end angle. */
  void ComputeImageArea();
  
  void ComputeAngleInterval();
  
  /** Computes where the center of the toroid should be located depending on the calculated image area. */
  virtual void ComputeCenter( const ImageType *image );
    
protected:
  
  /** Direction normal to the centerline. 0 = X, 1 = Y, 2 = Z */
  unsigned int     m_Direction;
  
  /** Radius of the circle. */
  double           m_Radius;
  
  /** Set the number of points that determines the sampling distance between centerline points. */
  unsigned long    m_NumberOfPoints;
      
  /** Start angle in radians. Default is 0.0. */
  double           m_StartAngle;
  
  /** End angle in radians. Default is 2*pi. */
  double           m_EndAngle;
  
  /** The interval in radians. It can be larger than 2*pi. */
  double           m_AngleInterval;
  
  /** The area that the image will cover in the toroid centerline plane. */
  ImageAreaType    m_ImageArea;
  
  /** Toroid center on the volume. This is only valid after creation. */
  CenterPointType  m_Center;
  
  /** Internal flag that indicates whether we need to generate the centerline again. */
  bool             m_Modified;
  
  /** Automatically compute the number of points to cover the toroid area without gaps. */
  bool             m_AutoComputeNumberOfPoints;
  
  /** Force the image area to cover the full torus circle even if the angles don't. */
  bool             m_AlwaysImageAreaFullCircle;  
};


template <class TPixel>
GaussianCircularToroidGenerator<TPixel>::GaussianCircularToroidGenerator() :
  m_Direction(2),
  m_NumberOfPoints( 50 ),
  m_Radius( 100.0 ),
  m_StartAngle(0),
  m_EndAngle( 2.0 * vnl_math::pi ),
  m_AngleInterval( 2.0 * vnl_math::pi ),
  m_ImageArea( FullCircle ),
  m_Modified( true ),
  m_AutoComputeNumberOfPoints( false ),
  m_AlwaysImageAreaFullCircle( false )
{
  m_Center.Fill( 0.0 );
}


template <class TPixel>
typename GaussianCircularToroidGenerator<TPixel>::ImagePointer
GaussianCircularToroidGenerator<TPixel>::CreateEmptyImage()
{  
  ImagePointer tubeImage = ImageType::New();
  
  typename ImageType::SpacingType spacing;
  spacing.Fill( this->m_ImageSpacing );
  
  tubeImage->SetSpacing( spacing );
  
  typename ImageType::PointType origin;
  origin.Fill( 0.0 );
  
  tubeImage->SetOrigin( origin );
  
  typename ImageType::RegionType::SizeType size;
    
  unsigned int circleXDir = ( this->m_Direction + 1 ) % 3;
  unsigned int circleYDir = ( this->m_Direction + 2 ) % 3;
  
  double imageWidthX = this->GetTotalWidth() + 2.0 * this->m_Offset;
  double imageWidthY = imageWidthX;
  double imageHeight = this->GetTotalHeight() + 2.0 * this->m_Offset;
  
  if( this->m_AlwaysImageAreaFullCircle )
    this->m_ImageArea = FullCircle;
  else
    this->ComputeImageArea();
  
  switch( this->m_ImageArea )
  {
    case TopHalf:
    case BottomHalf:
      
      imageWidthY *= 0.5;
      break;
    
    case LeftHalf:
    case RightHalf:
      
      imageWidthX *= 0.5;
      break;
    
    case FirstQuarter:
    case SecondQuarter:
    case ThirdQuarter:
    case FourthQuarter:
    
      imageWidthX *= 0.5;
      imageWidthY *= 0.5;
      break;
          
    /*case FullCircle: // we don't need to do anything
    default:
      
      
      break;*/
  };
  
    
  // Note that after setting a fixed spacing and resolution the image height and width may change
  // slightly from these numbers. The new width and height is the spacing times the size in the
  // corresponding direction
      
  size[circleXDir] = (unsigned long)floor( ( imageWidthX / this->m_ImageSpacing ) + 0.5 );
  size[circleYDir] = (unsigned long)floor( ( imageWidthY / this->m_ImageSpacing ) + 0.5 );
  size[this->m_Direction] = (unsigned long) floor( ( imageHeight / this->m_ImageSpacing ) + 0.5 );
  
  // Create an image with an even number of pixels in each direction
  for( unsigned int i=0; i<3; ++i )
  {
    if( size[i] % 2 == 0 )
      ++size[i];
  }      
  
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
typename GaussianCircularToroidGenerator<TPixel>::ImagePointer
GaussianCircularToroidGenerator<TPixel>::Create()
{
  ImagePointer image = this->CreateEmptyImage();
    
  // Do this after having image dimensions etc. calculated
  if( m_Modified )    
  {
    this->ComputeCenter( image );
    this->CreateCenterline();
    m_Modified = false;
  }
  
  this->DrawFromCenterline( image );
    
  return image;
}


template <class TPixel>
void
GaussianCircularToroidGenerator<TPixel>::CreateCenterline()
{
  CircularCenterlineGenerator<> generator;
  generator.SetDirection( this->m_Direction );
  generator.SetRadius( this->m_Radius );
  generator.SetCenter( this->m_Center );
  generator.SetStartAngle( this->m_StartAngle );
  generator.SetEndAngle( this->m_EndAngle );
  
  if( this->m_AutoComputeNumberOfPoints || this->m_NumberOfPoints == 0 )
  {
    // We choose two times the length divided by the spacing
    // The length is abs( startAngle - endAngle ) * pi
    m_NumberOfPoints = 2 * static_cast<unsigned int>( vcl_floor
      ( vcl_abs( this->m_EndAngle - this->m_StartAngle ) * this->m_Radius / this->m_ImageSpacing ) );
  }
  
  generator.SetNumberOfPoints( this->m_NumberOfPoints );
  
  this->m_Centerline = generator.Create();
}


template <class TPixel>
void GaussianCircularToroidGenerator<TPixel>::ComputeImageArea()
{
  // We can reduce the x and y dimensions of the image if the angles don't cover 360�
  // For simplicity and in order to keep the center of the toroid inside the image we will consider
  // half and quarters in the toroid centerline plane.
  
  double startAngle = this->m_StartAngle;
  double endAngle = this->m_EndAngle;
  
  // For calculations we need all this angles positive and in the 0-360� range
  if( startAngle < 0.0 )
    startAngle = 2.0 * vnl_math::pi + startAngle;
  if( endAngle < 0.0 )
    endAngle = 2.0 * vnl_math::pi + endAngle;
      
  if( startAngle > 2.0 * vnl_math::pi )
  {
    double times = vcl_floor( startAngle / ( 2.0 * vnl_math::pi ) );
    startAngle -= times * 2.0 * vnl_math::pi;
  }
  if( endAngle > 2.0 * vnl_math::pi )
  {
    double times = vcl_floor( endAngle / ( 2.0 * vnl_math::pi ) );
    endAngle -= times * 2.0 * vnl_math::pi;
  }
  
  // First try with the quarters, then with the halves
  if( startAngle >= 0.0 && startAngle <= 0.5 * vnl_math::pi && 
           endAngle >= 0.0 && endAngle <= 0.5 * vnl_math::pi )
    this->m_ImageArea = FirstQuarter;
  else if( startAngle >= 0.5 * vnl_math::pi && startAngle <= vnl_math::pi && 
           endAngle >= 0.5 * vnl_math::pi && endAngle <= vnl_math::pi )
    this->m_ImageArea = SecondQuarter;
  else if( startAngle >= vnl_math::pi && startAngle <= 1.5 * vnl_math::pi && 
           endAngle >= vnl_math::pi && endAngle <= 1.5 * vnl_math::pi )
    this->m_ImageArea = ThirdQuarter;
  else if( startAngle >= 1.5 * vnl_math::pi && startAngle <= 2.0 * vnl_math::pi && 
           endAngle >= 1.5 * vnl_math::pi && endAngle <= 2.0 * vnl_math::pi )
    this->m_ImageArea = FourthQuarter;
  if( startAngle >= 0.0 && startAngle <= vnl_math::pi && 
      endAngle >= 0.0 && endAngle <= vnl_math::pi )
    this->m_ImageArea = TopHalf;
  else if( startAngle >= vnl_math::pi && startAngle <= 2.0 * vnl_math::pi && 
           endAngle >= vnl_math::pi && endAngle <= 2.0 * vnl_math::pi )
    this->m_ImageArea = BottomHalf;
  else if( startAngle >= 0.5 * vnl_math::pi && startAngle <= 1.5 * vnl_math::pi && 
           endAngle >= 0.5 * vnl_math::pi && endAngle <= 1.5 * vnl_math::pi )
    this->m_ImageArea = LeftHalf;
  else if( ( startAngle >= 1.5 * vnl_math::pi || startAngle <= 0.5 * vnl_math::pi ) && 
           ( endAngle >= 1.5 * vnl_math::pi || endAngle <= 0.5 * vnl_math::pi ) &&
           ( vcl_abs( this->m_StartAngle - this->m_EndAngle ) <= vnl_math::pi ) )
    this->m_ImageArea = RightHalf;  
  else
    this->m_ImageArea = FullCircle;  
}


template <class TPixel>
void GaussianCircularToroidGenerator<TPixel>::ComputeAngleInterval()
{
  if( this->m_StartAngle > this->m_EndAngle )
  {
    // We assume that the start angle should be negative in this case 
    // WARNING: not considering m_StartAngle over 2 PI for now!!!
    this->m_AngleInterval = this->m_EndAngle - ( 2.0 * vnl_math::pi - this->m_StartAngle );
  }
  else
    this->m_AngleInterval = this->m_EndAngle - this->m_StartAngle; // works also when m_StartAngle < 0  
}


template <class TPixel>
void GaussianCircularToroidGenerator<TPixel>::ComputeCenter( const ImageType *image )
{
  // Compute the factors with respect to the image dimensions (already calculated)
  double xfactor, yfactor;
  
  switch( this->m_ImageArea )
  {
    case TopHalf:    
      xfactor = 0.5;
      yfactor = 1.0;
      break;
    
    case BottomHalf:
      xfactor = 0.5;
      yfactor = 0.0;
      break;
      
    case LeftHalf:    
      xfactor = 1.0;
      yfactor = 0.5;
      break;
    
    case RightHalf:
      xfactor = 0.0;
      yfactor = 0.5;
      break;
    
    case FirstQuarter:
      xfactor = 0.0;
      yfactor = 1.0;
      break;
    
    case SecondQuarter:
      xfactor = 1.0;
      yfactor = 1.0;
      break;
      
    case ThirdQuarter:
      xfactor = 1.0;
      yfactor = 0.0;
      break;
      
    case FourthQuarter:
      xfactor = 0.0;
      yfactor = 0.0;
      break;
    
    case FullCircle:
    default:      
      xfactor = 0.5;
      yfactor = 0.5;
      break;
  };
  
  unsigned int circleXDir = ( this->m_Direction + 1 ) % 3;
  unsigned int circleYDir = ( this->m_Direction + 2 ) % 3;
  
  this->m_Center[circleXDir] = xfactor * 
    image->GetLargestPossibleRegion().GetSize()[circleXDir] * image->GetSpacing()[circleXDir];
  this->m_Center[circleYDir] = yfactor * 
    image->GetLargestPossibleRegion().GetSize()[circleYDir] * image->GetSpacing()[circleYDir];
  this->m_Center[this->m_Direction] = 0.5 *
    image->GetLargestPossibleRegion().GetSize()[this->m_Direction] * image->GetSpacing()[this->m_Direction];
}

} // end namespace ivan

#endif // __CenterlineBasedGaussianTubeGenerator_h_

