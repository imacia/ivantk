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
// File: ivanSimpleCurvedCenterlineGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: class for generating a simple curve along the Z-axis. The curve will be normal to the
// Z-axis with a given radius of curvature or maximum X-offset. The y coordinates are constant
// Date: 2010/10/02


#ifndef __ivanSimpleCurvedCenterlineGenerator_h_
#define __ivanSimpleCurvedCenterlineGenerator_h_

#include "itkPoint.h"
#include "itkVectorContainer.h"

#include <cassert>

namespace ivan
{

template <class TComponent=double, unsigned int VDimension=3>
class SimpleCurvedCenterlineGenerator
{
public:
  
  typedef TComponent                          ComponentType;
  typedef itk::Point<TComponent, VDimension>  PointType;

  typedef itk::VectorContainer<unsigned int,PointType>  CenterlineType;
  typedef typename CenterlineType::Pointer              CenterlinePointer;
  
public:

  SimpleCurvedCenterlineGenerator();
  ~SimpleCurvedCenterlineGenerator() {};
  
  /** Set the first (top) point of the centerline. This point has the maximum X offset. */
  void SetOrigin( const PointType & origin )
    { m_Origin = origin; }
  const PointType & GetOrigin() const
    { return m_Origin; }  
  
  void SetNumberOfPoints( unsigned long numberOfPoints )
  {
    assert( numberOfPoints > 1 );
    m_NumberOfPoints = numberOfPoints;
    if( m_NumberOfPoints % 2 == 0 )
      m_NumberOfPoints += 1; // make sure it is odd  
  }
  unsigned long GetNumberOfPoints() const
    { return m_NumberOfPoints; }
  
  void SetHeight( double height )
    { m_Height = height; }
  double GetHeight() const
    { return m_Height; }
  
  /** Set the maximum x offset. The radius of curvature is calculated accordingly. */
  void SetMaxXOffset( double maxXOffset );
  double GetMaxXOffset() const
    { return m_MaxXOffset; }  
    
  /** Set the radius of curvature. The maximum x offset is calculateda ccordingly. */
  void SetRadiusOfCurvature( double radiusOfCurvature );
  double GetRadiusOfCurvature() const
    { return m_RadiusOfCurvature; }
    
  void SetRecenter( bool recenter )
    { m_Recenter = recenter; }
  void RecenterOn()
    { m_Recenter = true; }
  void RecenterOff()
    { m_Recenter = false; }
  bool GetRecenter() const
    { return m_Recenter; }
  
  virtual CenterlinePointer Create();

protected:
  
  /** Position of the first point. */
  PointType       m_Origin;
  
  /** Height of the centerline/tube. */
  double          m_Height;
  
  /** Set the number of points that determines the sampling distance between centerline points. */
  unsigned long   m_NumberOfPoints;
  
  /** Indirectly defines curvature. This is the x offset for the first and last
    * slice. The tube is normal to the z plane exactly at the middle. */
  double          m_MaxXOffset;
  
  /** Alternatively the radius of curvature can be provided directly. */
  double          m_RadiusOfCurvature;
  
  /** Center centerline in X coordinates. */
  bool            m_Recenter;
};



template <class TComponent, unsigned int VDimension>
SimpleCurvedCenterlineGenerator<TComponent,VDimension>::SimpleCurvedCenterlineGenerator() :
  m_Height( 100.0 ),
  m_NumberOfPoints( 50 ),
  m_MaxXOffset( 0.0 ),
  m_RadiusOfCurvature( 0.0 ),
  m_Recenter( false )
{
  m_Origin.Fill( 0.0 );
}


template <class TComponent, unsigned int VDimension>
void SimpleCurvedCenterlineGenerator<TComponent,VDimension>
::SetRadiusOfCurvature( double radiusOfCurvature )
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


template <class TComponent, unsigned int VDimension>
void SimpleCurvedCenterlineGenerator<TComponent,VDimension>
::SetMaxXOffset( double maxXOffset )
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


template <class TComponent, unsigned int VDimension>
typename SimpleCurvedCenterlineGenerator<TComponent,VDimension>::CenterlinePointer
SimpleCurvedCenterlineGenerator<TComponent,VDimension>::Create()
{
  CenterlinePointer centerline = CenterlineType::New();
  
  // We are describing a circunference. The center of the circumference is
  // xc = x0 + R - maxXOffset
  // yc = y0
  
  PointType centerPoint, currentPoint;
  if( !m_Recenter )
    centerPoint[0] = this->m_Origin[0] + this->m_RadiusOfCurvature - this->m_MaxXOffset;
  else
    centerPoint[0] = this->m_Origin[0] + this->m_RadiusOfCurvature;
  centerPoint[1] = this->m_Origin[1];
  centerPoint[2] = this->m_Origin[2] + 0.5 * this->m_Height;
  
  assert( this->m_NumberOfPoints > 1 );
  double phi = asin( 0.5 * this->m_Height / this->m_RadiusOfCurvature );
  double currentPhi = phi;
  double deltaPhi = 2.0 * phi / (double)( this->m_NumberOfPoints - 1 );
  double currentHeight = 0.0;  
    
  currentPoint[1] = this->m_Origin[1]; // fixed
    
  unsigned long idx = 0;

  while( vnl_math_abs( currentHeight - this->m_Height ) > 1e-2 )
  {
    currentPoint[0] = centerPoint[0] - this->m_RadiusOfCurvature * cos( currentPhi );
        
    if( idx < this->m_NumberOfPoints / 2 )
    {
      currentPoint[2] = centerPoint[2] + this->m_RadiusOfCurvature * sin( currentPhi );
      currentPhi -= deltaPhi;
    }
    else if( idx > this->m_NumberOfPoints / 2 )
    {
      currentPoint[2] = centerPoint[2] - this->m_RadiusOfCurvature * sin( currentPhi );
      currentPhi += deltaPhi;
    }
    else
    {
      currentPoint[2] = centerPoint[2];
      currentPhi = 0.0;
    }
    
    if( idx != 0 )
      currentHeight += vnl_math_abs( currentPoint[2] - centerline->at( idx-1 )[2] );

    centerline->push_back( currentPoint );

    ++idx;
  }
  
  return centerline;    
}

} // end namespace ivan

#endif // __ivanSimpleCurvedCenterlineGenerator_h_

