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
// File: ivanCircularCenterlineGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: class for generating a simple circular centerline in the given plane and position.
// This can be useful for example for creating a toroid.
// Date: 2010/06/16


#ifndef __ivanCircularCenterlineGenerator_h_
#define __ivanCircularCenterlineGenerator_h_

#include "ivanCenterlineGenerator.h"

#include "itkPoint.h"
#include "itkVectorContainer.h"

#include <cassert>

namespace ivan
{

template <class TCoordRep=double, unsigned int VDimension=3>
class CircularCenterlineGenerator : public CenterlineGenerator<TCoordRep,VDimension>
{
public:
  
  typedef CircularCenterlineGenerator<TCoordRep,VDimension>   Self;
  typedef CenterlineGenerator<TCoordRep,VDimension>           Superclass;

  typedef TCoordRep                        CoordRepType;
  typedef typename Superclass::PointType   PointType;

  typedef typename Superclass::CenterlineType     CenterlineType;
  typedef typename Superclass::CenterlinePointer  CenterlinePointer;
  
public:

  CircularCenterlineGenerator();
  ~CircularCenterlineGenerator() {};
  
  /** Set the first (top) point of the centerline. This point has the maximum X offset. */
  void SetCenter( const PointType & center )
    { m_Center = center; }
  const PointType & GetCenter() const
    { return m_Center; }
    
  void SetDirection( unsigned int direction )
    { m_Direction = direction; }
  unsigned int GetDirection() const
    { return m_Direction; }
    
  void SetStartAngle( double angle )
    { this->m_StartAngle = angle; }
  double GetStartAngle() const
    { return this->m_StartAngle; }
    
  void SetEndAngle( double angle )
    { this->m_EndAngle = angle; }
  double GetEndAngle() const
    { return this->m_EndAngle; }
  
  void SetNumberOfPoints( unsigned long numberOfPoints )
  {
    assert( numberOfPoints > 1 );
    m_NumberOfPoints = numberOfPoints;
    if( m_NumberOfPoints % 2 == 0 )
      m_NumberOfPoints += 1; // make sure it is odd  
  }
  unsigned long GetNumberOfPoints() const
    { return m_NumberOfPoints; }
  
  /** Set the radius of the circle. */
  void SetRadius( double radius )
    { m_Radius = radius; }
  double GetRadius() const
    { return m_Radius; }
  
  virtual CenterlinePointer Create();

protected:
  
  /** Position of the center of the circle. */
  PointType       m_Center;
  
  /** Direction normal to the centerline. 0 = X, 1 = Y, 2 = Z */
  unsigned int    m_Direction;
  
  /** Radius of the circle. */
  double          m_Radius;
  
  /** Set the number of points that determines the sampling distance between centerline points. */
  unsigned long   m_NumberOfPoints;
  
  /** Start angle in radians. Default is 0.0. */
  double          m_StartAngle;
  
  /** End angle in radians. Default is 2*pi. */
  double          m_EndAngle;
};


template <class TComponent, unsigned int VDimension>
CircularCenterlineGenerator<TComponent,VDimension>::CircularCenterlineGenerator() :
  m_NumberOfPoints( 50 ),
  m_Radius( 100.0 ),
  m_Direction(2),
  m_StartAngle(0),
  m_EndAngle( 2.0 * vnl_math::pi )
{
  m_Center.Fill(0.0);
}


template <class TComponent, unsigned int VDimension>
typename CircularCenterlineGenerator<TComponent,VDimension>::CenterlinePointer
CircularCenterlineGenerator<TComponent,VDimension>::Create()
{
  CenterlinePointer centerline = CenterlineType::New();
  
  double angle = this->m_StartAngle;
  double angleInc = ( this->m_EndAngle - this->m_StartAngle ) / (double)m_NumberOfPoints;
  
  PointType currentPoint;
  
  unsigned int circleXDir = ( this->m_Direction + 1 ) % 3;
  unsigned int circleYDir = ( this->m_Direction + 2 ) % 3;
  
  centerline->resize( this->m_NumberOfPoints );

  for( unsigned long i=0; i < this->m_NumberOfPoints; ++i, angle += angleInc )  
  {
    currentPoint[circleXDir] = this->m_Center[circleXDir] + this->m_Radius * cos( angle );
    currentPoint[circleYDir] = this->m_Center[circleYDir] + this->m_Radius * sin( angle ); 
    currentPoint[this->m_Direction] = m_Center[this->m_Direction];
    
    (*centerline)[i] = currentPoint;
  }
  
  return centerline;
}

} // end namespace ivan

#endif // __CircularCenterlineGenerator_h_

