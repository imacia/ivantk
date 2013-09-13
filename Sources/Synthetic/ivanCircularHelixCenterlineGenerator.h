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
// File: ivanCircularHelixCenterlineGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: class for generating a simple circular helix in the given position and normal to the given plane
// (for now one of the three orthogonal planes). A circular helix has constant radius and pitch. This can be useful 
// for example for creating a volumetric circular helix.
// Date: 2010/08/24


#ifndef __ivanHelixCenterlineGenerator_h_
#define __ivanHelixCenterlineGenerator_h_

#include "ivanCenterlineGenerator.h"

#include "itkPoint.h"
#include "itkVectorContainer.h"

#include <cassert>

namespace ivan
{

template <class TCoordRep=double, unsigned int VDimension=3>
class CircularHelixCenterlineGenerator : public CircularCenterlineGenerator<TCoordRep,VDimension>
{
public:
  
  typedef CircularHelixCenterlineGenerator<TCoordRep,VDimension>  Self;
  typedef CircularCenterlineGenerator<TCoordRep,VDimension>       Superclass;

  typedef TCoordRep                               CoordRepType;
  typedef typename Superclass::PointType          PointType;
    
  typedef typename Superclass::CenterlineType     CenterlineType;
  typedef typename Superclass::CenterlinePointer  CenterlinePointer;
  
public:

  CircularHelixCenterlineGenerator();
  ~CircularHelixCenterlineGenerator() {};
  
  /** Set the unit pitch of the helix, that is, what the helix adivances in axial direction per radian. */
  void SetUnitPitch( double pitch )
    { m_UnitPitch = pitch; }
  double GetUnitPitch() const
    { return m_UnitPitch; }
    
  /** Set the pitch of the helix, that is, what the helix adivances in axial direction per turn (360� = 2pi rad). */
  void SetPitch( double pitch )
    { m_UnitPitch = 0.5 * pitch / vnl_math::pi; }
  
  virtual CenterlinePointer Create();

protected:
  
  /** Set the unit step. This is what the vertical distance between points per turn of the helix. */
  double    m_UnitPitch;
};


template <class TComponent, unsigned int VDimension>
CircularHelixCenterlineGenerator<TComponent,VDimension>::CircularHelixCenterlineGenerator() :
  m_UnitPitch( 100.0 )
{
  
}


template <class TComponent, unsigned int VDimension>
typename CircularHelixCenterlineGenerator<TComponent,VDimension>::CenterlinePointer
CircularHelixCenterlineGenerator<TComponent,VDimension>::Create()
{
  CenterlinePointer centerline = CenterlineType::New();
  
  double angle = this->m_StartAngle;
  double angleInc = ( this->m_EndAngle - this->m_StartAngle ) / (double)this->m_NumberOfPoints;
  
  PointType currentPoint;
  
  unsigned int circleXDir = ( this->m_Direction + 1 ) % 3;
  unsigned int circleYDir = ( this->m_Direction + 2 ) % 3;
  
  centerline->resize( this->m_NumberOfPoints );

  for( unsigned long i=0; i < this->m_NumberOfPoints; ++i, angle += angleInc )  
  {
    currentPoint[circleXDir] = this->m_Center[circleXDir] + this->m_Radius * cos( angle );
    currentPoint[circleYDir] = this->m_Center[circleYDir] + this->m_Radius * sin( angle ); 
    currentPoint[this->m_Direction] = this->m_Center[this->m_Direction] + this->m_UnitPitch * angle;
    
    (*centerline)[i] = currentPoint;
  }
  
  return centerline;
}

} // end namespace ivan

#endif // __CircularCenterlineGenerator_h_

