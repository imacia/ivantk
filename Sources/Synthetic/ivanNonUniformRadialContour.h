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
// File: ivanNonUniformRadialContour.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanNonUniformRadialContour_h
#define __ivanNonUniformRadialContour_h

#include "ivanRadialContour.h"

#include <deque>

namespace ivan
{
  
/** \class NonUniformRadialContour
 *  \brief Describes an ND contour from a center point non-uniformly sampled.
 *
 * This class is used to describe uniformly sampled radial contours. Each contour 
 * location or element is described by a pair formed by the angle of the current
 * location and the point describing coordinates or distances.
 *
 * This is useful for example to describe sections obtained from 3D images. 
 * In this case the dimension would be 3, since the points/indexes are 3D, 
 * but it may represent a 2D contour. The definition of the contour, makes
 * it suitable to describe 3D contours from a center point not lying on
 * a plane, but this is not the common case.
 *
 * \ingroup 
 */

template <class TPoint, typename TAngle=AngularValueInRadians<double> >
class ITK_EXPORT NonUniformRadialContour
{
public:

  typedef NonUniformRadialContour   Self;
  typedef RadialContour             Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  /** Point type for describing the contour. This could be an itk::Point, an Index, 
    * an Array, a double or integer describing the distance to the center... */
  typedef TPoint                      PointType;
  typedef TAngle                      AngleType;
  typedef typename TAngle::ValueType  AngleValueType;
    
  typedef typename Superclass::ContourIdentifier   ContourIdentifier;
        
  /** A contour element is a pair representing both the angle and the point location. */
  typedef std::pair<AngleValueType,PointType>      ContourElementType;
  
  /** The deque container allows random access and constant time insertion of elements at front/back. */
  typedef std::deque<ContourElementType>           ContourContainerType;

public:
  
  itkNewMacro( NonUniformRadialContour );
  itkTypeMacro( NonUniformRadialContour, RadialContourBase );
  
  /** Return the point at the given container index. */
  virtual PointType & GetPoint( ContourIdentifier id )
    { return m_Elements[id]; }
  virtual const PointType & GetPoint( ContourIdentifier id ) const
    { return m_Elements[id]; }
  
  /** Return the closest point in the contour given the angle. */
  virtual PointType & GetPointByAngle( AngleType id );
  virtual const PointType & GetPointByAngle( AngleType id );
  
protected:
  
  NonUniformRadialContour();
  ~NonUniformRadialContour();
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;
    
protected:
  
  ContourContainerType   m_Elements;
};

} // end namespace Vessel

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanNonUniformRadialContour.hxx"
#endif

#endif
