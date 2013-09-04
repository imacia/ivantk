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
// File: ivanRadialContour.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanRadialContour_h
#define __ivanRadialContour_h

#include "itkObject.h"

#include "ivanAngularValue.h"


namespace ivan
{
  
  
/** \class RadialContour
 *  \brief Base class for describing an ND contour from a center point.
 *
 * This class is used to describe radial contours, that is contours that
 * are obtained by ray-casting from a center point. Contours are defined
 * by their point types, that may be objects that store coordinates
 * such as itk::Point, itk::Index, itk::Array. The contour may also be
 * described by simple floating point values or indexes that represent
 * the distance from the center. The point type may use continous or discrete 
 * coordinates and needs to be created on the stack. Subclasses define
 * how the contours are stored depending if the sampling is uniform or not
 * since in the second case we need to store the angle for each location.
 *
 * This class is useful for example to describe sections obtained from 3D images. 
 * In this case the dimension would be 3, since the points/indexes are 3D, 
 * but it may represent a 2D contour. The definition of the contour, makes
 * it suitable to describe 3D contours from a center point not lying on
 * a plane, but this is not the common case.
 *
 * \ingroup 
 */

template <class TPoint, typename TAngle=AngularValueInRadians<double> >
class ITK_EXPORT RadialContour : public itk::Object
{
public:

  typedef RadialContour                  Self;
  typedef itk::Object                    Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
    
  /** Point type for describing the contour. This could be an itk::Point, an Index, 
    * an Array, a double or integer describing the distance to the center... */
  typedef TPoint                      PointType;
  typedef TAngle                      AngleType;
  typedef typename TAngle::ValueType  AngleValueType;
  
  typedef unsigned int                ContourIdentifier;
  
public:
  
  itkNewMacro( RadialContour );
  itkTypeMacro( RadialContour, itk::Object );
  
  itkSetMacro( ContourId, ContourIdentifier );
  itkGetConstMacro( ContourId, ContourIdentifier );
  
  /** Return the point at the given container index. */
  virtual PointType & GetPoint( ContourIdentifier id ) = 0;
  virtual const PointType & GetPoint( ContourIdentifier id ) const = 0;
  
  /** Return the closest point in the contour given the angle. */
  virtual PointType & GetPointByAngle( AngleType id ) = 0;
  virtual const PointType & GetPointByAngle( AngleType id ) const = 0;

protected:
  
  RadialContour();
  ~RadialContour();
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;
    
protected:
  
  /** An identifier assigned to the contour. */
  unsigned int      m_ContourId;  
};

} // end namespace Vessel

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanRadialContour.hxx"
#endif

#endif
