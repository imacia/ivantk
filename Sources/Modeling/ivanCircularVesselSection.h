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
// File: ivanCircularVesselSection.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : generic node of an acyclic graph structure
// Date: 2009/02/06


#ifndef __ivanCircularVesselSection_h
#define __ivanCircularVesselSection_h

#include "ivanVesselSection.h"

#include "itkPoint.h"
#include "itkVector.h"
#include "itkIndex.h"


namespace ivan
{
  
/** \class CircularVesselSection
 *  \brief Simple circular VesselSection model with geometrical center point and normal.
 *
 * This is a simple point-based VesselSection model consisting of center point, radius and normal.
 * Subclasses may define more complication section models. The point type is in this case
 * and itk::Point describing the center.
 *
 * \ingroup 
 */

template <unsigned int VDimension=3, class TCurveMetrics = NullVesselSectionStruct, 
  class TSectionMetrics = NullVesselSectionStruct>
class ITK_EXPORT CircularVesselSection : public VesselSection< itk::Point<double,VDimension>,
  TCurveMetrics, TSectionMetrics>
{

public:

  /** Standard class typedefs. */
  typedef CircularVesselSection
    <VDimension,TCurveMetrics,TSectionMetrics>      Self;
  typedef VesselSection
    < itk::Point<double,VDimension>,
    TCurveMetrics, TSectionMetrics>                 Superclass;
  typedef itk::SmartPointer<Self>                   Pointer;
  typedef itk::SmartPointer<const Self>             ConstPointer;
  
  typedef TCurveMetrics            CurveMetricsType;
  typedef TSectionMetrics          SectionMetricsType;
  
  typedef double                   PointValueType;
  typedef itk::Point
    <PointValueType, VDimension>   PointType;
  typedef itk::Index<VDimension>   IndexType;
  typedef itk::Vector
    <PointValueType, VDimension>   VectorType;
    
  itkStaticConstMacro( Dimension, unsigned int, VDimension );
     
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( CircularVesselSection, VesselSection );
  
  itkSetMacro( Center, PointType & );
  itkGetConstMacro( Center, const PointType & );
  
  itkSetMacro( Normal, VectorType & );
  itkGetConstMacro( Normal, const VectorType & );
  
  itkSetMacro( Radius, double );
  itkGetConstMacro( Radius, double );
  
  itkSetMacro( Scale, double );
  itkGetConstMacro( Scale, double );
  
  itkSetMacro( Arclength, double );
  itkGetConstMacro( Arclength, double );
    
protected:
  
  CircularVesselSection();
  ~CircularVesselSection();
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  CircularVesselSection(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  /** This is the normal to the section which coincides with the tangent 
    * vector to the curve in the Frenet reference system. */
  VectorType  m_Normal;
  
  /** Average radius. */
  double      m_Radius;
  
  /** Scale at which features where calculated. This is the sigma for scale-space
    * derivative calculations and may not coincide with the radius. */
  double      m_Scale;
      
  /** Accumulated length from the beginning of the curve. */
  double      m_Arclength;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanCircularVesselSection.hxx"
#endif

#endif
