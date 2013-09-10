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
// File: ivanAorticAneurysmVesselSection.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : generic node of an acyclic graph structure
// Description : 
// Date: 2009/02/06


#ifndef __ivanAorticAneurysmVesselSection_h
#define __ivanAorticAneurysmVesselSection_h

#include "ivanVesselSection.h"
#include "ivanUniformRadialContour.h"


namespace ivan
{
  
/** \class AorticAneurysmVesselSection
 *  \brief Double contoured vessel section for describing (abdominal) aortic aneurysms.
 *
 * This is a special class of vessel section to describe mainly sections of Abdominal Aortic
 * Aneurysms (AAAs). In this case, the sections two contours, the internal contour, for the
 * lumen, and the external contour, for the thrombus.
 *
 * These vessel sections are also shared by more than one centerline, since in the iliac
 * arteries the aorta is bifurcated in two branches. During the vessel model generation
 * procedure, these sections must be added to both corresponding branches.
 *
 * \ingroup 
 */

template <class TRadialContour = UniformRadialContour, unsigned int VDimension=3, 
  class TCurveMetrics = NullVesselSectionStruct, class TSectionMetrics = NullVesselSectionStruct>
class ITK_EXPORT AorticAneurysmVesselSection : public CircularVesselSection
  <VDimension, TCurveMetrics, TSectionMetrics>
{

public:

  /** Standard class typedefs. */
  typedef AorticAneurysmVesselSection  Self;
  typedef CircularVesselSection           Superclass;
  typedef itk::SmartPointer<Self>           Pointer;
  typedef itk::SmartPointer<const Self>     ConstPointer;
  
  typedef TRadialContour                            RadialContourType;
  typedef typename RadialContourType::Pointer       RadialContourPointer;
  typedef typename RadialContourType::ConstPointer  RadialContourConstPointer;
  
  typedef TCurveMetrics     CurveMetricsType;
  typedef TSectionMetrics   SectionMetricsType;
  
  typedef typename Superclass::PointValueType  PointValueType;
  typedef typename Superclass::PointType       PointType;
  typedef typename Superclass::IndexType       IndexType;
  typedef typename Superclass::DirectionType   DirectionType;
    
  itkStaticConstMacro( Dimension, unsigned int, VDimension );
     
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( AorticAneurysmVesselSection, VesselSection );
  
  itkSetMacro( Center2, PointType & );
  itkGetConstMacro( Center2, const PointType & );
  
  itkSetMacro( AneurysmCenter, PointType & );
  itkGetConstMacro( AneurysmCenter, const PointType & );
  
  itkSetMacro( MaxAneurysmDiameter, double );
  itkGetConstMacro( MaxAneurysmDiameter, double );
  
  RadialContourPointer GetLumenContour()
    { return m_LumenContour; }
  RadialContourPointer GetLumenContour() const
    { return m_LumenContour; }
  
  RadialContourPointer GetThrombusContour()
    { return m_ThrombusContour; }
  RadialContourPointer GetThrombusContour() const
    { return m_ThrombusContour; }
  
protected:
  
  AorticAneurysmVesselSection();
  ~AorticAneurysmVesselSection();
    
  void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:

  AorticAneurysmVesselSection(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  /** This is the second center point of the section. This is used only when
    * the section has two centers in the iliac arteries. */
  PointType   m_Center2;
  
  /** This is the geometrical center of the aneurysm, used to approximately
    * estimate the diameter since it passes through it. */
  PointType   m_AneurysmCenter;
  
  /** Maximum radius of the aneurysm in the current section. */
  double      m_MaxAneurysmDiameter;
  
  /** Contour describing the lumen. */
  RadialContourPointer::Pointer   m_LumenContour;
  
  /** Contour describing the thrombus. */
  RadialContourPointer::Pointer   m_ThrombusContour;
};

} // end namespace Vessel

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanVesselSection.hxx"
#endif

#endif
