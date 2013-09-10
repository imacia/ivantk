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
// File: ivanVesselSection.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : generic node of an acyclic graph structure
// Date: 2009/02/06


#ifndef __ivanVesselSection_h
#define __ivanVesselSection_h

#include "itkObject.h"

#include "ivanNullVesselSectionStruct.h"


namespace ivan
{
  
/** \class VesselSection
 *  \brief Base class for VesselSection model.
 *
 * This base class uses three template parameters. The type may be any type that can describe 
 * the location of the center of the section, being it a geometrical point, an index, in the 
 * case of discrete centerlines, or even scalars in chain codes. VesselSection can be
 * instantiated, representing the most simple vessel section model that consists of a center
 * point and possibly metadata. In this case, the vessel section is merely used to describe
 * the centerline points.
 *
 * TCurveMetrics is a structure that represents the metrics for the curve at every point 
 * on the section. TSectionMetrics represents the metrics for the section itself. Both 
 * are incorporated as static structs. By default these template parameters use a dummy 
 * struct (NullVesselSectionStruct) which contain virtually nothing, in order to reduce 
 * required storage.
 *
 * \ingroup 
 */

template <class TCenterPoint, class TCurveMetrics = NullVesselSectionStruct, 
  class TSectionMetrics = NullVesselSectionStruct>
class ITK_EXPORT VesselSection : public itk::Object
{
public:

  /** Standard class typedefs. */
  typedef VesselSection                   Self;
  typedef itk::Object                     Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
  
  /** Type for the descriptor of the center point. */
  typedef TCenterPoint             CenterPointType;
  
  /** Type for the stored measurements. */
  typedef TCurveMetrics            CurveMetricsType;
  typedef TSectionMetrics          SectionMetricsType;
        
public:
  
  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselSection, itk::Object );
  
  CurveMetricsType & GetCurveMetrics()
    { return m_CurveMetrics; }
  const CurveMetricsType & GetCurveMetrics() const
    { return m_CurveMetrics; }
    
  SectionMetricsType & GetSectionMetrics()
    { return m_SectionMetrics; }
  const SectionMetricsType & GetSectionMetrics() const
    { return m_SectionMetrics; }
    
  itkSetMacro( Center, CenterPointType & );
  itkGetConstMacro( Center, const CenterPointType & );
    
protected:
  
  VesselSection() {}
  ~VesselSection() {}

  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const
    {
      Superclass::PrintSelf( os, indent );
      os << indent << "Center: " << m_Center << std::endl;
      //os << indent << "CurveMetrics: " << std::endl;
      //m_CurveMetrics.PrintSelf( os, indent );
      //os << indent << "SectionMetrics: " << std::endl;
      //m_SectionMetrics.PrintSelf( os, indent );
    }

private:

  VesselSection(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  /** This is the center point of the section. */
  CenterPointType     m_Center;
  
  /** Static storage for curve metrics. */
  CurveMetricsType    m_CurveMetrics;
    
  /** Static storage for section metrics. */
  SectionMetricsType  m_SectionMetrics;
};

} // end namespace ivan

#endif
