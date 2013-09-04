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
// File: ivanVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/05/28


#ifndef __ivanVesselSectionEstimator_h
#define __ivanVesselSectionEstimator_h

#include "ivanVesselSection.h"

#include "itkVectorContainer.h"
#include "itkFixedArray.h"


namespace ivan
{

  
template <class TCenterline, class TCalculator>
class VesselSectionEstimator;
  
template <class TCenterline, class TMetrics = NullVesselSectionStruct>
class VesselSectionMetricsCalculator : public itk::LightObject
{
  
public:

  typedef VesselSectionMetricsCalculator
    <TCenterline, TMetrics>              Self;
  typedef itk::LightObject               Superclass;
  
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef TMetrics                       MeasurementsType;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;
    
  typedef unsigned int                          RangeValueType;
  typedef itk::FixedArray<RangeValueType, 2>    RangeType;
  
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselSectionMetricsCalculator, itk::LightObject );
  
  virtual void SetCenterline( CenterlineType * centerline )
    { m_Centerline = centerline; }
  CenterlineType * GetCenterline()
    { return m_Centerline; }
  const CenterlineType * GetCenterline() const
    { return m_Centerline; }
    
  /** Range of sections to calculate. */
  void SetSectionRange( RangeValueType first, RangeValueType last );
  
  /** Calculate just for a section. */
  void SetSection( RangeValueType section );
  
  /** Set range of sections to calculate to all section. */
  void SetSectionRangeToAll();
  
  virtual void Compute() {}  

protected:

  VesselSectionMetricsCalculator();
  ~VesselSectionMetricsCalculator();
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;
  
private:

  VesselSectionMetricsCalculator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented
  
protected:
  
  /** Container of sections. */
  CenterlinePointer   m_Centerline;
  
  /** Range of sections to calculate. */
  RangeType           m_SectionRange;
};
 
  

/** \class VesselSectionEstimator
 *  \brief Estimates the vessel section and properties, once the center point is calculated.
 *
 * This class estimates a single or several vessel sectiona once an initial center point is set. 
 * This might be just the normal that defines the section plane or the complete section according 
 * to the model. This class is templated over the centeline type and the measurement calculator.
 * The whole centerline, which is in fact a section container in this model, needs to be provided, 
 * since some methods may need to interpolate betweens sections. The measurement calculator is 
 * used to calculate vessel related measurements of any type that are part of the section model 
 * and that are provided as a template parameter in that model. This allows storing/calculating 
 * several different section properties under the same basic section model, depending on the 
 * application.
 *
 * \ingroup 
 */

template <class TCenterline, class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT VesselSectionEstimator : public itk::Object
{

public:

  /** Standard class typedefs. */
  typedef VesselSectionEstimator
    <TCenterline, TMetricsCalculator>            Self;
  typedef itk::Object                            Superclass;
  typedef itk::SmartPointer<Self>                Pointer;
  typedef itk::SmartPointer<const Self>          ConstPointer;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;

  typedef TMetricsCalculator                          MetricsCalculatorType;
  typedef typename MetricsCalculatorType::Pointer     MeasurementCalculatorPointer;
  
  typedef typename MetricsCalculatorType::RangeValueType    RangeValueType;
  typedef typename MetricsCalculatorType::RangeType         RangeType;
  
public:

  /** Method for creation through the object factory. */
  //itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselSectionEstimator, itk::Object );
  
  virtual void SetCenterline( CenterlineType * centerline );
  CenterlineType * GetCenterline()
    { return m_Centerline; }
  const CenterlineType * GetCenterline() const
    { return m_Centerline; }
  
  /** Range of sections to calculate. */
  void SetSectionRange( RangeValueType first, RangeValueType last );
  
  /** Calculate just for a section. */
  void SetSection( RangeValueType section );
  
  /** Set range of sections to calculate to all section. */
  void SetSectionRangeToAll();

  itkGetConstMacro( CurrentScale, double );
  
  virtual void Compute() = 0;
    
protected:
  
  VesselSectionEstimator();
  ~VesselSectionEstimator();
  
  /** Check the validity range when the centerline or range is set. */
  void CheckSectionRangeIsValid();
    
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  VesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  /** Container of sections. */
  CenterlinePointer              m_Centerline;
  
  /** Object that calculates other non-generic measurements. */
  MeasurementCalculatorPointer   m_MeasurementCalculator;
  
  /** Range of sections to calculate. */
  RangeType                      m_SectionRange;
  
  /** Scale of last calculation. */
  double                         m_CurrentScale;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanVesselSectionEstimator.hxx"
#endif

#endif
