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
// File: ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/06/22


#ifndef __ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimator_h
#define __ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimator_h


#include "ivanMultiscaleHessianBasedVesselSectionEstimator.h"
#include "ivanImageFunctionInitializerBase.h"


namespace ivan
{
  
/** \class MultiscaleAdaptiveHessianBasedVesselSectionEstimator
 *  \brief Hessian matrix section estimator where scales are adapted from previous calculations
 *
 * Hessian-based section estimator uses the eigenvectors of the two most-negative eigenvalues
 * of the local Hessian matrix at the given section center. The Hessian matrix is calculated at 
 * each point at a scale that is adaptively calculated.
 *
 * The best scale is found among the range of possible scales by checking the maximum value of a 
 * metric function, usually a vesselness function. The metric function is initialized by providing
 * an initializer object of type MetricFunctionInitializerBase or any user-defined subclass. This
 * avoids to initialize externally every operator for each scale
 *
 * /sa MetricFunctionInitializerBase
 * /sa ScaleSpaceMetricFunctionInitializer
 *
 */

template <class TImage, class TMetricFunction, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT MultiscaleAdaptiveHessianBasedVesselSectionEstimator : 
  public MultiscaleHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
{
public:

  /** Standard class typedefs. */
  typedef MultiscaleAdaptiveHessianBasedVesselSectionEstimator
    <TImage,TMetricFunction,TCenterline,TMetricsCalculator>     Self;
  typedef MultiscaleHessianBasedVesselSectionEstimator
    <TImage,TCenterline,TMetricsCalculator>                     Superclass;
  typedef itk::SmartPointer<Self>                               Pointer;
  typedef itk::SmartPointer<const Self>                         ConstPointer;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;

  typedef TImage                                ImageType;
  typedef typename ImageType::Pointer           ImagePointer;
  typedef typename Superclass::PointType        PointType;
    
  typedef typename Superclass::ScaleVectorType  ScaleVectorType;
    
  typedef typename Superclass::ScaledImageFunctionType            ScaledImageFunctionType;
  typedef typename Superclass::ScaledImageFunctionPointer         ScaledImageFunctionPointer;
  
  typedef typename Superclass::ScaledImageFunctionContainerType   ScaledImageFunctionContainerType;
  
  typedef typename Superclass::StructureTensorType                StructureTensorType;
    
  typedef typename Superclass::HessianFunctionType                HessianFunctionType;
  typedef typename Superclass::HessianFunctionPointer             HessianFunctionPointer;
  
  typedef TMetricFunction                                         MetricFunctionType;
  typedef typename MetricFunctionType::Pointer                    MetricFunctionPointer;
  typedef std::vector<MetricFunctionPointer>                      MetricFunctionContainerType;
   
  typedef ImageFunctionInitializerBase<TMetricFunction,TImage>    MetricFunctionInitializerType; 
  typedef typename MetricFunctionInitializerType::Pointer         MetricFunctionInitializerPointer;
   
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( MultiscaleAdaptiveHessianBasedVesselSectionEstimator, 
    MultiscaleHessianBasedVesselSectionEstimator );
    
  /** Get the metric function container. This can be accessed in order to set the properties.
    * WARNING: do this after calling Initialize(). */
  MetricFunctionContainerType & GetMetricFunctionContainer()
    { return m_MetricFunctionContainer; }
  const MetricFunctionContainerType & GetMetricFunctionContainer() const
    { return m_MetricFunctionContainer; }
    
  itkSetObjectMacro( MetricFunctionInitializer, MetricFunctionInitializerType );
  itkGetObjectMacro( MetricFunctionInitializer, MetricFunctionInitializerType );
  itkGetConstObjectMacro( MetricFunctionInitializer, MetricFunctionInitializerType );
  
  /** Initialize the Gaussian kernel. Call this method before evaluating the function.
    * This method MUST be called after any changes to function parameters. */
  virtual void Initialize();
    
protected:
  
  MultiscaleAdaptiveHessianBasedVesselSectionEstimator();
  ~MultiscaleAdaptiveHessianBasedVesselSectionEstimator();
  
  /** Get scale at current point. Subclasses implement this by taking the scale from the
    * estimated radius or from a source scales image for example. */
  virtual double GetScaleAt( unsigned int centerlineIdx, const PointType & point );
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  MultiscaleAdaptiveHessianBasedVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  /** Metric function used to select scale by comparing the metric value accross scales and 
    * taking the maximum. */
  MetricFunctionContainerType        m_MetricFunctionContainer;
  
  /** Provide a means to initialize the metric function for each scale. This is called at 
    * initialization for the metrics of all scales. */
  MetricFunctionInitializerPointer   m_MetricFunctionInitializer;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimator.hxx"
#endif

#endif
