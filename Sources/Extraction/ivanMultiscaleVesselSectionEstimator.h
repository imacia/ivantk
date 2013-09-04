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
// File: ivanMultiscaleVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2012/02/03


#ifndef __ivanMultiscaleVesselSectionEstimator_h
#define __ivanMultiscaleVesselSectionEstimator_h


#include "ivanImageBasedVesselSectionEstimator.h"
#include "ivanDiscreteHessianGaussianImageFunction.h"


namespace ivan
{

/** \class MultiscaleVesselSectionEstimator
 *  \brief Abstract class estimating the vessel section using multiple scales.
 *
 * MultiscaleVesselSectionEstimator relies on an operator or image function that may be computed at multiple
 * scales, in order to compute a multi-scale section estimator.
 *
 * Subclasses must provide the operator (as an image function) used to calculate the section (examples are 
 * the Hessian or Optimally Oriented Flux) and a means to obtain the best scale at a point. This can be 
 * obtained for example from a source scale image, from the estimated vessel radius or as the scale which 
 * provides the maximum of a vesselness function.
 * 
 * It is assumed that the section has a Normal such as the CircularVesselSection.
 *
 * \ingroup 
 */

template <class TImage, class TScaledImageFunction, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT MultiscaleVesselSectionEstimator : 
  public ImageBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
{

public:

  /** Standard class typedefs. */
  typedef MultiscaleVesselSectionEstimator<TImage, 
    TScaledImageFunction, TCenterline, TMetricsCalculator>   Self;
  typedef ImageBasedVesselSectionEstimator
    <TImage,TCenterline,TMetricsCalculator>                  Superclass;
  typedef itk::SmartPointer<Self>                            Pointer;
  typedef itk::SmartPointer<const Self>                      ConstPointer;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;

  typedef TImage                                ImageType;
  typedef typename ImageType::Pointer           ImagePointer;
  typedef typename ImageType::PointType         PointType;
    
  typedef std::vector<double>                        ScaleVectorType;
    
  typedef TScaledImageFunction                       ScaledImageFunctionType;
  typedef typename ScaledImageFunctionType::Pointer  ScaledImageFunctionPointer;
  
  typedef std::vector<ScaledImageFunctionPointer>    ScaledImageFunctionContainerType;
    
  enum ScaleStepMethodType
  { 
    EquispacedScaleSteps = 0,
    LogarithmicScaleSteps = 1,
    PowerFactorScaleSteps = 2  // provide a power factor s such that scale n is s^n * sigma0 where sigma0 is the first scale
  };
    
public:

  /** Method for creation through the object factory. */
  //itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( MultiscaleVesselSectionEstimator, ImageBasedVesselSectionEstimator );
  
  /** Set/Get macros for the minimum scale. */
  itkSetMacro( MinimumScale, double );
  itkGetConstMacro( MinimumScale, double );
  
  /** Set/Get macros for maximum scale. */
  itkSetMacro( MaximumScale, double );
  itkGetConstMacro( MaximumScale, double );

  /** Set/Get macros for Number of Scales.*/
  itkSetMacro( NumberOfScales, unsigned int );
  itkGetConstMacro( NumberOfScales, unsigned int );
  
  /** Set/Get macros for the Power Factor. This is only used when selecting PowerFactorScaleSteps as scale selection method. */
  itkSetMacro( PowerFactor, double );
  itkGetConstMacro( PowerFactor, double );
  
  /** Set/Get the method used to generate scale sequence (Equispaced
   * or Logarithmic) */
  itkSetMacro( ScaleStepMethod, ScaleStepMethodType );
  itkGetConstMacro( ScaleStepMethod, ScaleStepMethodType );

  /**Set equispaced sigma step method */
  void SetScaleStepMethodToEquispaced()
    { this->SetScaleStepMethod( EquispacedScaleSteps ); }

  /**Set logartihmic sigma step method */
  void SetScaleStepMethodToLogarithmic()
    { this->SetScaleStepMethod( LogarithmicScaleSteps ); }
  
  /** Initialize the kernels. Call this method before evaluating the function.
    * This method MUST be called after any changes to function parameters. By default initialize
    * computes al the kernels using the chosen scale step computation method and fills the scale
    * container. In order to initialize each created kernel, the virtual method InitializeSpaceFunction()
    * may be reimplemented, instead of rewriting Initialize() completely. */
  virtual void Initialize();
    
protected:
  
  MultiscaleVesselSectionEstimator();
  ~MultiscaleVesselSectionEstimator();
  
  /** Get scale at current point. Subclasses implement this by taking the scale from the
    * estimated radius or from a source scales image for example. */
  virtual double GetScaleAt( unsigned int centerlineIdx, const PointType & point ) = 0;
  
  /** Initialize a single scaled image function or operator. This is called by Initialize() for
    * every operator. This may be reimplemented for specific operators. */    
  virtual void InitializeScaledFunction( ScaledImageFunctionType *scaledImageFunction, double scale ) {}
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  MultiscaleVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  /** Contains the different hessian function for the different scales. */  
  ScaledImageFunctionContainerType  m_ScaledImageFunctionContainer;
  
  /** Optimize across scales by calculating at several scales. */
  bool                          m_OptimizeScale;
  
  double                        m_MinimumScale;
  double                        m_MaximumScale;

  unsigned int                  m_NumberOfScales;
  
  ScaleStepMethodType           m_ScaleStepMethod;
  
  /** Power factor used when PowerFactorScaleStepMethod is selected for scales. */
  double                        m_PowerFactor;
  
  /** Sigmas calculated internally. */
  ScaleVectorType               m_Scales;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanMultiscaleVesselSectionEstimator.hxx"
#endif

#endif
