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
// File: ivanFixedScaleHessianBasedVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/06/04


#ifndef __ivanFixedScaleHessianBasedVesselSectionEstimator_h
#define __ivanFixedScaleHessianBasedVesselSectionEstimator_h


#include "ivanVesselSectionEstimator.h"
#include "ivanDiscreteHessianGaussianImageFunction.h"


namespace ivan
{
  
/** \class FixedScaleHessianBasedVesselSectionEstimator
 *  \brief Estimates the vessel section using the eigenvectors of the local Hessian matrix at fixed scale.
 *
 * Hessian-based section estimator uses the eigenvectors of the two most-negative eigenvalues
 * of the local Hessian matrix at the given section center. The Hessian matrix must be calculated
 * at a certain scale which corresponds to the typical deviation of the Gaussian convolution used
 * to calculated derivatives for the Hessian (scale-space derivatives). This calculated is performed
 * via DiscreteHessianGaussianImageFunction which calculates the Hessian locally.
 *
 * It is assumed that the section has a Normal such as the CircularVesselSection.
 *
 * \ingroup 
 */

template <class TImage, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT FixedScaleHessianBasedVesselSectionEstimator : 
  public VesselSectionEstimator<TCenterline,TMetricsCalculator>
{

public:

  /** Standard class typedefs. */
  typedef FixedScaleHessianBasedVesselSectionEstimator
    <TImage, TCenterline, TMetricsCalculator>   Self;
  typedef VesselSectionEstimator
    <TCenterline,TMetricsCalculator>            Superclass;
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;

  typedef TImage                                ImageType;
  typedef typename ImageType::Pointer           ImagePointer;
  
  typedef ivan::DiscreteHessianGaussianImageFunction<ImageType>   HessianFunctionType;
  typedef typename HessianFunctionType::Pointer                  HessianFunctionPointer;
  typedef typename HessianFunctionType::TensorType               HessianTensorType;
  typedef typename HessianFunctionType::PointType                HessianPointType;
    
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( FixedScaleHessianBasedVesselSectionEstimator, VesselSectionEstimator );
  
  virtual void SetScale( double scale );
  itkGetConstMacro( Scale, double );
  
  virtual void SetImage( ImageType *image );
      
  /** Provide access to the Hessian function in case we need to change some other properties. */
  HessianFunctionType * GetHessianFunction()
    { return m_HessianFunction; }
  const HessianFunctionType * GetHessianFunction() const
    { return m_HessianFunction; }
  
  virtual void Compute();
    
protected:
  
  FixedScaleHessianBasedVesselSectionEstimator();
  ~FixedScaleHessianBasedVesselSectionEstimator();
  
  /** Hessian kernel calculation is expensive so compute it only when properties change for 
    * fixed scale calculations. */
  void RecomputeKernel();
    
  void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:

  FixedScaleHessianBasedVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  double         m_Scale;
 
  ImagePointer   m_Image;
  
  HessianFunctionPointer  m_HessianFunction;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanFixedScaleHessianBasedVesselSectionEstimator.hxx"
#endif

#endif
