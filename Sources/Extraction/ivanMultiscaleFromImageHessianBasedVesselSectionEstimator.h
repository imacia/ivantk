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
// File: ivanMultiscaleHessianBasedVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/06/22


#ifndef __ivanMultiscaleFromImageHessianBasedVesselSectionEstimator_h
#define __ivanMultiscaleFromImageHessianBasedVesselSectionEstimator_h


#include "itkMultiscaleHessianBasedVesselSectionEstimator.h"
#include "itkLinearInterpolateImageFunction.h"

namespace ivan
{
  
/** \class MultiscaleFromImageHessianBasedVesselSectionEstimator
 *  \brief Hessian matrix section estimator where scales are given by a scales image.
 *
 * Hessian-based section estimator uses the eigenvectors of the two most-negative eigenvalues
 * of the local Hessian matrix at the given section center. The Hessian matrix is calculated
 * at each point at a scale which corresponds to the estimated interpolated value obtained
 * from a scales image.
 *
 * It is assumed that the section has a Normal such as the CircularVesselSection.
 *
 * \ingroup 
 */

template <class TImage, class TScaleImage, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT MultiscaleFromImageHessianBasedVesselSectionEstimator : 
  public MultiscaleHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
{
public:

  /** Standard class typedefs. */
  typedef MultiscaleFromImageHessianBasedVesselSectionEstimator
    <TImage,TScaleImage,TCenterline,TMetricsCalculator>          Self;
  typedef MultiscaleHessianBasedVesselSectionEstimator
    <TImage,TCenterline,TMetricsCalculator>                      Superclass;
  typedef itk::SmartPointer<Self>                                Pointer;
  typedef itk::SmartPointer<const Self>                          ConstPointer;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;

  typedef TImage                                ImageType;
  typedef typename ImageType::Pointer           ImagePointer;
    
  typedef TScaleImage                           ScaleImageType;
  typedef typename ScaleImageType::Pointer      ScaleImagePointer;
  typedef itk::LinearInterpolateImageFunction
    <ScaleImageType>                            ScaleImageInterpolatorType;
  
  typedef typename Superclass::HessianFunctionType       HessianFunctionType;
  typedef typename Superclass::HessianFunctionPointer    HessianFunctionPointer;
  typedef typename Superclass::HessianTensorType         HessianTensorType;
  typedef typename Superclass::HessianPointType          HessianPointType;
    
  typedef typename Superclass::HessianFunctionContainer  HessianFunctionContainer;
    
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( MultiscaleFromImageHessianBasedVesselSectionEstimator,
    MultiscaleHessianBasedVesselSectionEstimator );
  
  virtual void SetScaleImage( const ScaleImageType *image );
      
  const ScaleImageType * GetScaleImage() const
    { return m_ScaleImage; }
      
protected:
  
  MultiscaleFromImageHessianBasedVesselSectionEstimator();
  ~MultiscaleFromImageHessianBasedVesselSectionEstimator();
  
  /** Get scale at current point. Subclasses implement this by taking the scale from the
    * estimated radius or from a source scales image for example. */
  virtual double GetScaleAt( unsigned int centerlineIdx, const PointType & point );
      
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  MultiscaleFromImageHessianBasedVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  /** Image that represents the estimated scales at each point. */
  ScaleImagePointer            m_ScaleImage;
  
  /** Interpolate scale values at current location. */
  ScaleImageInterpolatorType   m_ScaleInterpolator;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanMultiscaleFromImageHessianBasedVesselSectionEstimator.hxx"
#endif

#endif
