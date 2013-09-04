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
// File: ivanMultiscaleFromRadiusHessianBasedVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/06/22


#ifndef __ivanMultiscaleFromRadiusHessianBasedVesselSectionEstimator_h
#define __ivanMultiscaleFromRadiusHessianBasedVesselSectionEstimator_h


#include "itkMultiscaleHessianBasedVesselSectionEstimator.h"


namespace ivan
{
  
namespace Vessel
{
  
class SimpleScaledRadiusFunctor
{

public:

  SimpleScaledRadiusFunctor() : m_ScaleFactor( 1.732050808 ) {}
  
  itkSetMacro( ScaleFactor, double );
  itkGetConstMacro( ScaleFactor, double );
  
  /** Overloaded operator returns the scale given the radius. */
  double operator() ( double radius )
    { return radius / m_ScaleFactor; }

private:

  /** The radius is divided by this factor to estimate the scale. */
  double m_ScaleFactor; 
}
 
  
/** \class MultiscaleFromRadiusHessianBasedVesselSectionEstimator
 *  \brief Hessian matrix section estimator where scales are given by the currently estimated vessel radius
 *
 * Hessian-based section estimator uses the eigenvectors of the two most-negative eigenvalues
 * of the local Hessian matrix at the given section center. The Hessian matrix is calculated
 * at each point at a scale which corresponds to a function of the estimated radius.
 *
 * The TRadiusFunctor template parameter provides a generic mean to calculate the scale from the radius.
 * This can be as simple as dividing the radius by a scale.
 *
 * It is assumed that the section has a Normal such as the CircularVesselSection.
 *
 * \ingroup 
 */

template <class TImage, class TScaleImage, class TRadiusFunctor, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT MultiscaleFromRadiusHessianBasedVesselSectionEstimator : 
  public VesselSectionEstimator<TCenterline,TMetricsCalculator>
{
public:

  /** Standard class typedefs. */
  typedef MultiscaleFromRadiusHessianBasedVesselSectionEstimator
    <TImage,TScaleImage,TRadiusFunctor, TCenterline,TMetricsCalculator>   Self;
  typedef MultiscaleHessianBasedVesselSectionEstimator
    <TImage,TCenterline,TMetricsCalculator>                               Superclass;
  typedef itk::SmartPointer<Self>                                         Pointer;
  typedef itk::SmartPointer<const Self>                                   ConstPointer;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;

  typedef TImage                                ImageType;
  typedef typename ImageType::Pointer           ImagePointer;
    
  typedef typename Superclass::HessianFunctionType       HessianFunctionType;
  typedef typename Superclass::HessianFunctionPointer    HessianFunctionPointer;
  typedef typename Superclass::HessianTensorType         HessianTensorType;
  typedef typename Superclass::HessianPointType          HessianPointType;
    
  typedef typename Superclass::HessianFunctionContainer  HessianFunctionContainer;
  
  typedef TRadiusFunctor       RadiusFunctorType;
    
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( MultiscaleFromRadiusHessianBasedVesselSectionEstimator, 
    MultiscaleHessianBasedVesselSectionEstimator );
  
  TRadiusFunctor * GetRadiusFunctor()
    { return &m_RadiusFunctor; }
  const TRadiusFunctor * GetRadiusFunctor() const
    { return &m_RadiusFunctor; }
  
protected:
  
  MultiscaleFromRadiusHessianBasedVesselSectionEstimator();
  ~MultiscaleFromRadiusHessianBasedVesselSectionEstimator();
  
  /** Get scale at current point. Subclasses implement this by taking the scale from the
    * estimated radius or from a source scales image for example. */
  virtual double GetScaleAt( unsigned int centerlineIdx, const PointType & point );
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  MultiscaleFromRadiusHessianBasedVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  RadiusFunctorType    m_RadiusFunctor;
};

} // end namespace Vessel

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "itkMultiscaleFromRadiusHessianBasedVesselSectionEstimator.hxx"
#endif

#endif
