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
// Date: 2012/02/03


#ifndef __ivanMultiscaleHessianBasedVesselSectionEstimator_h
#define __ivanMultiscaleHessianBasedVesselSectionEstimator_h


#include "ivanMultiscaleTensorBasedVesselSectionEstimator.h"
#include "ivanDiscreteHessianGaussianImageFunction.h"


namespace ivan
{

/** \class MultiscaleHessianBasedVesselSectionEstimator
 *  \brief Abstract class estimating the vessel section using eigenvectors of the Hessian matrix at multiple scales
 *
 * Hessian-based section estimator uses the eigenvectors of the two most-negative eigenvalues
 * of the local Hessian matrix at the given section center. The Hessian matrix must be calculated
 * at a certain scale which corresponds to the typical deviation of the Gaussian convolution used
 * to calculated derivatives for the Hessian (scale-space derivatives). This calculated is performed
 * via DiscreteHessianGaussianImageFunction which calculates the Hessian locally.
 *
 * It is assumed that the section has a Normal such as the CircularVesselSection.
 *
 * This is an abstract class that defines most of the operations. Subclasses implement how the scale
 * at each point is chosen, for example from a source scale image, from the estimated vessel radius
 * or as the scale which provides the maximum of a vesselness function.
 *
 * \ingroup 
 */

template <class TImage, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT MultiscaleHessianBasedVesselSectionEstimator : 
  public MultiscaleTensorBasedVesselSectionEstimator<TImage,
    DiscreteHessianGaussianImageFunction<TImage>,TCenterline,TMetricsCalculator>
{

public:

  /** Standard class typedefs. */
  typedef MultiscaleHessianBasedVesselSectionEstimator
    <TImage, TCenterline, TMetricsCalculator>             Self;
  typedef MultiscaleTensorBasedVesselSectionEstimator<TImage,
      DiscreteHessianGaussianImageFunction<TImage>,
      TCenterline,TMetricsCalculator>                     Superclass;
  typedef itk::SmartPointer<Self>                         Pointer;
  typedef itk::SmartPointer<const Self>                   ConstPointer;
  
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
    
  /** The use of the previous traits is preferred for the image function. */
  typedef ivan::DiscreteHessianGaussianImageFunction<ImageType>    HessianFunctionType;
  typedef typename HessianFunctionType::Pointer                   HessianFunctionPointer;
    
public:

  /** Method for creation through the object factory. */
  //itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( MultiscaleHessianBasedVesselSectionEstimator, MultiscaleTensorBasedVesselSectionEstimator );
      
protected:
  
  MultiscaleHessianBasedVesselSectionEstimator();
  ~MultiscaleHessianBasedVesselSectionEstimator();
  
  /** Initialize a single scaled image function or operator. This is called by Initialize() for
    * every operator. This may be reimplemented for specific operators. Reimplemented to initialize
    * Hessian Gaussian operator. */    
  virtual void InitializeScaledFunction( ScaledImageFunctionType *scaledImageFunction, double scale );
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  MultiscaleHessianBasedVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanMultiscaleHessianBasedVesselSectionEstimator.hxx"
#endif

#endif
