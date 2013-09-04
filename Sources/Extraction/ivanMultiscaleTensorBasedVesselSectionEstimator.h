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
// File: ivanMultiscaleTensorBasedVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2012/02/03


#ifndef __ivanMultiscaleTensorBasedVesselSectionEstimator_h
#define __ivanMultiscaleTensorBasedVesselSectionEstimator_h


#include "ivanMultiscaleVesselSectionEstimator.h"



namespace ivan
{

/** \class MultiscaleTensorBasedVesselSectionEstimator
 *  \brief Abstract class estimating the vessel section using multiple scales of a tensor-based operator.
 *
 * MultiscaleTensorBasedVesselSectionEstimator relies on a tensor-based operator or image function that 
 * may be computed at multiple scales, in order to compute a multi-scale section estimator. Such tensor
 * describes the local structure around the given point. Examples of structure tensors (matrices) are
 * the Hessian matrix and the Oriented Flux (Q) matrix. The scaled operator must be able to calculate
 * the structure tensor TStructureTensor. It is left as a parameter, since it may not be necessarily
 * the output of the image function.
 *
 * Default computation computes multi-scale eigenvalues and eigenvectors of the structure tensor and uses
 * them for the section calculation.
 *
 * Subclasses must provide the operator (as an image function) used to calculate the section (examples are 
 * the Hessian or Optimally Oriented Flux) and a means to obtain the best scale at a point and a way
 * to initialize this operator if necessary (via Initialize() or InitializeScaledFunction() ). This can be 
 * obtained for example from a source scale image, from the estimated vessel radius or as the scale which 
 * provides the maximum of a vesselness function.
 * 
 * It is assumed that the section has a Normal such as the CircularVesselSection.
 *
 * \ingroup 
 */

template <class TImage, class TScaledImageFunction, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT MultiscaleTensorBasedVesselSectionEstimator : 
  public MultiscaleVesselSectionEstimator<TImage,TScaledImageFunction,TCenterline,TMetricsCalculator>
{

public:

  /** Standard class typedefs. */
  typedef MultiscaleTensorBasedVesselSectionEstimator
    <TImage, TScaledImageFunction, TCenterline, TMetricsCalculator>   Self;
  typedef MultiscaleVesselSectionEstimator
    <TImage, TScaledImageFunction, TCenterline, TMetricsCalculator>   Superclass;
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;

  typedef TImage                                ImageType;
  typedef typename ImageType::Pointer           ImagePointer;
  typedef typename Superclass::PointType        PointType;
  
  typedef typename Superclass::ScaleVectorType  ScaleVectorType;
    
  typedef TScaledImageFunction                  ScaledImageFunctionType;
  typedef typename ScaledImageFunctionType      ScaledImageFunctionPointer;
  
  typedef typename Superclass::ScaledImageFunctionContainerType   ScaledImageFunctionContainerType;
  
  typedef typename ScaledImageFunctionType::TensorType            StructureTensorType;
      
public:

  /** Method for creation through the object factory. */
  //itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( MultiscaleTensorBasedVesselSectionEstimator, MultiscaleVesselSectionEstimator );
  
  /** Default computation computes multi-scale eigenvalues and eigenvectors of the structure tensor and uses
    * them for the section calculation. */
  virtual void Compute();
    
protected:
  
  MultiscaleTensorBasedVesselSectionEstimator();
  ~MultiscaleTensorBasedVesselSectionEstimator();
  
  /** Get scale at current point. Subclasses implement this by taking the scale from the
    * estimated radius or from a source scales image for example. */
  virtual double GetScaleAt( unsigned int centerlineIdx, const PointType & point ) = 0;
  
  /** Get interpolated structure tensor value at given scale. */
  void GetInterpolatedTensor( const PointType & center, double scale, StructureTensorType & tensor );
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  MultiscaleTensorBasedVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanMultiscaleTensorBasedVesselSectionEstimator.hxx"
#endif

#endif
