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
// File: ivanMultiscaleOOFBasedVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2012/02/07


#ifndef __ivanMultiscaleOOFBasedVesselSectionEstimator_h
#define __ivanMultiscaleOOFBasedVesselSectionEstimator_h


#include "ivanMultiscaleTensorBasedVesselSectionEstimator.h"
#include "ivanOrientedFluxMatrixImageFunction.h"



namespace ivan
{

/** \class MultiscaleOOFBasedVesselSectionEstimator
 *  \brief Abstract class estimating the vessel section using Optimally Oriented Flux (OOF) at multiple scales
 *
 * OOF-based section estimator uses the eigenvectors of the two most-negative eigenvalues
 * of the local Oriented Flux (Q) matrix at the given section center. The normal to this section corresponds
 * to the Optimally Oriented Flux.
 * 
 * The Q matrix must be calculated at a certain scale which corresponds to the radius of the sphere in whose
 * surface the flux is calculated. For this we use OrientedFluxMatrixImageFunction.
 *
 * It is assumed that the section has a Normal such as the CircularVesselSection.
 *
 * This is an abstract class that defines most of the operations. Subclasses implement how the scale
 * at each point is chosen, for example from a source scale image, from the estimated vessel radius
 * or as the scale which provides the maximum of a vesselness function.
 *
 * \ingroup 
 *
 * /sa OrientedFluxMatrixImageFunction
 */

template <class TImage, class TVectorField, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT MultiscaleOOFBasedVesselSectionEstimator : 
  public MultiscaleTensorBasedVesselSectionEstimator<TImage,
    OrientedFluxMatrixImageFunction<TImage,TVectorField>,TCenterline,TMetricsCalculator>
{

public:

  /** Standard class typedefs. */
  typedef MultiscaleOOFBasedVesselSectionEstimator
    <TImage, TCenterline, TMetricsCalculator>                   Self;
  typedef MultiscaleTensorBasedVesselSectionEstimator<TImage,
    OrientedFluxMatrixImageFunction<TImage,TVectorField>,
    TCenterline,TMetricsCalculator>                             Superclass;
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

  typedef TVectorField                          VectorFieldType;
  typedef typename VectorFieldType::Pointer     VectorFieldPointer;

  typedef typename Superclass::ScaledImageFunctionType            ScaledImageFunctionType;
  typedef typename Superclass::ScaledImageFunctionPointer         ScaledImageFunctionPointer;
  
  typedef typename Superclass::ScaledImageFunctionContainerType   ScaledImageFunctionContainerType;
  
  typedef typename Superclass::StructureTensorType                StructureTensorType;

  /** The use of the previous traits is preferred for the image function. */
  typedef ivan::OrientedFluxMatrixImageFunction
    <ImageType, VectorFieldType>                                  OOFFunctionType;
  typedef typename OOFFunctionType::Pointer                       OOFFunctionPointer;
    
public:

  /** Method for creation through the object factory. */
  //itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( MultiscaleOOFBasedVesselSectionEstimator, MultiscaleTensorBasedVesselSectionEstimator );
      
protected:
  
  MultiscaleOOFBasedVesselSectionEstimator();
  ~MultiscaleOOFBasedVesselSectionEstimator();
  
  /** Initialize a single scaled image function or operator. This is called by Initialize() for
    * every operator. This may be reimplemented for specific operators. Reimplemented to initialize
    * OOF operator. */    
  virtual void InitializeScaledFunction( ScaledImageFunctionType *scaledImageFunction, double scale );
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  MultiscaleOOFBasedVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:


};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanMultiscaleOOFBasedVesselSectionEstimator.hxx"
#endif

#endif
