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
// File: ivanFixedScaleOOFBasedVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2012/01/21


#ifndef __ivanFixedScaleOOFBasedVesselSectionEstimator_h
#define __ivanFixedScaleOOFBasedVesselSectionEstimator_h


#include "ivanVesselSectionEstimator.h"
#include "ivanOrientedFluxMatrixBasedVesselnessImageFunction.h"


namespace ivan
{
  
/** \class FixedScaleOOFBasedVesselSectionEstimator
 *  \brief Estimates the vessel section using Optimally Oriented Flux
 *
 * This object estimates the vessel section using the eigenvectors of the local flux matrix Q at a fixed scale.
 * This is also called Optimally Oriented Flux (OOF), since it is the direction where the flux is minimal. Typically
 * the vector field used for flux calculation is the gradient field.
 * 
 * This estimator uses the eigenvectors of the two most-negative eigenvalues of the local flux matrix at 
 * the given section center, similarly to the Hessian-based calculations. The flux matrix must be calculated
 * at a certain scale corresponding to the radius of the discrete sphere used for the flux computation.
 *
 * The OOF, as compared to the Hessian, is less sensitive to noise and to the presence of spurious adjacente 
 * structures. Usually the scale for the derivative calculations is fixed and small. This class also fixes
 * the radius of the sphere.
 *
 * It is assumed that the section has a Normal such as the CircularVesselSection.
 *
 * \par REFERENCES
 * Law, M.W.K. and Chung, A.C.S. "Three Dimensional Curvilinear Structure Detection Using Optimally 
 * Oriented Flux�, The 10th European Conf. on Computer Vision (ECCV'08), LNCS 5305:368�382 (2008).
 *
 * \ingroup 
 */

template <class TImage, class TVectorField, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT FixedScaleOOFBasedVesselSectionEstimator : 
  public VesselSectionEstimator<TCenterline,TMetricsCalculator>
{

public:

  /** Standard class typedefs. */
  typedef FixedScaleOOFBasedVesselSectionEstimator
    <TImage,TVectorField,TCenterline,TMetricsCalculator>   Self;
  typedef VesselSectionEstimator
    <TCenterline,TMetricsCalculator>                       Superclass;
  typedef itk::SmartPointer<Self>                          Pointer;
  typedef itk::SmartPointer<const Self>                    ConstPointer;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;

  typedef TImage                                ImageType;
  typedef typename ImageType::Pointer           ImagePointer;
    
  typedef TVectorField                          VectorFieldType;
  typedef typename VectorFieldType::Pointer     VectorFieldPointer;
  
  typedef OrientedFluxMatrixBasedVesselnessImageFunction
    <ImageType,VectorFieldType,double>                           OOFFunctionType;
  typedef typename OOFFunctionType::Pointer                      OOFFunctionPointer;
  typedef typename OOFFunctionType::FluxMatrixType               FluxMatrixType;
  typedef typename FluxMatrixType::EigenVectorsMatrixType        EigenVectorsMatrixType;
  typedef typename FluxMatrixType::EigenValuesArrayType          EigenValuesArrayType;
    
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( FixedScaleOOFBasedVesselSectionEstimator, VesselSectionEstimator );
  
  itkSetMacro( Radius, double );
  itkGetConstMacro( Radius, double );
  
  /** Set the scale for derivative calculations. In OOF this should be a small scale,
    * at the pixel resolution. Default is one. */
  itkSetMacro( GradientSigma, double );
  itkGetConstMacro( GradientSigma, double );
  
  virtual void SetImage( ImageType *image );
  
  /** Initialize OOF operator. Call this before performing any computation. */
  void Initialize();
      
  /** Provide access to the OOF function in case we need to change some other properties. */
  OOFFunctionType * GetOOFFunction()
    { return m_OOFFunction; }
  const OOFFunctionType * GetOOFFunction() const
    { return m_OOFFunction; }
  
  virtual void Compute();
    
protected:
  
  FixedScaleOOFBasedVesselSectionEstimator();
  ~FixedScaleOOFBasedVesselSectionEstimator();
  
  void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:

  FixedScaleOOFBasedVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  double                m_Radius;
 
  double                m_GradientSigma;
 
  ImagePointer          m_Image;
  
  OOFFunctionPointer    m_OOFFunction;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanFixedScaleOOFBasedVesselSectionEstimator.hxx"
#endif

#endif
