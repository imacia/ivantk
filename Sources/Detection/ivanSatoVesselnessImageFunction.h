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
// File: ivanSatoVesselnessImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanSatoVesselnessImageFunction_h
#define __ivanSatoVesselnessImageFunction_h

#include "ivanHessianOnlyBasedVesselnessImageFunction.h"


namespace ivan
{
  
/**
 * \class SatoVesselnessImageFunction
 * \brief Compute Sato's vesselness measure in the form an image function.
 *
 * Line filter to provide a vesselness measure for tubular objects from the
 * hessian matrix. The filter takes as input an image of hessian pixels
 * (SymmetricSecondRankTensor pixels) and preserves pixels that have
 * eigen values \f$ \lambda_3 \f$ close to 0 and \f$\lambda_2\f$ and \f$\lambda_1\f$ as
 * large negative values. (for bright tubular structures). This is the same as 
 * Hessian3DToVesselnessMeasureImageFilter but in the form of an ImageFunction.
 * 
 * This class is templated over the input image type.
 *
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 * \sa ImageFunction
 * \sa Hessian3DToVesselnessMeasureImageFilter
 * 
 */
template <class TInputImage, class TOutput, class TCoordRep=double>
class ITK_EXPORT SatoVesselnessImageFunction :
  public HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  /**Standard "Self" typedef */
  typedef SatoVesselnessImageFunction
    <TInputImage,TOutput,TCoordRep>            Self;
  typedef HessianOnlyBasedVesselnessImageFunction
    <TInputImage,TOutput,TCoordRep>            Superclass;

  /** Smart pointer typedef support */
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef TInputImage  InputImageType;
  typedef TOutput      OutputType;
  typedef TCoordRep    CoordRepType;
  
  typedef typename Superclass::InputPixelType        InputPixelType;
  typedef typename Superclass::IndexType             IndexType;
  typedef typename Superclass::ContinuousIndexType   ContinuousIndexType;
  typedef typename Superclass::PointType             PointType;
    
  /** Types for Hessian function/matrix. */
  typedef ivan::DiscreteHessianGaussianImageFunction
    <TInputImage,TCoordRep>                          HessianFunctionType;
  typedef typename HessianFunctionType::Pointer      HessianFunctionPointer;
  typedef typename HessianFunctionType::TensorType   HessianTensorType;
    
  /** Interpolation modes */
  typedef typename Superclass::InterpolationModeType InterpolationModeType;

public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( SatoVesselnessImageFunction, HessianOnlyBasedVesselnessImageFunction );
  
  itkSetMacro( FilterByEigenValues, bool );
  itkGetConstMacro( FilterByEigenValues, bool );
  itkBooleanMacro( FilterByEigenValues );
  
  itkSetMacro( Alpha, double );
  itkGetConstMacro( Alpha, double );
  
  itkSetMacro( Gamma12, double );
  itkGetConstMacro( Gamma12, double );
    
  itkSetMacro( Gamma23, double );
  itkGetConstMacro( Gamma23, double );
  
protected:

  SatoVesselnessImageFunction();
  ~SatoVesselnessImageFunction();
  
  /** Evaluate vesselness given Hessian. */
  virtual OutputType EvaluateVesselness( const HessianTensorType & hessian ) const;
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  SatoVesselnessImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

private:

  double m_Gamma12;
  double m_Gamma23;
  double m_Alpha;
  
  /** Flag for filtering by eigenvalues. This will filter all points where most two negative eigenvalues
    * are positive or the third is larger than the second in absolute value divided by alpha. */
  bool   m_FilterByEigenValues;
};

} // end namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_SatoVesselnessImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT SatoVesselnessImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef SatoVesselnessImageFunction< ITK_TEMPLATE_2 x > \
                                                  SatoVesselnessImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanSatoVesselnessImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanSatoVesselnessImageFunction.hxx"
#endif

#endif
