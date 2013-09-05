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
// File: ivanHessianEigenValuesImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/11/20

#ifndef __ivanHessianEigenValuesImageFunction_h
#define __ivanHessianEigenValuesImageFunction_h

#include "ivanMacros.h"
#include "ivanHessianOnlyBasedVesselnessImageFunction.h"

#include "itkSymmetricSecondRankTensor.h"


namespace ivan
{
  
/**
 * \class HessianEigenValuesImageFunction
 * \brief Image function that returns local Hessian-matrix eigenvalues.
 *
 * This class returns the local Hessian-matrix eigenvalues as a FixedArray.
 * 
 * This class is templated over the input image type.
 *
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 * \sa ImageFunction
 * 
 */
template <class TInputImage, class TCoordRep=double>
class ITK_EXPORT HessianEigenValuesImageFunction :
  public HessianOnlyBasedVesselnessImageFunction<TInputImage,
    typename itk::SymmetricSecondRankTensor
    < TCoordRep, ITKImageDimensionMacro(TInputImage) >::EigenValuesArrayType, TCoordRep>
{
public:

  /**Standard "Self" typedef */
  typedef HessianEigenValuesImageFunction
    <TInputImage,TCoordRep>                              Self;
  typedef HessianOnlyBasedVesselnessImageFunction
    <TInputImage, typename itk::SymmetricSecondRankTensor
    <TCoordRep,ITKImageDimensionMacro(TInputImage)>::EigenValuesArrayType,
    TCoordRep>                                           Superclass;

  /** Smart pointer typedef support */
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef TInputImage  InputImageType;
  typedef TCoordRep    CoordRepType;
  
  typedef typename itk::SymmetricSecondRankTensor
    < TCoordRep, ITKImageDimensionMacro(TInputImage) >
    ::EigenValuesArrayType                           OutputType;
  
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
  itkTypeMacro( HessianEigenValuesImageFunction, HessianOnlyBasedVesselnessImageFunction );
  
protected:

  HessianEigenValuesImageFunction();
  ~HessianEigenValuesImageFunction();
  
  /** Evaluate vesselness given Hessian. */
  virtual OutputType EvaluateVesselness( const HessianTensorType & hessian ) const;
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  HessianEigenValuesImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

private:


};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_HessianEigenValuesImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT HessianEigenValuesImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef HessianEigenValuesImageFunction< ITK_TEMPLATE_2 x > \
                                                  HessianEigenValuesImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanHessianEigenValuesImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanHessianEigenValuesImageFunction.hxx"
#endif

#endif
