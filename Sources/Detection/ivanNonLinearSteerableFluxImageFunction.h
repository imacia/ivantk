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
// File: ivanNonLinearSteerableFluxImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanNonLinearSteerableFluxImageFunction_h
#define __ivanNonLinearSteerableFluxImageFunction_h

#include "ivanFluxBasedVesselnessImageFunction.h"


namespace ivan
{

/**
 * \class NonLinearSteerableFluxImageFunction
 * \brief Computes flux in the direction given by the eigenvectors that estimate the vessel plane.
 *
 * This class computes the flux as a combination of a non-linear flux measure. The calculation of 
 * this non-linear flux, which tries to avoid responses from step edges is described in Koller et al.
 * The original implementation combines the responses from both section eigenvector directions
 * by taking the minimum of both non-linear flux calculations. The shift for the filters is taken
 * as the scale (Sigma) times the RadiusFactor. The original implementation took as shift the value
 * of Sigma, that is RadiusFactor = 1.0. 
 * 
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 * References: 
 * Th. M. Koller, G. Gerig, G. Szekely and D. Dettwiler, ``Multiscale Detection of Curvilinear
 * Structures in 2-D and 3-D Image Data'', In Proc. International Conference on Computer Vision 
 * (ICCV'95), pp. 864, 1995.
 *
 * \sa ImageFunction
 */
template <class TInputImage, class TOutput=double>
class ITK_EXPORT NonLinearSteerableFluxImageFunction :
  public FluxBasedVesselnessImageFunction<TInputImage,TOutput,TOutput>
{
public:

  /**Standard "Self" typedef */
  typedef NonLinearSteerableFluxImageFunction
    <TInputImage,TOutput>            Self;
  typedef FluxBasedVesselnessImageFunction
    <TInputImage,TOutput,TOutput>            Superclass;

  /** Smart pointer typedef support */
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef TInputImage  InputImageType;
  typedef TOutput      OutputType;
    
  typedef typename Superclass::InputPixelType        InputPixelType;
  typedef typename Superclass::IndexType             IndexType;
  typedef typename Superclass::ContinuousIndexType   ContinuousIndexType;
  typedef typename Superclass::PointType             PointType;
    
  /** Types for Hessian function/matrix. */
  typedef ivan::DiscreteHessianGaussianImageFunction
    <TInputImage,TOutput>                            HessianFunctionType;
  typedef typename HessianFunctionType::Pointer      HessianFunctionPointer;
  typedef typename HessianFunctionType::TensorType   HessianTensorType;
  
  /** Types for gradient calculation. */
  typedef typename Superclass::GradientFunctionType      GradientFunctionType;
  typedef typename Superclass::GradientFunctionPointer   GradientFunctionPointer;
    
  /** Interpolation modes */
  typedef typename Superclass::InterpolationModeType     InterpolationModeType;
    
  typedef itk::Vector<double,InputImageType::ImageDimension>   VectorType;

public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( NonLinearSteerableFluxImageFunction, HessianBasedVesselnessImageFunction );
  
protected:

  NonLinearSteerableFluxImageFunction();
  ~NonLinearSteerableFluxImageFunction();
  
  /** Evaluate vesselness given Hessian. This must be reimplemented by subclasses. */
  virtual OutputType EvaluateVesselnessAtContinuousIndex( const HessianTensorType & hessian,
    const ContinuousIndexType & cindex ) const;
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  NonLinearSteerableFluxImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

private:
  


};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_NonLinearSteerableFluxImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT NonLinearSteerableFluxImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef NonLinearSteerableFluxImageFunction< ITK_TEMPLATE_2 x > \
                                                  NonLinearSteerableFluxImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanNonLinearSteerableFluxImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanNonLinearSteerableFluxImageFunction.hxx"
#endif

#endif
