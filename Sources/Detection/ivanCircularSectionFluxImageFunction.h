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
// File: ivanCircularSectionFluxImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanCircularSectionFluxImageFunction_h
#define __ivanCircularSectionFluxImageFunction_h

#include "ivanFluxBasedVesselnessImageFunction.h"


namespace ivan
{
  
/**
 * \class CircularSectionFluxImageFunction
 * \brief Computes inward gradient flux through circular cross-sections as vesselness features.
 *
 * This class computes the flux as the inward gradient flux through circular cross-sections as
 * a vesselness feature. A non-linear penalization can be used in order to reduce the contribution 
 * of non-assymetric flux to the final response. 
 *
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 * References: 
 * D. Lesage, E. D. Angelini, I. Bloch and G. Funka-Lea, ``Design and Study of Flux-based Features
 * for 3D Vascular Tracking'', In Proc. IEEE Int. Symposium on Biomedical Imaging: From Nano to Macro 
 * (ISBI '09), 286-289, 2009.
 *
 */
template <class TInputImage, class TOutput=double>
class ITK_EXPORT CircularSectionFluxImageFunction :
  public FluxBasedVesselnessImageFunction<TInputImage,TOutput,TOutput>
{
public:

  /**Standard "Self" typedef */
  typedef CircularSectionFluxImageFunction
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
    
  enum NonLinearFluxFunction
  {
    MinimumFluxFunction,
    AverageFluxFunction    
  };

public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( CircularSectionFluxImageFunction, HessianBasedVesselnessImageFunction );
  
  /** Set the scale, that is, the standard deviation of the Gaussian kernel. Reimplemented to 
    * recompute intervals when using adaptative sampling. */
  virtual void SetSigma( double sigma );
  
  /** Set/Get use adaptative sampling. In adaptative sampling, the number of samples in the circle
    * depends on the current scale, and the radial resolution is ignored. */
  virtual void SetAdaptativeSampling( bool adaptative );
  itkGetConstMacro( AdaptativeSampling, bool );
  itkBooleanMacro( AdaptativeSampling );
  
  /** Set/Get the radial resolution. If non-linear flux is used, the number of samples will be 
    * added one to be even. */
  virtual void SetRadialResolution( const unsigned int resolution );
  itkGetConstMacro( RadialResolution, unsigned int );
  
  /** Set the option to use non-linear flux. This changes the number of samples to be even
    * by adding one if necessary. */
  virtual void SetUseNonLinearFlux( bool );
  itkGetConstMacro( UseNonLinearFlux, bool );
  itkBooleanMacro( UseNonLinearFlux );
  
  /** Set/Get how the opposite circular values are combined when using NonLinearFlux. */
  itkSetMacro( NonLinearFluxFunction, NonLinearFluxFunction );
  itkGetConstMacro( NonLinearFluxFunction, NonLinearFluxFunction );
  
protected:

  CircularSectionFluxImageFunction();
  ~CircularSectionFluxImageFunction();
  
  /** Precompute values that depends on the radial resolution such as sin or cos values. */
  virtual void ComputeIntervals();
  
  /** Evaluate vesselness given Hessian. This must be reimplemented by subclasses. */
  virtual OutputType EvaluateVesselnessAtContinuousIndex( const HessianTensorType & hessian,
    const ContinuousIndexType & cindex ) const;
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  CircularSectionFluxImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

protected:
  
  /** In adaptative sampling, the number of samples in the circle
    * depends on the current scale, and the radial resolution is ignored. */
  bool                   m_AdaptativeSampling;
  
  /** Calculated flux non-linearly by taking the minimum for each pair of samples diametrally opposed. */
  bool                   m_UseNonLinearFlux;
  
  /** Number of points taken radially. */
  unsigned int           m_RadialResolution;
  
  /** Defines how the opposite circular values are combined when using NonLinearFlux. */
  NonLinearFluxFunction  m_NonLinearFluxFunction;
  
  /** Cached angular values. */
  itk::Array<double>     m_CosArray;
  itk::Array<double>     m_SinArray;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_CircularSectionFluxImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT CircularSectionFluxImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef CircularSectionFluxImageFunction< ITK_TEMPLATE_2 x > \
                                                  CircularSectionFluxImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanCircularSectionFluxImageFunction+-.h"
#endif

//#if ITK_TEMPLATE_TXX
# include "ivanCircularSectionFluxImageFunction.hxx"
//#endif

#endif
