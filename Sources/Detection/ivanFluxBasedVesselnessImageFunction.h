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
// File: ivanFluxBasedVesselnessImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanFluxBasedVesselnessImageFunction_h
#define __ivanFluxBasedVesselnessImageFunction_h

#include "ivanHessianBasedVesselnessImageFunction.h"
#include "ivanDiscreteGradientGaussianImageFunction.h"

#include "itkArray.h"
#include "itkVector.h"

namespace ivan
{
  
/**
 * \class FluxBasedVesselnessImageFunction
 * \brief Base class that computes vesselness as some type of flux in the current section plane.
 *
 * This class computes an vesselness as an inward flux in the current section plane. The flux 
 * is calculated as the product of the gradient vector and the orientation of the surface. 
 * This flux is maximized when the surface is aligned with the gradient vector field. Thus, it
 * can be used for tube detection since at the center of the tube, with a correct estimation
 * of the surface normals and a proper scale, the flux will be maximum.
 *
 * Subclasses differ in how this flux is calculated, the number of samples an linearity or not.
 * 
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 * \sa ImageFunction
 */
template <class TInputImage, class TOutput, class TCoordRep=double>
class ITK_EXPORT FluxBasedVesselnessImageFunction :
  public HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  /**Standard "Self" typedef */
  typedef FluxBasedVesselnessImageFunction
    <TInputImage,TOutput,TCoordRep>            Self;
  typedef HessianBasedVesselnessImageFunction
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
  
  /** Types for gradient vector calculation. */
  typedef DiscreteGradientGaussianImageFunction
    <TInputImage,TOutput>                                 GradientFunctionType;
  typedef typename GradientFunctionType::Pointer          GradientFunctionPointer;
    
  /** Interpolation modes */
  typedef typename Superclass::InterpolationModeType      InterpolationModeType;
    
  typedef itk::Vector<double,InputImageType::ImageDimension>   VectorType;

public:

  /** Method for creation through the object factory */
  //itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( FluxBasedVesselnessImageFunction, HessianBasedVesselnessImageFunction );
  
  /** Set/Get the radius factor. */
  itkSetMacro( RadiusFactor, double );
  itkGetConstMacro( RadiusFactor, double );
  
  /** Set the scale, that is, the standard deviation of the Gaussian kernel. Reimplemented to pass
    * the scale to the gradient function too. */
  virtual void SetSigma( double sigma );
  
  /** Set the input image.
   * \warning this method caches BufferedRegion information.
   * If the BufferedRegion has changed, user must call
   * SetInputImage again to update cached values. */
  virtual void SetInputImage( const InputImageType * ptr );
  
  /** Reimplemented to initialize the gradient function also. */
  virtual void Initialize();

protected:

  FluxBasedVesselnessImageFunction();
  ~FluxBasedVesselnessImageFunction();
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  FluxBasedVesselnessImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

protected:
  
  /** Image function used to calculate gradients. */
  GradientFunctionPointer  m_GradientFunction;

  /** Radius factor. This value is multiplied by the current scale to obtain the distance 
    * (radius) at which points are sampled in a circle (by default sqrt(3.0) which gives the 
    * maximum value for medialness. */
  double                   m_RadiusFactor;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_FluxBasedVesselnessImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT FluxBasedVesselnessImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef FluxBasedVesselnessImageFunction< ITK_TEMPLATE_2 x > \
                                                  FluxBasedVesselnessImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanFluxBasedVesselnessImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanFluxBasedVesselnessImageFunction.hxx"
#endif

#endif
