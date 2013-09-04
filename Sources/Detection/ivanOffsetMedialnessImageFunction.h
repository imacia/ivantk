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
// File: ivanOffsetMedialnessImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanOffsetMedialnessImageFunction_h
#define __ivanOffsetMedialnessImageFunction_h

#include "ivanHessianBasedVesselnessImageFunction.h"
#include "ivanDiscreteGradientGaussianImageFunction.h"

#include "itkDiscreteGradientMagnitudeGaussianImageFunction.h"
#include "itkArray.h"
#include "itkVector.h"


namespace ivan
{
  
/**
 * \class OffsetMedialnessImageFunction
 * \brief Computes offset medialness in a circle defined by the lowest-eigenvalued eigenvectors.
 *
 * This class computes an offset medialness in the points defined by a circle whose radius depends
 * on the current scale and that lies in the plane defined by the eigenvectors corresponding two
 * the two most negative eigenvalues. 
 *
 * A section normal may be provided to be taken as the plane in which to calculate the radial medialness
 * components that are to be summed. Otherwise, a normal is calculated from the image content as the
 * lowest eigenvalued eigenvector of the Hessian matrix. Then, the section plane is formed by the 
 * other two eigenvalues.
 * 
 * This class is templated over the input image type.
 *
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 * \sa ImageFunction
 */
template <class TInputImage, class TOutput, class TCoordRep=double>
class ITK_EXPORT OffsetMedialnessImageFunction :
  public HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  /**Standard "Self" typedef */
  typedef OffsetMedialnessImageFunction
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
  
  /** Types for gradient calculation. */
  typedef itk::DiscreteGradientMagnitudeGaussianImageFunction
    <TInputImage,TOutput>                                      GradientMagnitudeFunctionType;
  typedef typename GradientMagnitudeFunctionType::Pointer      GradientMagnitudeFunctionPointer;
    
  typedef ivan::DiscreteGradientGaussianImageFunction
    <TInputImage,TOutput>                                      GradientFunctionType;
  typedef typename GradientFunctionType::Pointer               GradientFunctionPointer;
    
    
  /** Interpolation modes */
  typedef typename Superclass::InterpolationModeType           InterpolationModeType;
    
  typedef itk::Vector<double,InputImageType::ImageDimension>   VectorType;
    
  enum GradientImageFunctionType
  {
    GradientNormalProjectionFunctionHyperIntense,
    GradientNormalProjectionFunctionHypoIntense,
    GradientMagnitudeFunction     
  };

public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( OffsetMedialnessImageFunction, HessianBasedVesselnessImageFunction );
  
  itkSetMacro( GradientImageFunctionType, GradientImageFunctionType );
  itkGetConstMacro( GradientImageFunctionType, GradientImageFunctionType );
  
  /** Set/Get the symmetry coefficient. */
  itkSetMacro( SymmetryCoefficient, double );
  itkGetConstMacro( SymmetryCoefficient, double );
  
  /** Set/Get the radius at which offset values are calculated. */
  itkSetMacro( Radius, double );
  itkGetConstMacro( Radius, double );
  
  /** Output threshold. */
  //itkSetMacro( Threshold, double );
  //itkGetConstMacro( Threshold, double );

  itkSetMacro( UseCentralBoundariness, bool );
  itkGetConstMacro( UseCentralBoundariness, bool );
  itkBooleanMacro( UseCentralBoundariness );
  
  /** Set/Get use adaptative sampling. In adaptative sampling, the number of samples in the circle
    * depends on the current scale, and the radial resolution is ignored. */
  virtual void SetAdaptativeSampling( bool adaptative );
  itkGetConstMacro( AdaptativeSampling, bool );
  itkBooleanMacro( AdaptativeSampling );
  
  /** Set/Get the flag to autocompute the section normal using the eigenvectors of the local Hessian matrix.
    * If set to false, the provided section normal will be used. */
  itkSetMacro( AutoComputeSectionNormal, bool );
  itkGetConstMacro( AutoComputeSectionNormal, bool );
  itkBooleanMacro( AutoComputeSectionNormal );
  
  /** Set the section normal when provided by the user. For this AutoComputeSectionNormal must be set to off.
    * Otherwise a section normal would be calculated. */
  itkSetMacro( SectionNormal, VectorType );
  itkGetConstMacro( SectionNormal, VectorType );
  
  /** Set/Get the radial resolution. */
  virtual void SetRadialResolution( const unsigned int resolution );
  itkGetConstMacro( RadialResolution, unsigned int );
  
  /** Set the sigma for the gradient calculations in the circle boundary. In general this sigma should be
    * smaller than the one used to calculate the Hessian. */
  virtual void SetGradientSigma( double gradientSigma );
  
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

  OffsetMedialnessImageFunction();
  ~OffsetMedialnessImageFunction();
  
  /** Precompute values that depends on the radial resolution such as sin or cos values. */
  virtual void ComputeIntervals();
  
  /** Evaluate vesselness given Hessian. This must be reimplemented by subclasses. */
  virtual OutputType EvaluateVesselnessAtContinuousIndex( const HessianTensorType & hessian,
    const ContinuousIndexType & cindex ) const;
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  OffsetMedialnessImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

private:
  
  /** Gradient magnitude image function used to calculate gradients. */
  GradientMagnitudeFunctionPointer  m_GradientMagnitudeFunction;
  
  /** Gradient image function used to calculate gradients. In this case, the projection of the gradient
    * in the radial direction is used. */
  GradientFunctionPointer           m_GradientFunction;
  
  /** Type of gradient function used to calculate gradient. */
  GradientImageFunctionType         m_GradientImageFunctionType;
  
  /** Sigma used for the gradient calculations. In general this sigma should be smaller than the 
    * sigma used for the Hessian. By default is 1.0. */
  double               m_GradientSigma;

  /** Coefficient for taking into account the simmetry in the medialness. */
  double               m_SymmetryCoefficient;
  
  /** Radius at which offset values are calculated. This may not need to coincide with the scale, since
    * the later is used as the sigma/scale for the gradient calcuations. The theory says that the
    * radius should be sqrt(3) times the scale for optimal detection in circular straight tubes. */
  double               m_Radius;
  
  /** Number of points taken radially. */
  unsigned int         m_RadialResolution;

  /** Use central boundariness for adaptive calculations. The central boundariness should be almost
    * zero inside the vessel. */
  bool                 m_UseCentralBoundariness;
  
  /** In adaptative sampling, the number of samples in the circle
    * depends on the current scale, and the radial resolution is ignored. */
  bool                 m_AdaptativeSampling;
  
  /** Flag to autocompute the section normal using the eigenvectors of the local Hessian matrix.
    * If set to false, the provided section normal will be used. */  
  bool                 m_AutoComputeSectionNormal;
  
  /** Section normal. If not provided, the section plane will be calculated using the eigenvectors
    * of the local Hessian matrix. The variable is mutable since, for convenience, it can be changed 
    * in const methods. */
  mutable VectorType   m_SectionNormal;  
  
  /** Cached angular values. */
  itk::Array<double>   m_CosArray;
  itk::Array<double>   m_SinArray;
    
  /** Difference threshold. The difference between the medialness and boundariness must be greater than
    * this value. By default zero. The result is that areas surrounding the medial line result in low
    * medialness values which otherwise would have high values. */
  //OutputType           m_Threshold;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_OffsetMedialnessImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT OffsetMedialnessImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef OffsetMedialnessImageFunction< ITK_TEMPLATE_2 x > \
                                                  OffsetMedialnessImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanOffsetMedialnessImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanOffsetMedialnessImageFunction.hxx"
#endif

#endif
