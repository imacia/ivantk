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
// File: ivanHessianBasedVesselnessImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanHessianBasedVesselnessImageFunction_h
#define __ivanHessianBasedVesselnessImageFunction_h


#include "itkImageFunction.h"
#include "ivanDiscreteHessianGaussianImageFunction.h"


namespace ivan
{
  
/**
 * \class HessianBasedVesselnessImageFunction
 * \brief Base class that computes vesselness using local Hessian matrix information.
 *
 * Base class that computes vesselness using local Hessian matrix information, such as that
 * obtained from the eigenanalysis of the local Hessian matrix. Other information, such as
 * the physical location of the point, can be used to calculate vesselness.
 *
 * This class is templated over the input image type.
 *
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 * Subclasses must only reimplement the EvaluateAtIndex() method. The rest of methods perform
 * the interpolation as desired. 
 *
 * \sa ImageFunction
 * \sa DiscreteHessianGaussianImageFunction
 * \sa HessianRecursiveGaussianImageFilter 
 * \sa SymmetricEigenAnalysisImageFilter
 * \sa SymmetricSecondRankTensor
 */
template <class TInputImage, class TOutput, class TCoordRep=double>
class ITK_EXPORT HessianBasedVesselnessImageFunction :
  public itk::ImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  typedef HessianBasedVesselnessImageFunction
    <TInputImage,TOutput,TCoordRep>   Self;
  typedef itk::ImageFunction
    <TInputImage,TOutput,TCoordRep>   Superclass;
  
  /** Smart pointer typedef support */
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;
  
  typedef TInputImage  InputImageType;
  typedef TOutput      OutputType;
  typedef TCoordRep    CoordRepType;
  
  typedef typename Superclass::InputPixelType       InputPixelType;
  typedef typename Superclass::IndexType            IndexType;
  typedef typename Superclass::ContinuousIndexType  ContinuousIndexType;
  typedef typename Superclass::PointType            PointType;
    
  /** Types for Hessian function/matrix. */
  typedef ivan::DiscreteHessianGaussianImageFunction
    <TInputImage,TCoordRep>                         HessianFunctionType;
  typedef typename HessianFunctionType::Pointer     HessianFunctionPointer;
  typedef typename HessianFunctionType::TensorType  HessianTensorType;
  
  /** Necessary for multi-scale compatibility. */
  typedef HessianTensorType                         TensorType;
    
  /** Interpolation modes */
  typedef typename HessianFunctionType::InterpolationModeType 
    InterpolationModeType;

  /** Dimension of the underlying image */
  itkStaticConstMacro(ImageDimension, unsigned int,
                      InputImageType::ImageDimension);

public:

  /** Method for creation through the object factory */
  //itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( HessianBasedVesselnessImageFunction, ImageFunction );
  
  /** Set the scale, that is, the standard deviation of the Gaussian kernel. */
  virtual void SetSigma( double sigma )
    { 
      m_HessianFunction->SetSigma( sigma );
      this->Modified();  
    }
  
  itkGetConstMacro( Sigma, double );
  
  /** Set/Get the desired maximum error of the gaussian kernel approximation used
   * for the Hessian. Maximum error is the difference between the area under the 
   * discrete Gaussian curve and the area under the continuous Gaussian. Maximum error 
   * affects the Gaussian operator size. The value is clamped between 0.00001 and
   * 0.99999. */
  virtual void SetMaximumError( double maxError )
    { m_HessianFunction->SetMaximumError( maxError ); }
  virtual double GetMaximumError() const
    { return m_HessianFunction->GetMaximumError(); }
 
  /** Set/Get the flag for calculating scale-space normalized derivatives in Hessian computation.
   * Normalized derivatives are obtained multiplying by the scale parameter t. */
  virtual void SetNormalizeAcrossScale( bool normalize )
    { m_HessianFunction->SetNormalizeAcrossScale( normalize ); }
  virtual bool GetNormalizeAcrossScale() const
    { return m_HessianFunction->GetNormalizeAcrossScale(); }
  itkBooleanMacro( NormalizeAcrossScale );

  /** Set/Get the gamma for derivative normalization. */
  virtual void SetGamma( double gamma )
    { m_HessianFunction->SetGamma( gamma ); }
  virtual bool GetGamma() const
    { return m_HessianFunction->GetGamma(); }

  /** Set/Get the flag for using image spacing when calculating Hessian derivatives. */
  virtual void SetUseImageSpacing( bool useSpacing )
    { m_HessianFunction->SetUseImageSpacing( useSpacing ); }
  virtual bool GetUseImageSpacing() const
    { return m_HessianFunction->GetUseImageSpacing(); }
  itkBooleanMacro( UseImageSpacing );

  /** Set/Get a limit for growth of the Hessian kernel. Small maximum error values with
   *  large variances will yield very large kernel sizes. This value can be
   *  used to truncate a kernel in such instances. A warning will be given on
   *  truncation of the kernel. */
  virtual void SetMaximumKernelWidth( double maxWidth )
    { m_HessianFunction->SetMaximumKernelWidth( maxWidth ); }
  virtual double GetMaximumKernelWidth() const
    { return m_HessianFunction->GetMaximumKernelWidth(); }

  /** Set/Get the interpolation mode. */
  virtual void SetInterpolationMode( const InterpolationModeType mode )
    { m_HessianFunction->SetInterpolationMode( mode ); }
  virtual InterpolationModeType GetInterpolationMode() const
    { return m_HessianFunction->GetInterpolationMode(); } 

  /** Set the input image.
   * \warning this method caches BufferedRegion information.
   * If the BufferedRegion has changed, user must call
   * SetInputImage again to update cached values. */
  virtual void SetInputImage( const InputImageType * ptr );

  /** Initialize the Gaussian kernel. Call this method before evaluating the function.
   * This method MUST be called after any changes to function parameters. */
  virtual void Initialize() { m_HessianFunction->Initialize(); }
  
  /** Evalutate the  in the given dimension at specified point */
  virtual OutputType Evaluate( const PointType& point ) const;

  /** Evaluate the function at specified Index position */
  virtual OutputType EvaluateAtIndex( const IndexType & index ) const;

  /** Evaluate the function at specified ContinousIndex position */
  virtual OutputType EvaluateAtContinuousIndex( const ContinuousIndexType & index ) const;
  
protected:

  HessianBasedVesselnessImageFunction();
  HessianBasedVesselnessImageFunction( const Self& ){};

  virtual ~HessianBasedVesselnessImageFunction(){};
    
  /** Evaluate vesselness given Hessian. This must be reimplemented by subclasses. */
  virtual OutputType EvaluateVesselnessAtContinuousIndex( const HessianTensorType & hessian,
    const ContinuousIndexType & cindex ) const = 0;
  
  /** Evaluate vesselness given Hessian. This version simply converts the index to a continuous
    * index and calls EvaluateVesselnessAtContinuousIndex(). Subclasses may reimplement faster
    * approacher at center locations. */
  virtual OutputType EvaluateVesselnessAtIndex( const HessianTensorType & hessian,
    const IndexType & index ) const;
  
  /** Evaluate vesselness given Hessian. The default implementation translates the closest point
    * to a continuous index and performs the desired interpolation. */
  virtual OutputType EvaluateVesselness( const HessianTensorType & hessian,
    const PointType& point ) const;
        
  /** Evalutate the  in the given dimension at specified point */
  HessianTensorType EvaluateHessian( const PointType& point ) const;

  /** Evaluate the function at specified Index position */
  HessianTensorType EvaluateHessianAtIndex( const IndexType & index ) const;

  /** Evaluate the function at specified ContinousIndex position */
  HessianTensorType EvaluateHessianAtContinuousIndex(
    const ContinuousIndexType & index ) const;

  void operator=( const Self& ){};
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

protected:

  /** Image function used to calculate hessian. */
  HessianFunctionPointer  m_HessianFunction;
  
  /** Current scale, which is the standard deviation for Gaussian kernel. */
  double                  m_Sigma;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_HessianBasedVesselnessImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT HessianBasedVesselnessImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef HessianBasedVesselnessImageFunction< ITK_TEMPLATE_2 x > \
                                                  HessianBasedVesselnessImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanHessianBasedVesselnessImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanHessianBasedVesselnessImageFunction.hxx"
#endif

#endif
