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
// File: ivanHessianOnlyBasedVesselnessImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/19

#ifndef __ivanHessianOnlyBasedVesselnessImageFunction_h
#define __ivanHessianOnlyBasedVesselnessImageFunction_h


#include "ivanHessianBasedVesselnessImageFunction.h"


namespace ivan
{
  
/**
 * \class HessianOnlyBasedVesselnessImageFunction
 * \brief Base class that computes vesselness using only the local Hessian matrix information.
 *
 * Base class that computes vesselness using local Hessian matrix information, such as that
 * obtained from the eigenanalysis of the local Hessian matrix. The difference from the parent
 * class is that this class is intended to be inherited by classes that use exclusively the Hessian
 * matrix information, and not other information such as the location of the current point. 
 * Subclasses must only reimplement a version of EvaluateVesselness() that receives the Hessian
 * matrix information only.
 *
 * This class is templated over the input image type.
 *
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 *
 */
template <class TInputImage, class TOutput, class TCoordRep=double>
class ITK_EXPORT HessianOnlyBasedVesselnessImageFunction :
  public HessianBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  typedef HessianOnlyBasedVesselnessImageFunction
    <TInputImage,TOutput,TCoordRep>   Self;
  typedef HessianBasedVesselnessImageFunction
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
    
  /** Interpolation modes */
  typedef typename HessianFunctionType::InterpolationModeType 
    InterpolationModeType;

public:

  /** Method for creation through the object factory */
  //itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( HessianOnlyBasedVesselnessImageFunction, HessianBasedVesselnessImageFunction );
  
protected:

  HessianOnlyBasedVesselnessImageFunction();
  HessianOnlyBasedVesselnessImageFunction( const Self& ){};

  virtual ~HessianOnlyBasedVesselnessImageFunction(){};
    
  /** Evaluate vesselness given Hessian. Reimplemented to call the version without point location. */
  virtual OutputType EvaluateVesselnessAtContinuousIndex( const HessianTensorType & hessian,
    const ContinuousIndexType & cindex ) const ;
  
  /** Evaluate vesselness given Hessian. Reimplemented to call the version without point location. */
  virtual OutputType EvaluateVesselnessAtIndex( const HessianTensorType & hessian,
    const IndexType & index ) const;
  
  /** Evaluate vesselness given Hessian. Reimplemented to call the version without point location. */
  virtual OutputType EvaluateVesselness( const HessianTensorType & hessian,
    const PointType& point ) const;
  
  /** Evaluate vesselness using Hessian information only. This must be reimplemented by subclasses. */
  virtual OutputType EvaluateVesselness( const HessianTensorType & hessian ) const = 0; 
                
  void operator=( const Self& ){};
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

protected:

};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_HessianOnlyBasedVesselnessImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT HessianOnlyBasedVesselnessImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef HessianOnlyBasedVesselnessImageFunction< ITK_TEMPLATE_2 x > \
                                                  HessianOnlyBasedVesselnessImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanHessianOnlyBasedVesselnessImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanHessianOnlyBasedVesselnessImageFunction.hxx"
#endif

#endif
