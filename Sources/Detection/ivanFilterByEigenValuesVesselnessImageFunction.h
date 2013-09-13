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
// File: ivanFilterByEigenValuesVesselnessImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/08/16

#ifndef __ivanFilterByEigenValuesVesselnessImageFunction_h
#define __ivanFilterByEigenValuesVesselnessImageFunction_h

#include "ivanHessianOnlyBasedVesselnessImageFunction.h"


namespace ivan
{
  
/**
 * \class FilterByTwoNegativeEigenValuesFunctor
 * \brief Default functor that filters locations where at least one of the lowest valued eigenvalues is non-negative.
 *
 * Default functor that filters locations where at least one of the lowest valued 
 * eigenvalues is non-negative. It is assumed that the eigenvalues come in order,
 * with the first eigenvalue in the array being the smallest. If there are negative
 * eigenvalues this would be the most non-negative. 
 *
 * \sa FilterByEigenValuesVesselnessImageFunction
 * 
 */ 
template<class TCoordRep=double>
class FilterByTwoNegativeEigenValuesFunctor
{
public:
  
  typedef itk::SymmetricSecondRankTensor<TCoordRep>          HessianTensorType;
  typedef typename HessianTensorType::EigenValuesArrayType   EigenValuesArrayType;
  
  FilterByTwoNegativeEigenValuesFunctor() {};
  ~FilterByTwoNegativeEigenValuesFunctor() {};
  
  inline bool operator() ( const EigenValuesArrayType & eigenValues ) const
  {
    if( eigenValues[0] < 0.0 && eigenValues[1] < 0.0 )
      return true;
    else
      return false;
  }
};


/**
 * \class FilterByEigenValuesVesselnessImageFunction
 * \brief Image function that produces a non-zero response depending on a function of Hessian-matrix eigenvalues.
 *
 * This class produces a non-zero response depending on the result of a function of Hessian-matrix 
 * eigenvalues. This function is specified as a functor template parameter that can be defined by the 
 * user. The default implementation filters all locations where there are not at least two negative
 * eigenvalues.
 * 
 * This class is templated over the input image type.
 *
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 * \sa ImageFunction
 * 
 */
template <class TInputImage, class TOutput, class TCoordRep=double,
  class TFunctor = FilterByTwoNegativeEigenValuesFunctor<TCoordRep> >
class ITK_EXPORT FilterByEigenValuesVesselnessImageFunction :
  public HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  /**Standard "Self" typedef */
  typedef FilterByEigenValuesVesselnessImageFunction
    <TInputImage,TOutput,TCoordRep>            Self;
  typedef HessianOnlyBasedVesselnessImageFunction
    <TInputImage,TOutput,TCoordRep>            Superclass;

  /** Smart pointer typedef support */
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef TInputImage  InputImageType;
  typedef TOutput      OutputType;
  typedef TCoordRep    CoordRepType;
  typedef TFunctor     FunctorType;
  
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
  itkTypeMacro( FilterByEigenValuesVesselnessImageFunction, HessianOnlyBasedVesselnessImageFunction );
  
  itkSetMacro( OutputValue, TCoordRep );
  itkGetConstMacro( OutputValue, TCoordRep );
  
  /** Get the functor object.  The functor is returned by reference.
   * (Functors do not have to derive from itk::LightObject, so they do
   * not necessarily have a reference count. So we cannot return a
   * itk::SmartPointer.) */
  FunctorType & GetFunctor() { return m_Functor; }

  /** Get the functor object.  The functor is returned by reference.
   * (Functors do not have to derive from itk::LightObject, so they do
   * not necessarily have a reference count. So we cannot return a
   * itk::SmartPointer.) */
  const FunctorType & GetFunctor() const
    {
    return m_Functor;
    }

  /** Set the functor object.  This replaces the current Functor with a
   * copy of the specified Functor. This allows the user to specify a
   * functor that has ivars set differently than the default functor.
   * This method requires an operator!=() be defined on the functor
   * (or the compiler's default implementation of operator!=() being
   * appropriate). */
  void SetFunctor( const FunctorType & functor )
    {
    if (m_Functor != functor)
      {
      m_Functor = functor;
      this->Modified();
      }
    }
  
protected:

  FilterByEigenValuesVesselnessImageFunction();
  ~FilterByEigenValuesVesselnessImageFunction();
  
  /** Evaluate vesselness given Hessian. */
  virtual OutputType EvaluateVesselness( const HessianTensorType & hessian ) const;
  
  void PrintSelf( std::ostream& os, itk::Indent indent) const;

private:
  
  FilterByEigenValuesVesselnessImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

private:

  TCoordRep    m_OutputValue;  
  FunctorType  m_Functor;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_FilterByEigenValuesVesselnessImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT FilterByEigenValuesVesselnessImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef FilterByEigenValuesVesselnessImageFunction< ITK_TEMPLATE_2 x > \
                                                  FilterByEigenValuesVesselnessImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanFilterByEigenValuesVesselnessImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanFilterByEigenValuesVesselnessImageFunction.hxx"
#endif

#endif
