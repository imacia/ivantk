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
// File: ivanObjectnessMeasureImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanObjectnessMeasureImageFunction_h
#define __ivanObjectnessMeasureImageFunction_h

#include "ivanHessianOnlyBasedVesselnessImageFunction.h"


namespace ivan
{

/**
 * \class ObjectnessMeasureImageFunction
 * \brief 
 *
 * This class is templated over the input image type.
 *
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 * \sa ImageFunction
 */
template <class TInputImage, class TOutput, class TCoordRep=double>
class ITK_EXPORT ObjectnessMeasureImageFunction :
  public HessianOnlyBasedVesselnessImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  /**Standard "Self" typedef */
  typedef ObjectnessMeasureImageFunction
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
  typedef itk::DiscreteHessianGaussianImageFunction
    <TInputImage,TCoordRep>                          HessianFunctionType;
  typedef typename HessianFunctionType::Pointer      HessianFunctionPointer;
  typedef typename HessianFunctionType::TensorType   HessianTensorType;
    
  /** Interpolation modes */
  typedef typename Superclass::InterpolationModeType InterpolationModeType;
  
  /** Image dimension */
  itkStaticConstMacro( ImageDimension, unsigned int, ::itk::GetImageDimension<InputImageType>::ImageDimension );

public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( ObjectnessMeasureImageFunction, HessianOnlyBasedVesselnessImageFunction );
  
  itkSetMacro( Alpha, double );
  itkGetConstMacro( Alpha, double );
  
  itkSetMacro( Beta, double );
  itkGetConstMacro( Beta, double );
  
  itkSetMacro( Gamma, double );
  itkGetConstMacro( Gamma, double );
  
  /** Set/Get the dimensionality of the object (0: points (blobs),
    * 1: lines (vessels), 2: planes (plate-like structures), 3: hyper-planes.
    * ObjectDimension must be smaller than ImageDimension. */
  itkSetMacro( ObjectDimension, unsigned int );
  itkGetConstMacro( ObjectDimension, unsigned int );
  
protected:

  ObjectnessMeasureImageFunction();
  ~ObjectnessMeasureImageFunction();
  
  /** Evaluate vesselness given Hessian. */
  virtual OutputType EvaluateVesselness( const HessianTensorType & hessian ) const;
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  ObjectnessMeasureImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

private:

  /** Parameter corresponding to the Ra term filtering plate-like structures. */
  double         m_Alpha;
  
  /** Parameter corresponding to the Rb term filtering blob-like structures. */
  double         m_Beta;
  
  /** Parameter corresponding to the Frobenius norm term that filters noise. */
  double         m_Gamma;
  
  unsigned int   m_ObjectDimension;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_ObjectnessMeasureImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT ObjectnessMeasureImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef ObjectnessMeasureImageFunction< ITK_TEMPLATE_2 x > \
                                                  ObjectnessMeasureImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanObjectnessMeasureImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanObjectnessMeasureImageFunction.hxx"
#endif

#endif
