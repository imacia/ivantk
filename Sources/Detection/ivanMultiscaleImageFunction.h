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
// File: ivanMultiscaleImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanMultiscaleImageFunction_h
#define __ivanMultiscaleImageFunction_h

#include "itkImageFunction.h"

#include "ivanImageFunctionInitializerBase.h"


namespace ivan
{

/**
 * \class MultiscaleImageFunction
 * \brief Convert a single-scale discrete Gaussian function into a multiscale function.
 *
 * This class is templated over the input image type and Gaussian image function type. It stores a
 * container of Gaussian image functions of the given type. These can be accessed to set additional
 * properties.
 *
 * The Initialize() method must be called after setting the parameters and before
 * evaluating the function.
 *
 * The class uses an ImageFunctionInitializerBase (or subclass) for the user to provide a means of 
 * initializing each scaled image function. This is called in Initialize().
 *
 * \sa ImageFunction
 */
template <class TScaledImageFunction, class TInputImage, class TOutput, class TCoordRep=double>
class ITK_EXPORT MultiscaleImageFunction :
  public itk::ImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  typedef MultiscaleImageFunction
    <TScaledImageFunction,TInputImage,TOutput,TCoordRep>   Self;
  typedef itk::ImageFunction
    <TInputImage,TOutput,TCoordRep>                        Superclass;
  
  /** Smart pointer typedef support */
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;
  
  typedef TInputImage                 InputImageType;
  typedef TOutput                     OutputType;
  typedef TCoordRep                   CoordRepType;

  typedef typename InputImageType::ConstPointer        InputImageConstPointer;
  
  typedef typename Superclass::InputPixelType          InputPixelType;
  typedef typename Superclass::IndexType               IndexType;
  typedef typename Superclass::ContinuousIndexType     ContinuousIndexType;
  typedef typename Superclass::PointType               PointType;
    
  
  typedef std::vector<double>                          ScaleVectorType;
    
  typedef TScaledImageFunction                         ScaledImageFunctionType;
  typedef typename ScaledImageFunctionType::Pointer    ScaledImageFunctionPointer;
  
  typedef std::vector<ScaledImageFunctionPointer>      ScaledImageFunctionContainerType;
    
  typedef ImageFunctionInitializerBase<TScaledImageFunction,TInputImage>   ScaledImageFunctionInitializerType; 
  typedef typename ScaledImageFunctionInitializerType::Pointer             ScaledImageFunctionInitializerPointer;
    
  enum ScaleStepMethodType
  { 
    EquispacedScaleSteps = 0,
    LogarithmicScaleSteps = 1
  };
    
public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( MultiscaleImageFunction, itk::ImageFunction );
 
  /** Set/Get macros for the minimum scale. */
  itkSetMacro( MinimumScale, double );
  itkGetConstMacro( MinimumScale, double );
  
  /** Set/Get macros for maximum scale. */
  itkSetMacro( MaximumScale, double );
  itkGetConstMacro( MaximumScale, double );

  /** Set/Get macros for Number of Scales */
  itkSetMacro( NumberOfScales, unsigned int );
  itkGetConstMacro( NumberOfScales, unsigned int );
  
  /** Set/Get the method used to generate scale sequence (Equispaced
   * or Logarithmic) */
  itkSetMacro( ScaleStepMethod, ScaleStepMethodType );
  itkGetConstMacro( ScaleStepMethod, ScaleStepMethodType );

  /**Set equispaced sigma step method */
  void SetScaleStepMethodToEquispaced();

  /**Set logartihmic sigma step method */
  void SetScaleStepMethodToLogarithmic();
  
  itkSetObjectMacro( ScaledImageFunctionInitializer, ScaledImageFunctionInitializerType );
  itkGetObjectMacro( ScaledImageFunctionInitializer, ScaledImageFunctionInitializerType );
  itkGetConstObjectMacro( ScaledImageFunctionInitializer, ScaledImageFunctionInitializerType );
  
  void SetInputImage( const InputImageType *inputImage );
  
  /** Initialize the Gaussian kernel. Call this method before evaluating the function.
    * This method MUST be called after any changes to function parameters. */
  virtual void Initialize();
  
  /** Evalutate the  in the given dimension at specified point */
  virtual OutputType Evaluate( const PointType& point ) const;

  /** Evaluate the function at specified Index position */
  virtual OutputType EvaluateAtIndex( const IndexType & index ) const;

  /** Evaluate the function at specified ContinousIndex position */
  virtual OutputType EvaluateAtContinuousIndex( const ContinuousIndexType & index ) const;

protected:

  MultiscaleImageFunction();
  MultiscaleImageFunction( const Self& ){};

  ~MultiscaleImageFunction(){};

  void operator=( const Self& ){};
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  /** Contains the different hessian function for the different scales. */  
  ScaledImageFunctionContainerType        m_ScaledImageFunctionContainer;
  
  /** Provide a means to initialize the scaled image function for each scale. This is called at 
    * initialization for the image functions of all scales. */
  ScaledImageFunctionInitializerPointer   m_ScaledImageFunctionInitializer;
  
  InputImageConstPointer    m_InputImage;

  double                    m_MinimumScale;
  double                    m_MaximumScale;

  unsigned int              m_NumberOfScales;
  ScaleStepMethodType       m_ScaleStepMethod;
  
  /** Sigmas calculated internally. */
  ScaleVectorType           m_Scales;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_MultiscaleImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT MultiscaleImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef MultiscaleImageFunction< ITK_TEMPLATE_2 x > \
                                                  MultiscaleImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanMultiscaleImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanMultiscaleImageFunction.hxx"
#endif

#endif
