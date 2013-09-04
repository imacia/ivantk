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
// File: ivanPolarProfileVesselnessImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/08/17

#ifndef __ivanPolarProfileVesselnessImageFunction_h
#define __ivanPolarProfileVesselnessImageFunction_h

#include "ivanSphereGridBasedImageFunction.h"

#include "itkImageFunction.h"
#include "itkLinearInterpolateImageFunction.h"
#include "ivanDiscreteHessianGaussianImageFunction.h"

namespace ivan
{
  
/**
 * \class PolarProfileVesselnessImageFunction
 * \brief Base class that computes vesselness by partitioning space in polar coordinates and examinining intensity profile.
 *
 * Base class that computes vesselness by partitioning space in polar coordinates and obtaining a
 * function of the intensity profiles in each partition.
 *
 * This base class performs partitioning of space in polar coordinates. Subclasses must define
 * how vesselness is calculated according to the corresponding profiles.
 *
 */
template <class TInputImage, class TOutput, class TCoordRep=double>
class ITK_EXPORT PolarProfileVesselnessImageFunction :
  public SphereGridBasedImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  typedef PolarProfileVesselnessImageFunction
    <TInputImage,TOutput,TCoordRep>   Self;
  typedef SphereGridBasedImageFunction
    <TInputImage,TOutput,TCoordRep>   Superclass;
  
  /** Smart pointer typedef support */
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;
  
  typedef TInputImage  InputImageType;
  typedef TOutput      OutputType;
  typedef TCoordRep    CoordRepType;
  
  typedef typename Superclass::InputPixelType        InputPixelType;
  typedef typename Superclass::IndexType             IndexType;
  typedef typename Superclass::ContinuousIndexType   ContinuousIndexType;
  typedef typename Superclass::PointType             PointType;
    
  typedef typename Superclass::SphereSourceType      SphereSourceType;
  typedef typename Superclass::SphereSourcePointer   SphereSourcePointer;
  typedef typename Superclass::SphereType            SphereType;
  typedef typename Superclass::SpherePointer         SpherePointer;
  
  typedef itk::LinearInterpolateImageFunction
    <TInputImage>                                    InterpolatorType;
  typedef typename InterpolatorType::Pointer         InterpolatorPointer;
    
  enum SpatialWeightingType
  {
    UniformSpatialWeighting,
    GaussianSpatialWeighting
  };
    
public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( PolarProfileVesselnessImageFunction, itk::ImageFunction );
  
  itkSetMacro( SpatialWeightingType, SpatialWeightingType );
  itkGetConstMacro( SpatialWeightingType, SpatialWeightingType );
  
  itkSetMacro( Beta, double );
  itkGetConstMacro( Beta, double );
  
  itkSetMacro( Sigma, double );
  itkGetConstMacro( Sigma, double );

  itkSetMacro( Tau, double );
  itkGetConstMacro( Tau, double );
  
  itkSetMacro( Gamma, double );
  itkGetConstMacro( Gamma, double );
      
  /** Set the input image.
   * \warning this method caches BufferedRegion information.
   * If the BufferedRegion has changed, user must call
   * SetInputImage again to update cached values. */
  virtual void SetInputImage( const InputImageType * ptr );

  /** Evalutate the  in the given dimension at specified point */
  virtual OutputType Evaluate( const PointType& point ) const;

  /** Evaluate the function at specified Index position */
  virtual OutputType EvaluateAtIndex( const IndexType & index ) const;

  /** Evaluate the function at specified ContinousIndex position */
  virtual OutputType EvaluateAtContinuousIndex( const ContinuousIndexType & index ) const;
  
protected:

  PolarProfileVesselnessImageFunction();
  virtual ~PolarProfileVesselnessImageFunction() {};
    
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  PolarProfileVesselnessImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

protected:

  SpatialWeightingType   m_SpatialWeightingType;
  
  /** Image interpolator used to obtain image values. */
  InterpolatorPointer    m_Interpolator;
  
  /** Beta parameter for distribution. This multiplies the exponential term. */
  double                 m_Beta;
  
  /** Sigma for distribution. This is the standard deviation of the image or region of interest, in
    * order to make the beta parameter adaptative. Default value is 1.0 */
  double                 m_Sigma;
  
  /** Tau parameter for distribution. This multiplies the entropy in the final exponential. */
  double                 m_Tau;
  
  /** Gamma parameter for the exponential that makes the brightness term. */
  double                 m_Gamma;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_PolarProfileVesselnessImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT PolarProfileVesselnessImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef PolarProfileVesselnessImageFunction< ITK_TEMPLATE_2 x > \
                                                  PolarProfileVesselnessImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanPolarProfileVesselnessImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanPolarProfileVesselnessImageFunction.hxx"
#endif

#endif
