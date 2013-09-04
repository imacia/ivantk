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
// File: ivanImageToCircleFitCostFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: cost function to fit a circle to a 2D image


#ifndef __ivanImageToCircleFitCostFunction_h
#define __ivanImageToCircleFitCostFunction_h

#include <time.h>

#include "itkImage.h"
#include "itkArray2D.h"
#include "itkOnePlusOneEvolutionaryOptimizer.h"
#include "itkArray.h"
#include "itkInterpolateImageFunction.h"


namespace ivan
{
/** \class ImageToCircleFitCostFunction
 *  \brief a cost function for optimization
 *
 *   
 */
template<class TImage>
class ImageToCircleFitCostFunction : public SingleValuedCostFunction
{
public:
  /** Standard class typedefs. */
  typedef ImageToCircleFitCostFunction   Self;
  typedef SingleValuedCostFunction       Superclass;
  typedef itk::SmartPointer<Self>             Pointer;
  typedef itk::SmartPointer<const Self>       ConstPointer;
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( ImageToCircleFitCostFunction, CostFunction );

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Image related type definitions. */
  typedef TImage ImageType ;
  typedef typename ImageType::Pointer    ImagePointer ;
  typedef typename ImageType::PixelType  PixelType;
  typedef typename ImageType::IndexType  IndexType;
  typedef typename ImageType::RegionType RegionType;
  typedef typename ImageType::PointType  PointType;
  
  /** Constants for the image dimensions */
  itkStaticConstMacro( ImageDimension, unsigned int, ImageType::ImageDimension);
  
  /** Parameters type for optimizer. */
  typedef Superclass::ParametersType      ParametersType ;
  
  /** Type used for representing point components  */
  typedef Superclass::ParametersValueType CoordinateRepresentationType;
  
    /** Not used, but expected by SingleValuedNonLinearOptimizer class. */
  typedef Superclass::DerivativeType      DerivativeType;

  /** The cost value type. */
  typedef Superclass::MeasureType         MeasureType;
  
  /** Interpolator type. */
	typedef InterpolateImageFunction
		< ImageType, CoordinateRepresentationType >   InterpolatorType;
	typedef typename InterpolatorType::Pointer      InterpolatorPointer;
	

  /** Specify the input image. */
  itkSetObjectMacro( Image, ImageType );
  
  /** Set/Get the image region which will be included for calculation. */
  itkSetMacro( Region, RegionType );
  itkGetMacro( Region, RegionType );
  
  /** Set/Get the center of the circle. */
  itkSetMacro( Center, PointType );
  itkGetMacro( Center, PointType );
  
  /** Set/Get the symmetry coefficient (from zero to one). */
  itkSetMacro( SymmetryCoefficient, double );
  itkGetMacro( SymmetryCoefficient, double );
  
  /** Set/Get the Interpolator. */
  itkSetObjectMacro( Interpolator, InterpolatorType );
	itkGetObjectMacro( Interpolator, InterpolatorType );
  
  /** Gets the cost function value. */
  virtual MeasureType GetValue(const ParametersType & parameters ) const ;

  /** Dummy implementation to confirm to the SingleValuedCostFunction 
   * interfaces. It is pure virtual in the superclass */
  virtual void GetDerivative( const ParametersType & parameters,
    DerivativeType & derivative ) const;

	virtual void GetValueAndDerivative( const ParametersType &parameters, 
		MeasureType &value, DerivativeType &derivative) const;

  virtual unsigned int GetNumberOfParameters(void) const;
  
  /** Initialize the Cost Function by making sure that all the components
   *  are present and plugged together correctly     */
  virtual void Initialize(void) throw ( ExceptionObject );

protected:
  /** Constructor: */
  ImageToCircleFitCostFunction();

  /** Destructor: */
  virtual ~ImageToCircleFitCostFunction();


private:
  
  ImageToCircleFitCostFunction(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented
  
  
private:
  
  /** Input image smart pointer. */
  ImagePointer      m_Image ;

  /** Region of interest. */
  RegionType        m_Region ;
  
  /** Interpolator used to interpolate circle values. */
  InterpolatorPointer    m_Interpolator;
  
  /** Center of the circle. */
  PointType							 m_Center;  
  
  /** Symmetry coefficient for medialness calculation (from 0.0 to 1.0). */
  double						m_SymmetryCoefficient;

  
  
}; // end of class


} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "itkImageToCircleFitCostFunction.hxx"
#endif

#endif
