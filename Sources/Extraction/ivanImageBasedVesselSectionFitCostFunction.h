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
// File: ivanImageBasedVesselSectionFitCostFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2011/11/05


#ifndef __ivanImageBasedVesselSectionFitCostFunction_h
#define __ivanImageBasedVesselSectionFitCostFunction_h

#include "ivanVesselSectionFitCostFunction.h"

#include "itkImage.h"


namespace ivan
{
  
/** \class ImageBasedVesselSectionFitCostFunction
 *  \brief Base class of cost functions for calculationg optimized vessel sections
 *
 * This is the base class of cost functions that is to be maximized or minimized by calculating
 * an optimal set of parameters of the function. Typical parameters are the normal and center of
 * the section. The function must be designed so it gives a minimum (maximum) when the parameters
 * approximate well the real vessel section.
 *
 *   
 */
template<class TImage>
class ImageBasedVesselSectionFitCostFunction : public VesselSectionFitCostFunction
{
public:
  /** Standard class typedefs. */
  typedef ImageBasedVesselSectionFitCostFunction   Self;
  typedef VesselSectionFitCostFunction             Superclass;
  typedef itk::SmartPointer<Self>                  Pointer;
  typedef itk::SmartPointer<const Self>            ConstPointer;
  
  /** Image related type definitions. */
  typedef TImage ImageType;
  typedef typename ImageType::Pointer    	ImagePointer;
  typedef typename ImageType::PixelType  	PixelType;
  typedef typename ImageType::IndexType  	IndexType;
  typedef typename ImageType::RegionType 	RegionType;
  typedef typename ImageType::PointType  	PointType;
  typedef typename ImageType::SpacingType SpacingType;
  typedef typename RegionType::SizeType	 	SizeType;
  
  /** Constants for the image dimensions */
  itkStaticConstMacro( ImageDimension, unsigned int, ImageType::ImageDimension );
  
  /** Parameters type for optimizer. */
  typedef Superclass::ParametersType      ParametersType ;
  
  /** Type used for representing point components  */
  typedef Superclass::ParametersValueType CoordinateRepresentationType;
  
    /** Not used, but expected by SingleValuedNonLinearOptimizer class. */
  typedef Superclass::DerivativeType      DerivativeType;

  /** The cost value type. */
  typedef Superclass::MeasureType         MeasureType;
	
public:
  
  /** Method for creation through the object factory. */
  //itkNewMacro(Self);
    
  /** Run-time type information (and related methods). */
  itkTypeMacro( ImageBasedVesselSectionFitCostFunction, VesselSectionFitCostFunction );
  
  /** Specify the input image. */
  itkSetObjectMacro( Image, ImageType );
  
  /** Connect the Interpolator. */
  //itkSetObjectMacro( Interpolator, InterpolatorType );

  /** Get a pointer to the Interpolator.  */
  //itkGetObjectMacro( Interpolator, InterpolatorType );
  
  /** Gets the cost function value. */
  //virtual MeasureType GetValue(const ParametersType & parameters ) const ;

  /** Dummy implementation to confirm to the SingleValuedCostFunction 
   * interfaces. It is pure virtual in the superclass */
  //virtual void GetDerivative( const ParametersType & parameters,
    //DerivativeType & derivative ) const;

	//virtual void GetValueAndDerivative( const ParametersType &parameters, 
		//MeasureType &value, DerivativeType &derivative) const;

  //virtual unsigned int GetNumberOfParameters(void) const;
  
  /** Initialize the Cost Function by making sure that all the components
   *  are present and plugged together correctly     */
  //virtual void Initialize(void) throw ( ExceptionObject );

protected:
  /** Constructor: */
  ImageBasedVesselSectionFitCostFunction();

  /** Destructor: */
  virtual ~ImageBasedVesselSectionFitCostFunction();

private:
  
  ImageBasedVesselSectionFitCostFunction(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented
    
protected:
  
  /** Input image smart pointer. */
  ImagePointer      m_Image ;

  /** Region of interest. */
  //RegionType        m_Region ;
}; // end of class

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanImageBasedVesselSectionFitCostFunction.hxx"
#endif

#endif
