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
// File: ivanOffsetMedialnessVesselSectionFitCostFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2011/11/05


#ifndef __ivanOffsetMedialnessVesselSectionFitCostFunction_h
#define __ivanOffsetMedialnessVesselSectionFitCostFunction_h

#include "ivanImageBasedVesselSectionFitCostFunction.h"
#include "ivanImageFunctionToCostFunctionAdaptor.h"
#include "ivanOffsetMedialnessImageFunction.h"

#include "itkImage.h"


namespace ivan
{
  
/** \class OffsetMedialnessVesselSectionFitCostFunction
 *  \brief Cost functions for calculating optimized vessel sections based on offset medialness
 *
 * This class represents a cost function for calculating a vessel section based on obtaining
 * an offset medialness value. This value is obtained as the sum of the gradients at an offset
 * location, which corresponds to a circle around the center point in the current vessel plane.
 *
 * Different from the OffsetMedialnessImageFunction
 *
 *   
 */
template <class TImage>
class OffsetMedialnessVesselSectionFitCostFunction : public ImageBasedVesselSectionFitCostFunction<TImage>
{
public:
  
  /** Standard class typedefs. */
  typedef OffsetMedialnessVesselSectionFitCostFunction<TImage>   Self;
  typedef ImageBasedVesselSectionFitCostFunction<TImage>         Superclass;
  typedef itk::SmartPointer<Self>                                Pointer;
  typedef itk::SmartPointer<const Self>                          ConstPointer;
  
  /** Image related type definitions. */
  typedef TImage                           ImageType;
  typedef typename ImageType::Pointer    	 ImagePointer;
  typedef typename ImageType::PixelType  	 PixelType;
  typedef typename ImageType::IndexType  	 IndexType;
  typedef typename ImageType::RegionType 	 RegionType;
  typedef typename ImageType::PointType  	 PointType;
  typedef typename ImageType::SpacingType  SpacingType;
  typedef typename RegionType::SizeType	 	 SizeType;
  
  /** Constants for the image dimensions */
  itkStaticConstMacro( ImageDimension, unsigned int, ImageType::ImageDimension );
  
  /** Parameters type for optimizer. */
  typedef typename Superclass::ParametersType         ParametersType;
  
  /** Type used for representing point components  */
  typedef typename Superclass::ParametersValueType    CoordinateRepresentationType;
  
    /** Not used, but expected by SingleValuedNonLinearOptimizer class. */
  typedef typename Superclass::DerivativeType         DerivativeType;

  /** The cost value type. */
  typedef typename Superclass::MeasureType            MeasureType;
  
  /** Offset medialness function adapted as a cost function. */
	typedef OffsetMedialnessImageFunction
		<ImageType, MeasureType>                          MedialnessFunctionType;
	typedef typename MedialnessFunctionType::Pointer    MedialnessFunctionPointer;
	  
	typedef typename MedialnessFunctionType::PointType  CenterPointType;
	
public:
  
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
    
  /** Run-time type information (and related methods). */
  itkTypeMacro( OffsetMedialnessVesselSectionFitCostFunction, ImageBasedVesselSectionFitCostFunction );
  
  itkSetMacro( MaxRadius, double );
  itkGetConstMacro( MaxRadius, double );
  
  /** Set/Get the medialness function and its properties. */
  itkSetObjectMacro( MedialnessFunction, MedialnessFunctionType );
  itkGetObjectMacro( MedialnessFunction, MedialnessFunctionType );
  itkGetConstObjectMacro( MedialnessFunction, MedialnessFunctionType );
  
  /** Return the number of parameters. These are [nx ny nz r lag1] where [nx ny nz] are the
    * components of the normal, r is the radius used for the medialness calculation and
    * lag1 is the Lagrangian parameter for the constraint nx^2 + ny^2 + nz^2 = 1 */
  virtual unsigned int GetNumberOfParameters() const
    { return 3; }
  
  /** Initialize the Cost Function by making sure that all the components
    *  are initialized correctly. */
  virtual void Initialize();
  
  /** Gets the cost function value. */
  virtual MeasureType GetValue(const ParametersType & parameters ) const ;

  /** Dummy implementation to confirm to the SingleValuedCostFunction 
   * interfaces. It is pure virtual in the superclass */
  virtual void GetDerivative( const ParametersType & parameters,
    DerivativeType & derivative ) const
    {
      itkWarningMacro( "GetDerivative() method not implemented yet." );
    }

	//virtual void GetValueAndDerivative( const ParametersType &parameters, 
		//MeasureType &value, DerivativeType &derivative) const;

  //virtual unsigned int GetNumberOfParameters(void) const;
  
protected:
  
  /** Constructor: */
  OffsetMedialnessVesselSectionFitCostFunction();

  /** Destructor: */
  virtual ~OffsetMedialnessVesselSectionFitCostFunction();
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;
    
  MeasureType ComputeMedialness();

private:
  
  OffsetMedialnessVesselSectionFitCostFunction(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented
    
private:
  
  MedialnessFunctionPointer   m_MedialnessFunction;
  
  /** Maximum radius allowed for the medialness calculations. */
  double                      m_MaxRadius; 
}; // end of class

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanOffsetMedialnessVesselSectionFitCostFunction.hxx"
#endif

#endif
