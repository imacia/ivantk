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
// File: ivanMedialnessVesselSectionAndRadiusFitCostFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: cost function to fit a circle section (orientation and radius) to a 3D image
// Parameters of the cost function are the radius and the x,y,z coordinates of the unit vector
// that defines the section plane (and its normal to it)


#ifndef __ivanMedialnessVesselSectionAndRadiusFitCostFunction_h
#define __ivanMedialnessVesselSectionAndRadiusFitCostFunction_h

#include <time.h>

#include "itkImage.h"
#include "itkArray.h"
#include "itk_hash_map.h"
#include "itkDiscreteGradientMagnitudeGaussianImageFunction.h"

namespace ivan
{
/** \class MedialnessVesselSectionAndRadiusFitCostFunction
 *  \brief A cost function for optimization of a vessel point radius
 *
 * This class is a cost function for optimization of a vessel point
 * radius and section. The cost function is based on the sum of
 * medialness values at a line passing by the current point and
 * along the vessel current vessel normal. This value is thus a 
 * function of the current radius and vessel normal and can be
 * optimized.
 *
 * Points are taken along the line based on a sampling distance
 * parameter. We assume that in the vecinity of the current point
 * the vessel is straight and thus estimation is not so reliable in 
 * places of high vessel curvature.
 *
 */
template<class TImage, class TScale = float>
class MedialnessVesselSectionAndRadiusFitCostFunction : public SingleValuedCostFunction
{
public:
  /** Standard class typedefs. */
  typedef MedialnessVesselSectionAndRadiusFitCostFunction   Self;
  typedef SingleValuedCostFunction    Superclass;
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;
  
  /** Image related type definitions. */
  typedef TImage ImageType;
  typedef typename ImageType::Pointer     ImagePointer;
  typedef typename ImageType::PixelType   PixelType;
  typedef typename ImageType::IndexType   IndexType;
  typedef typename ImageType::RegionType  RegionType;
  typedef typename ImageType::PointType   PointType;
  typedef typename ImageType::SpacingType SpacingType;
  typedef typename RegionType::SizeType   SizeType;
    
  /** Type used for representing the scale. */
  typedef TScale   ScaleType;
  
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
  	
  /** Image function for gradient calculation. */
  typedef itk::DiscreteGradientMagnitudeGaussianImageFunction
    <SourceImageType, ScaleType>                    GradientFunctionType;
  typedef typename GradientFunctionType::Pointer    GradientFunctionPointer;
  
  /** Hash map for buffering gradient calculations. */
  typedef itk::hash_map<unsigned long,ScaleType>  HashMapType;

public:
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( MedialnessVesselSectionAndRadiusFitCostFunction, CostFunction );

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Specify the input image. */
  void SetImage( const ImageType * image );
  
  /** Set/Get the image region which will be included for calculation.
    * If no region is provided, the RequestedRegion() of the input
    * image is used. */
  itkSetMacro( Region, RegionType );
  itkGetMacro( Region, RegionType );
  
  /** Set/Get the center of the circle. */
  itkSetMacro( Center, PointType );
  itkGetMacro( Center, PointType );
  
  /** Set/Get the sampling distance. */
  itkSetMacro( SamplingDistance, double );
  itkGetMacro( SamplingDistance, double );

  /** Set/Get the symmetry coefficient (from zero to one). */
  itkSetMacro( SymmetryCoefficient, double );
  itkGetMacro( SymmetryCoefficient, double );
  
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
  MedialnessVesselSectionAndRadiusFitCostFunction();

  /** Destructor: */
  virtual ~MedialnessVesselSectionAndRadiusFitCostFunction();


private:
  
  MedialnessVesselSectionAndRadiusFitCostFunction(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented
  
  
private:
  
  /** Input image smart pointer. */
  ImagePointer  m_Image ;

  /** Region of interest. */
  RegionType    m_Region ;
  
  /** Center of the circle. */
  PointType     m_Center;
  
  /** Symmetry coefficient for medialness calculation (from 0.0 to 1.0). */
  double        m_SymmetryCoefficient;
    
  /** Sampling distance. By default this is the minimum spacing. */
  double        m_SamplingDistance;
  
  /** Tolerance in order to avoid recalculation in scaled measurements
    * such as medialness. Default is 0.5. */
  double        m_ScaleTolerance;
  
  /** Current scale used in medialness calculations. */
  double        m_CurrentScale;
  
  /** Hash map used for buffering gradient calculations. */
  HashMapType   m_HashMap;
  
  /** Image function used for gradient calculations. */
  GradientFunctionPointer  m_Gradient;
  
}; // end of class


} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "itkMedialnessVesselSectionAndRadiusFitCostFunction.hxx"
#endif

#endif
