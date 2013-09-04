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
// File: ivanOffsetMedialnessImageFunctionInitializer.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2012/02/07


#ifndef __ivanOffsetMedialnessImageFunctionInitializer_h
#define __ivanOffsetMedialnessImageFunctionInitializer_h


#include "ivanImageFunctionInitializerBase.h"
#include "ivanOffsetMedialnessImageFunction.h"


#define OFFSET_MEDIALNESS_INITIALIZER_FIXED_SIGMA  1.0
#define OFFSET_MEDIALNESS_INITIALIZER_FIXED_RADIUS 1.0


namespace ivan
{
  
/** \class OffsetMedialnessImageFunctionInitializer
 *  \brief Initializer for offset medialness metric functions.
 *
 * Specific initialization for offset medialness, where the scale affects the radius of the medialness
 * calculation and not the scale of the Hessian matrix used to compute the plane.
 *
 */
template <class TImage, class TOutput = double>
class ITK_EXPORT OffsetMedialnessImageFunctionInitializer : 
  public ImageFunctionInitializerBase<OffsetMedialnessImageFunction<TImage,TOutput>,TImage>
{
public:
  
  typedef OffsetMedialnessImageFunctionInitializer
    <TImage, TOutput>                                         Self;
  typedef ImageFunctionInitializerBase
    <OffsetMedialnessImageFunction<TImage,TOutput>,TImage>    Superclass;
  typedef itk::SmartPointer<Self>                             Pointer;
  typedef itk::SmartPointer<const Self>                       ConstPointer;

  typedef TImage                                              ImageType;
  typedef OffsetMedialnessImageFunction<TImage,TOutput>       ImageFunctionType;
  
  typedef typename ImageFunctionType::GradientImageFunctionType    GradientImageFunctionType; 
  
  enum ScaleSelectionMethodType
  {
    UseScaleAsRadius, // use initialization scale for radius
    UseScaleAsSigma,  // use initialization scale for sigma
    IgnoreScale       // and set sigma and radius to the provided values  
  };
    
public:
  
  itkNewMacro( Self );
  itkTypeMacro( OffsetMedialnessImageFunctionInitializer, ImageFunctionInitializerBase );
  
  void SetScaleSelectionMethod( ScaleSelectionMethodType scaleSelection )
    { this->m_ScaleSelectionMethod = scaleSelection; }
  ScaleSelectionMethodType GetScaleSelectionMethod() const
    { return this->m_ScaleSelectionMethod; }

  void SetHessianSigma( double sigma )
    { this->m_HessianSigma = sigma; }
  double GetHessianSigma() const
    { return this->m_HessianSigma; }

  void SetRadius( double radius )
    { this->m_Radius = radius; }
  double GetRadius() const
    { return this->m_Radius; }
    
  void SetGradientSigma( double gradientSigma )
    { this->m_GradientSigma = gradientSigma; }
  double GetGradientSigma() const
    { return this->m_GradientSigma; }
    
  void SetGradientImageFunctionType( GradientImageFunctionType func )
    { m_GradientImageFunctionType = func; }
  GradientImageFunctionType GetGradientImageFunctionType() const
    { return m_GradientImageFunctionType; }
  
  void SetSymmetryCoefficient( double coeff )
    {
      if( coeff < 0.0 )
      {
        itkWarningMacro( "Symmetry Coefficient cannot be less than zero. Setting to 0.0." );
        m_SymmetryCoefficient = 0.0;
      }
      else if( coeff > 1.0 )
      {
        itkWarningMacro( "Symmetry Coefficient cannot be less than zero. Setting to 1.0." );
        m_SymmetryCoefficient = 1.0;
      }
      else
      {
        m_SymmetryCoefficient = coeff;
      }
    }
    
  virtual void Initialize( ImageFunctionType *imageFunction, const ImageType *image = 0, double scale = 0.0 )
    {
      imageFunction->SetInputImage( image );
      
      switch( this->m_ScaleSelectionMethod )
      {
        case UseScaleAsRadius:
        
          imageFunction->SetSigma( this->m_HessianSigma );
          imageFunction->SetRadius( scale );
          break;
          
        case UseScaleAsSigma:
        
          imageFunction->SetSigma( scale );
          imageFunction->SetRadius( this->m_Radius );
          break;
          
        case IgnoreScale:
        
          imageFunction->SetSigma( this->m_HessianSigma );
          imageFunction->SetRadius( this->m_Radius );
          break;
      }
      
      imageFunction->SetGradientSigma( this->m_GradientSigma );      
      imageFunction->SetAdaptativeSampling( true );
      imageFunction->SetAutoComputeSectionNormal( true );
      imageFunction->SetNormalizeAcrossScale( true );
      imageFunction->SetUseImageSpacing( true );
      imageFunction->Initialize();
    }
  
protected:
  
  OffsetMedialnessImageFunctionInitializer() : 
    m_ScaleSelectionMethod( UseScaleAsRadius ),
    m_HessianSigma ( OFFSET_MEDIALNESS_INITIALIZER_FIXED_SIGMA ),
    m_Radius( OFFSET_MEDIALNESS_INITIALIZER_FIXED_RADIUS ),
    m_GradientSigma( 1.0 ),
    m_SymmetryCoefficient( 0.5 ),
    m_GradientImageFunctionType( ImageFunctionType::GradientNormalProjectionFunctionHyperIntense )
    {}

  virtual ~OffsetMedialnessImageFunctionInitializer() {}
  
private:
  
  OffsetMedialnessImageFunctionInitializer(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented
  
private:
  
  /** Sigma for Hessian matrix calculations. Default is 1.0. */
  double           m_HessianSigma;
      
  /** Radius for medialness calculation. Default is 1.0. */
  double           m_Radius;
  
  /** Sigma used for the gradient calculations. In general this sigma should be smaller than the 
    * sigma used for the Hessian. By default is 1.0. */
  double           m_GradientSigma;
  
  /** Set the symmetry coefficient for the offset medialness (between 0.0 and 1.0). */
  double           m_SymmetryCoefficient;
  
  /** Flag that indicates whether the scale provided at initialization will be used as value for sigma 
    * or for radius. If ignore scale is chosen, fixed values provided will be used for sigma and radius. */
  ScaleSelectionMethodType     m_ScaleSelectionMethod;
  
  /** The type of gradient use (i.e. magnitude or projected along normal hyperintense and hypointense). */
  GradientImageFunctionType    m_GradientImageFunctionType;
};

} // end namespace ivan

#endif
