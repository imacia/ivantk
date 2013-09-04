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
// File: ivanVesselnessBasedSearchVesselTrackerFilter.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2011/01/21


#ifndef __ivanVesselnessBasedSearchVesselTrackerFilter_h
#define __ivanVesselnessBasedSearchVesselTrackerFilter_h


#include "ivanVesselTrackerFilter.h"
#include "ivanImageFunctionInitializerBase.h"

#include "itkImage.h"
#include "itkSingleValuedNonLinearOptimizer.h"


namespace ivan
{

/** \class VesselnessBasedSearchVesselTrackerFilter
 * \brief VesselTracker that takes a vesselness measure to guide the a search stage.
 *
 * VesselnessBasedSearchVesselTrackerFilter takes a vesselness measure to guide the search stage. This is
 * a correction stage used to reestimate parameters such as the section center, normal or scale.
 * This vesselness measure can be calculated on the fly from the source image or can be retrieved 
 * from an external image.
 *
 * The vesselness measure is specified as a template parameter which corresponds to an image function.
 * 
 *
 */

template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
class ITK_EXPORT VesselnessBasedSearchVesselTrackerFilter : public VesselTrackerFilter<TInputImage,TOutputVessel>
{
public:
	
  /** Standard class typedefs. */
  typedef VesselnessBasedSearchVesselTrackerFilter
    <TInputImage, TOutputVessel, TVesselnessFunction>  Self;
  typedef VesselTrackerFilter
    <TInputImage,TOutputVessel>                        Superclass;
  typedef itk::SmartPointer<Self>                      Pointer;
  typedef itk::SmartPointer<const Self>                ConstPointer;
  
  /** Some convenient typedefs. */
  typedef TInputImage                               ImageType;
  typedef typename Superclass::ImagePointer         ImagePointer;
  typedef typename Superclass::ImageConstPointer    ImageConstPointer;
  typedef typename Superclass::ImageRegionType      ImageRegionType; 
  typedef typename Superclass::ImagePixelType       ImagePixelType;
  typedef typename Superclass::ImagePointType       ImagePointType;
  typedef typename Superclass::ImageIndexType       ImageIndexType; 
  	
  /** Some convenient typedefs. */
  typedef TOutputVessel                             OutputVesselType;
  typedef typename OutputVesselType::Pointer        OutputVesselPointer;
  
  typedef typename Superclass::CenterlineType       CenterlineType;
  typedef typename Superclass::SectionType          SectionType;
    
  typedef itk::Image<typename TVesselnessFunction::OutputType, 
    ImageType::ImageDimension>                                  VesselnessImageType;

  typedef typename TVesselnessFunction                          VesselnessFunctionType;
  typedef typename VesselnessFunctionType::Pointer              VesselnessFunctionPointer;
  typedef typename VesselnessFunctionType::OutputType           VesselnessValueType;

  typedef ImageFunctionInitializerBase
    <VesselnessFunctionType,ImageType>                          VesselnessFunctionInitializerType; 
  typedef typename VesselnessFunctionInitializerType::Pointer   VesselnessFunctionInitializerPointer;
    
public:
  
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselnessBasedSearchVesselTrackerFilter, VesselTrackerFilter );
  
  virtual void SetInput( const InputImageType *image );
  virtual void SetInput( unsigned int, const InputImageType * image );
  
  /** Set/Get the VesselnessImageFunction to use. This can be a vesselness calculated from a source
    * image as an image function, or, if the full vesselness image has been previously calculated,
    * can be an interpolator of that image, such as nearest neighbour or linear interpolator. */
  virtual void SetVesselnessImageFunction( VesselnessFunctionType *func )
    {
      m_VesselnessFunction = func;
      this->Modified();      
    }
  itkGetObjectMacro( VesselnessFunction, VesselnessFunctionType );
  itkGetConstObjectMacro( VesselnessFunction, VesselnessFunctionType );
  
  itkSetObjectMacro( VesselnessFunctionInitializer, VesselnessFunctionInitializerType );
  itkGetObjectMacro( VesselnessFunctionInitializer, VesselnessFunctionInitializerType );
  itkGetConstObjectMacro( VesselnessFunctionInitializer, VesselnessFunctionInitializerType );
    
  VesselnessValueType GetVesselnessValueAtCurrentPoint();
  VesselnessValueType GetVesselnessValueAtPreviousPoint();
   
protected:
  
  VesselnessBasedSearchVesselTrackerFilter();
  ~VesselnessBasedSearchVesselTrackerFilter() {}
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  VesselnessBasedSearchVesselTrackerFilter(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
  
protected:

  VesselnessFunctionPointer                  m_VesselnessFunction;
  
  /** Provide a means to initialize the vesselness function for each scale. This is called at 
    * initialization for the metrics of all scales and during the search stage whenever the
    * scale changes. */
  VesselnessFunctionInitializerPointer       m_VesselnessFunctionInitializer;
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanVesselnessBasedSearchVesselTrackerFilter.hxx"
#endif

#endif
