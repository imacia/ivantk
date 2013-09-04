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
// File : ivanMultiscaleAnalysisImageFilter.h
// Author : Iv�n Mac�a (imacia@vicomtech.org)
// Description : Calculates the multiscale medialness response of the given dataset.


#ifndef __ivanMultiscaleAnalysisImageFilter_h
#define __ivanMultiscaleAnalysisImageFilter_h


#include "itkImageToImageFilter.h"
#include "itkRecursiveGaussianImageFilter.h"
#include "itkGradientMagnitudeRecursiveGaussianImageFilter.h"
#include "itkGradientRecursiveGaussianImageFilter.h"
#include "itkHessianRecursiveGaussianImageFilter.h"
#include "itkLaplacianRecursiveGaussianImageFilter.h"
#include <vector>


namespace ivan
{

/** \class MultiscaleAnalysisImageFilter
 *  \brief Base class for filters that perform analysis in scale-space at multiple scales
 *
 * 
 * 
 * \ingroup IntensityImageFilters
 */
template <class TInputImage, class TOutputImage, class TScalePixel = double, 
  class TMaskImage = TInputImage>
class ITK_EXPORT MultiscaleAnalysisImageFilter : public itk::ImageToImageFilter<TInputImage,TOutputImage>
{
public:

  /** Standard class typedefs. */
  typedef MultiscaleAnalysisImageFilter                       Self;
  typedef itk::ImageToImageFilter<TInputImage,TOutputImage>  Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef TInputImage		InputImageType;
  typedef TOutputImage  OutputImageType;

	/** Image dimension = 3. */
  itkStaticConstMacro(ImageDimension, unsigned int,
                      ::itk::GetImageDimension<InputImageType>::ImageDimension);
                      
  /** Mask image types. */
  typedef TMaskImage			MaskImageType;
 
 	/** Scale pixel types. */
  typedef TScalePixel		ScalePixelType;
  typedef std::vector<ScalePixelType>	 ScaleContainerType;
  typedef itk::Vector< ScalePixelType, ImageDimension >    VectorScalePixelType;
  typedef itk::SymmetricSecondRankTensor< ScalePixelType, 
		ImageDimension >																	     TensorScalePixelType;

	/** Scale image types. */
	typedef itk::Image<ScalePixelType,ImageDimension>				ScaleImageType;
	typedef itk::Image<VectorScalePixelType,ImageDimension>	VectorScaleImageType;
	typedef itk::Image<TensorScalePixelType,ImageDimension>	TensorScaleImageType;
  
  /** Scale filters. */
  typedef itk::GradientMagnitudeRecursiveGaussianImageFilter
    <TInputImage,ScaleImageType>	        GradientMagnitudeFilterType;
  typedef itk::LaplacianRecursiveGaussianImageFilter
    <TInputImage,ScaleImageType>        	LaplacianFilterType;
  typedef itk::GradientRecursiveGaussianImageFilter
    <TInputImage,VectorScaleImageType>	  GradientFilterType;
  typedef itk::HessianRecursiveGaussianImageFilter
    <TInputImage,TensorScaleImageType>	  HessianFilterType;  
      
public:

  /** Method for creation through the object factory. */
  itkNewMacro(Self);  

  /** Run-time type information (and related methods). */
  itkTypeMacro( MultiscaleAnalysisImageFilter, itk::ImageToImageFilter );
  
  /** Set the image input of this process object. Overriden from ImageToImageFilter.
    * Sets the input also to the existing scale filters. */
  virtual void SetInput( const InputImageType *image);
  virtual void SetInput( unsigned int index, const TInputImage * image);
  
  /** Set the mask image. */
  void SetMaskImage(TMaskImage* mask);
  
  /** Get the mask image. */
  const TMaskImage* GetMaskImage() const; 
  
  /** Get the scales image. This is the scale at which each pixel value of the output was calculated. */
	ScaleImageType* GetOutputScales()
		{
			return dynamic_cast< ScaleImageType * >( this->ProcessObject::GetOutput(1) );	
		}

  /** Define which normalization factor will be used for the Gaussian */
  void SetNormalizeAcrossScale( bool normalizeInScaleSpace );
  itkGetMacro( NormalizeAcrossScale, bool );
  
  /** Set if we want to make computations in pixels with zero value. If true, a mask is created
  	* with the pixels that have zero value and is assigned as the mask image. */
  void SetDoNotComputeZeroPixels( bool computeZeroPixels )
    { 
      m_DoNotComputeZeroPixels = computeZeroPixels;
      this->Modified();
    }
  itkGetMacro( DoNotComputeZeroPixels, bool );
  
  /** Get the current number of scales. */
  unsigned int GetNumberOfScales() const;
  
  /** Get the minimum scale in the scales vector. */
  ScalePixelType GetMinimumScale() const;
  
  /** Get the maximum scale in the scales vector. */
  ScalePixelType GetMaximumScale() const;
  
  /** Set scales specifying minimum, maximum and number of scales. */
  void SetScales( const unsigned int numberOfScales, const ScalePixelType minimumScale, 
  	const ScalePixelType maximumScale );
  	
  /** Add a new scale at the end of the container. */
  void AddScale( const ScalePixelType newScale );
  
  /** Copy the scales in the given container (does not delete stored scales). */
  void AddScales( const ScaleContainerType& scales );

  /** Clear all scales. */
  void ClearScales();   
  
  /** This filter needs all of the input to produce an output.
   * Therefore, MultiscaleAnalysisImageFilter needs to provide
   * an implementation for GenerateInputRequestedRegion in order to inform
   * the pipeline execution model.
   * \sa ImageToImageFilter::GenerateInputRequestedRegion() */
  virtual void GenerateInputRequestedRegion() throw( itk::InvalidRequestedRegionError );
  
protected:

  MultiscaleAnalysisImageFilter();
  ~MultiscaleAnalysisImageFilter() {};
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;
  
  /** Override since the filter produces the entire dataset and has one 
    * more output than MultiscaleAnalysisImageFilter. */  
  virtual void EnlargeOutputRequestedRegion( itk::DataObject *output );

	/** Prepare output images. Overriden from MultiscaleAnalysisImageFilter. */
	virtual void PrepareData();

	/** Reimplemented to generate data once at each scale. */
  virtual void GenerateData();
  
  /** Generate a mask with the pixels which have zero value in the image. Then assign this
    * as the mask image. This is called in GenerateData() when no mask is assigned and
    * m_DoNotComputeZeroPixels is true. */
  virtual void GenerateZeroPixelsMask();
  
  /** Called by GenerateData to generate data for the current scale. Must be reimplemented by subclasses. */
  virtual void GenerateDataAtScale( const ScalePixelType currentScale ) {}
  
  /** Called by GenerateData to generate data for the current scale using the provided mask. 
    * By default this calls GenerateDataAtScale() but can be overriden to speed-up computations
    * using the provided mask. 
    */
  virtual void GenerateDataUsingMaskAtScale( const ScalePixelType currentScale );
  
  /** Updates allocated scale filters at the current scale. */
  virtual void UpdateScaleFiltersAtScale( const ScalePixelType currentScale );

private:

  MultiscaleAnalysisImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

	/** Container for storing the different scales. */
	ScaleContainerType	m_Scales;
	
	/** Internal flag for scale modification. Avoid recalculation of scale filters if not necessary. */
	bool     m_ScalesModified;
	
	/** Gradient magnitude gaussian. */
	typename GradientMagnitudeFilterType::Pointer   m_GradientMagnitudeGaussian;
	
	/** Gradient gaussian. */
	typename GradientFilterType::Pointer   m_GradientGaussian;
	
	/** Hessian gaussian. */
	typename HessianFilterType::Pointer    m_HessianGaussian;
		
	/** Laplacian gaussian. */
	typename LaplacianFilterType::Pointer	 m_LaplacianGaussian;
	  
	/** Normalize the image across scale space */
  bool 		 m_NormalizeAcrossScale; 
  
  /** Flag for computing pixels with zero value. */
  bool		 m_DoNotComputeZeroPixels;
  
  /** Mask with zero pixels. */
  typename MaskImageType::Pointer		m_ZeroPixelsMaskImage;
    
};

  
} // end namespace ivan
  
#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanMultiscaleAnalysisImageFilter.txx"
#endif
  
#endif
