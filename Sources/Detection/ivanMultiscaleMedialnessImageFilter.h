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
// File : ivanMultiscaleMedialnessImageFilter.h
// Author : Iv�n Mac�a (imacia@vicomtech.org)
// Description : Calculates the multiscale medialness response of the given image.


#ifndef __ivanMultiscaleMedialnessImageFilter_h
#define __ivanMultiscaleMedialnessImageFilter_h


#include "ivanMultiscaleAnalysisImageFilter.h"

#include "itkSymmetricEigenAnalysis.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkVector.h"


namespace ivan
{

/** \class MultiscaleMedialnessImageFilter
 *  \brief 
 *
 * 
 * 
 * \ingroup IntensityImageFilters
 */
template <class TInputImage, class TOutputImage, class TScalePixel = double, class TMaskImage = TInputImage>
class ITK_EXPORT MultiscaleMedialnessImageFilter : 
	public MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
{
public:

  /** Standard class typedefs. */
  typedef MultiscaleMedialnessImageFilter               Self;
  typedef MultiscaleAnalysisImageFilter<TInputImage,
  	TOutputImage,TScalePixel,TMaskImage>  							Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  /** Typedefs for image types. */
  typedef TInputImage		InputImageType;
  typedef TOutputImage  OutputImageType;
  typedef TMaskImage		MaskImageType;
 
  typedef TScalePixel		ScalePixelType;
  typedef typename OutputImageType::PixelType		      OutputImagePixelType;
  typedef typename Superclass::TensorScalePixelType		TensorScalePixelType;
  
  typedef typename Superclass::ScaleContainerType	    ScaleContainerType;

	/** Scale image types. */
	typedef typename Superclass::ScaleImageType					ScaleImageType;
	typedef typename Superclass::TensorScaleImageType		TensorScaleImageType;
  
  /** Image dimension = 3. */
  itkStaticConstMacro(ImageDimension, unsigned int,
                      ::itk::GetImageDimension<InputImageType>::ImageDimension);
  
  /** Image type for section normal. */
  typedef itk::Vector<TScalePixel,ImageDimension>     VectorPixelType;
  typedef itk::Image<VectorPixelType,ImageDimension>  VectorImageType;
  
  /** Iterators. */
  typedef itk::ImageRegionConstIterator<InputImageType>        InputConstIterator;
  typedef itk::ImageRegionIterator<OutputImageType>            OutputIterator;
  typedef itk::ImageRegionConstIterator<MaskImageType>         MaskImageIterator;
  typedef itk::ImageRegionConstIterator<ScaleImageType>        GradientImageIterator;
  typedef itk::ImageRegionIterator<ScaleImageType>             ScalesImageIterator;
  typedef	itk::ImageRegionConstIterator<TensorScaleImageType>	 TensorImageIterator;
  typedef itk::ImageRegionIterator<VectorImageType>						 VectorImageIterator;
  
  /** Interpolator used for calculations. */
  typedef	itk::LinearInterpolateImageFunction<ScaleImageType,double>		InterpolatorType;
  
  /** Types for eigenanalysis. */
  typedef	itk::SymmetricEigenAnalysis< TensorScalePixelType, 
		typename TensorScalePixelType::EigenValuesArrayType, 
		typename TensorScalePixelType::EigenVectorsMatrixType >		EigenAnalysisType;
		
	typedef typename Superclass::GradientMagnitudeFilterType  GradientMagnitudeFilterType;
  typedef typename Superclass::HessianFilterType  					HessianFilterType;
      
public:

  /** Method for creation through the object factory. */
  itkNewMacro(Self);  

  /** Run-time type information (and related methods). */
  itkTypeMacro( MultiscaleMedialnessImageFilter, MultiscaleAnalysisImageFilter );

  /** Get/set the output threshold value. */
  itkSetMacro( OutputThreshold, OutputImagePixelType );
  itkGetMacro( OutputThreshold, OutputImagePixelType );
  
  /** Get/set the symmetry coefficient. */
  itkSetMacro( SymmetryCoefficient, double );
  itkGetMacro( SymmetryCoefficient, double );
  
   /** Get/set the radius factor. */
  itkSetMacro( RadiusFactor, double );
  itkGetMacro( RadiusFactor, double );
  
  /** Flag for activating eigenvalue filtering. */
	void SetFilterByEigenValues( bool filterByEigenValues );
  itkBooleanMacro( FilterByEigenValues );
  
  /** Get the normals image. This is the normal of the section estimated as the third eigenvalue. */
	VectorImageType* GetOutputNormals()
		{ return dynamic_cast< VectorImageType* >( this->ProcessObject::GetOutput(2) );	}
  

protected:

  MultiscaleMedialnessImageFilter();
  ~MultiscaleMedialnessImageFilter() {};
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;
  
  /** Override since the filter produces the entire dataset. */  
  virtual void EnlargeOutputRequestedRegion( itk::DataObject *output );

	/** Prepare output images. */
	virtual void PrepareData();

  /** Called by GenerateData to generate data for the current scale. */
  virtual void GenerateDataAtScale( const ScalePixelType currentScale );
  
  /** Called by GenerateData to generate data for the current scale using the provided mask. 
    * Overriden to speed-up computations using the provided mask. 
    */
  virtual void GenerateDataUsingMaskAtScale( const ScalePixelType currentScale );
  
  
private:

  MultiscaleMedialnessImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

private:

	/** Output threshold for medialness. */
  OutputImagePixelType		m_OutputThreshold;
  
  /** Coefficient for taking into account the simmetry in the medialness. */
  double		m_SymmetryCoefficient;
  
  /** Radius factor. This is multiplied by the current scale to obtain the distance (radius) 
    * at which points are sampled in a circle (by default sqrt(3.0) which gives the maximum
    * value for medialness. */
  double    m_RadiusFactor;
  
  /** Flag for filtering pixels by eigenvalue. */
  bool      m_FilterByEigenValues;
  
  /** Eigenanalysis calculator. */
  EigenAnalysisType  m_EigenAnalysis;
  
  /** Interpolator used to estimate gradient at non-index positions. */
	typename InterpolatorType::Pointer 	m_GradientInterpolator;
  

};

  
} // end namespace ivan
  
#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanMultiscaleMedialnessImageFilter.txx"
#endif
  
#endif
