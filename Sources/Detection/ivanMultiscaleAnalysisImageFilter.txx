// File : itkMultiscaleAnalysisImageFilter.txx
// Author : Iv�n Mac�a (imacia@vicomtech.org)
// Description : base class for multiscale analysis filters


#ifndef _ivanMultiscaleAnalysisImageFilter_txx
#define _ivanMultiscaleAnalysisImageFilter_txx


#include "itkMinimumMaximumImageCalculator.h"
#include "itkDataObject.h"
#include "itkNumericTraits.h"
#include "itkProgressReporter.h"


namespace ivan
{

template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::MultiscaleAnalysisImageFilter() :
	m_NormalizeAcrossScale(false),
	m_DoNotComputeZeroPixels(false)
{
	typename ScaleImageType::Pointer scalesOutput = ScaleImageType::New();
	this->ProcessObject::SetNumberOfRequiredOutputs(2);
	this->ProcessObject::SetNthOutput(1, scalesOutput.GetPointer());
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::SetInput( const InputImageType *image )
{
  Superclass::SetInput( image );
  
  if( m_GradientMagnitudeGaussian.IsNotNull() )
   	m_GradientMagnitudeGaussian->SetInput( this->GetInput() );
    	
  if( m_GradientGaussian.IsNotNull() )
  	m_GradientGaussian->SetInput( this->GetInput() );
 	
	if( m_HessianGaussian.IsNotNull() )
	 	m_HessianGaussian->SetInput( this->GetInput() );
   
  if( m_LaplacianGaussian.IsNotNull() )
   	m_LaplacianGaussian->SetInput( this->GetInput() );
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::SetInput( unsigned int index, const TInputImage * image)
{
  Superclass::SetInput( index, image );
  
  if( !index )
  {
	  if( m_GradientMagnitudeGaussian.IsNotNull() )
	   	m_GradientMagnitudeGaussian->SetInput( this->GetInput() );
	    	
	  if( m_GradientGaussian.IsNotNull() )
	  	m_GradientGaussian->SetInput( this->GetInput() );
	 	
		if( m_HessianGaussian.IsNotNull() )
		 	m_HessianGaussian->SetInput( this->GetInput() );
	   
	  if( m_LaplacianGaussian.IsNotNull() )
	   	m_LaplacianGaussian->SetInput( this->GetInput() );
	}
}



template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::SetMaskImage(TMaskImage* mask)
{
   this->SetNthInput(1, const_cast<MaskImageType*>( mask ));
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
const TMaskImage*
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::GetMaskImage() const 
{
  return (static_cast<const TMaskImage*>(this->ProcessObject::GetInput(1)));
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::SetNormalizeAcrossScale( bool normalizeInScaleSpace )
{
	if( m_GradientMagnitudeGaussian.IsNotNull() )
   	m_GradientMagnitudeGaussian->SetNormalizeAcrossScale( normalizeInScaleSpace );
 
  if( m_GradientGaussian.IsNotNull() )
   	m_GradientGaussian->SetNormalizeAcrossScale( normalizeInScaleSpace );
 	
	if( m_GradientGaussian.IsNotNull() )
  	m_GradientGaussian->SetNormalizeAcrossScale( normalizeInScaleSpace );
  	
  if( m_LaplacianGaussian.IsNotNull() )
   	m_LaplacianGaussian->SetNormalizeAcrossScale( normalizeInScaleSpace );

	this->Modified();
	
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
unsigned int 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::GetNumberOfScales() const
{
	return m_Scales.size();
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
TScalePixel 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::GetMinimumScale() const
{
	ScalePixelType minimum = itk::NumericTraits<ScalePixelType>::max();
	
	typename ScaleContainerType::const_iterator it;
	it = m_Scales.begin();
	
	while( it != m_Scales.end() )
	{
		if( (*it) < minimum )
		  minimum = (*it);
		++it;	
	}
	
	return minimum;
		
}

  
template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
TScalePixel 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::GetMaximumScale() const
{
	ScalePixelType maximum = itk::NumericTraits<ScalePixelType>::min();
	
	typename ScaleContainerType::const_iterator it;
	it = m_Scales.begin();
	
	while( it != m_Scales.end() )
	{
		if( (*it) > maximum )
		  maximum = (*it);
		++it;	
	}
	
	return maximum;	
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::SetScales( const unsigned int numberOfScales, const ScalePixelType minimumScale, 
 	const ScalePixelType maximumScale )
{
  if( !numberOfScales ) // warning? exception?
  	return;
    	
  if( maximumScale < minimumScale ) // warning? exception?
  	return;
	
	double scaleIncrement = ( maximumScale - minimumScale ) / ( numberOfScales - 1 );
	double currentScale = minimumScale;
	
	for( unsigned int i=0; i<numberOfScales; ++i )
	{
		m_Scales.push_back( currentScale );
		currentScale += scaleIncrement;
	}
	
	this->Modified();
} 


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::AddScale( const ScalePixelType newScale )
{
	m_Scales.push_back( newScale );
	this->Modified();
}
    
      
template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::ClearScales()
{
	m_Scales.clear();
}   
 

template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::AddScales( const ScaleContainerType& scales )
{
	typename ScaleContainerType::const_iterator it = scales.begin();
	
	while( it != scales.end() )
	{
		m_Scales.push_back( *it );
		++it;		
	}

	this->Modified();
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );

  os << "ScalesModified" << m_ScalesModified << std::endl;
	os << "NormalizeAcrossScale: " << m_NormalizeAcrossScale << std::endl;
	os << "DoNotComputeZeroPixels: " << m_DoNotComputeZeroPixels << std::endl;
	
	for( int i=0; i<m_Scales.size(); ++ i )
	{
  	os << indent << "Scale " << i << ": "
  	   << static_cast<typename NumericTraits<ScalePixelType>::PrintType>(m_Scales[i])
       << std::endl;
 	}
  
  
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::GenerateInputRequestedRegion() throw( itk::InvalidRequestedRegionError )
{
  // call the superclass' implementation of this method. this should
  // copy the output requested region to the input requested region
  Superclass::GenerateInputRequestedRegion();

  // This filter needs all of the input

  typename InputImageType::Pointer image = const_cast<InputImageType *>( this->GetInput() );
  image->SetRequestedRegion( this->GetInput()->GetLargestPossibleRegion() );

	typename MaskImageType::Pointer maskImage = dynamic_cast<TMaskImage*>(this->ProcessObject::GetInput(1));
	if( maskImage )
		maskImage->SetRequestedRegion( this->GetInput()->GetLargestPossibleRegion() );

}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::EnlargeOutputRequestedRegion( itk::DataObject *output )
{
  TOutputImage *out = dynamic_cast<TOutputImage*>(output);

  if (out)
    out->SetRequestedRegion( out->GetLargestPossibleRegion() );
      
  ScaleImageType *scaleOutput = this->GetOutputScales();
  
  if( scaleOutput )
  	scaleOutput->SetRequestedRegion( out->GetLargestPossibleRegion() );  
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::PrepareData()
{
	// Allocate memory for both outputs and fill the buffers with zeros
  // This is called instead of allocate outputs which does not work well here

	typename OutputImageType::Pointer outputPtr = this->GetOutput(0);
  outputPtr->SetBufferedRegion(outputPtr->GetRequestedRegion());
  outputPtr->Allocate();
	outputPtr->FillBuffer( itk::NumericTraits<typename OutputImageType::PixelType>::Zero );
	
	typename ScaleImageType::Pointer scalesPtr = dynamic_cast< ScaleImageType * >
		( this->ProcessObject::GetOutput(1) );
	scalesPtr->SetBufferedRegion(scalesPtr->GetRequestedRegion());
  scalesPtr->Allocate();
	scalesPtr->FillBuffer( itk::NumericTraits<ScalePixelType>::Zero );

}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::GenerateData()
{
  itkDebugMacro(<<"Actually executing");

	if( !m_Scales.size() )
	{
		itkWarningMacro(<<"No scales set. Aborting execution");
		return;
	}


	// Do not call AllocateOutputs() here
	// Allocate memory for the outputs  
 	this->PrepareData();
	
	typename MaskImageType::ConstPointer mask = this->GetMaskImage();
	
	if( !mask && m_DoNotComputeZeroPixels )
	{
		this->GenerateZeroPixelsMask();
		mask = this->GetMaskImage();
	}
	
	   	
 	typename ScaleContainerType::const_iterator it = m_Scales.begin();
	
	while( it != m_Scales.end() )
	{
		this->UpdateScaleFiltersAtScale( *it );
		if (!mask)
			this->GenerateDataAtScale( *it );
		else
			this->GenerateDataUsingMaskAtScale( *it );
		++it;		
	}
   
}


template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::GenerateZeroPixelsMask()
{
	typename InputImageType::ConstPointer inputPtr  = this->GetInput();
	
	m_ZeroPixelsMaskImage = MaskImageType::New();
	m_ZeroPixelsMaskImage->SetRegions( inputPtr->GetRequestedRegion() );
	m_ZeroPixelsMaskImage->SetOrigin( inputPtr->GetOrigin() );
	m_ZeroPixelsMaskImage->SetSpacing( inputPtr->GetSpacing() );
	m_ZeroPixelsMaskImage->SetDirection( inputPtr->GetDirection() );
	m_ZeroPixelsMaskImage->Allocate();
	
	// Initialize the mask with ones
	m_ZeroPixelsMaskImage->FillBuffer( itk::NumericTraits<typename MaskImageType::PixelType>::One );
	
	typedef ImageRegionConstIterator<InputImageType> 		ConstIteratorType;
	typedef ImageRegionIterator<MaskImageType> 					IteratorType;
	
	ConstIteratorType it( inputPtr, inputPtr->GetRequestedRegion() );
	IteratorType out( m_ZeroPixelsMaskImage, inputPtr->GetRequestedRegion() );
	
	// If a pixel has zero value set it to zero in the mask too
	for( it.GoToBegin(), out.GoToBegin(); !it.IsAtEnd(); ++it, ++out )
	{
		if( !it.Get() )
			out.Set( itk::NumericTraits<typename MaskImageType::PixelType>::Zero );
	}
	
	this->SetMaskImage( m_ZeroPixelsMaskImage );
	
}



template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::GenerateDataUsingMaskAtScale( const ScalePixelType currentScale )
{
	// If no reimplemented call the standard version
	this->GenerateDataAtScale( currentScale );	
}



template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleAnalysisImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::UpdateScaleFiltersAtScale( const ScalePixelType currentScale )
{
  itkDebugMacro(<<"Updating scale filters at scale" << currentScale);

  if( m_GradientMagnitudeGaussian.IsNotNull() )
  {
  	m_GradientMagnitudeGaussian->SetSigma( currentScale );
  	m_GradientMagnitudeGaussian->Update();
  }
  	
  if( m_GradientGaussian.IsNotNull() )
  {
  	m_GradientGaussian->SetSigma( currentScale );
  	m_GradientGaussian->Update();
  }
	
	if( m_HessianGaussian.IsNotNull() )
	{
  	m_HessianGaussian->SetSigma( currentScale );
  	m_HessianGaussian->Update();
  }
  
  if( m_LaplacianGaussian.IsNotNull() )
  {
  	m_LaplacianGaussian->SetSigma( currentScale );
  	m_LaplacianGaussian->Update();
  }  
}

} // end namespace ivan

#endif
