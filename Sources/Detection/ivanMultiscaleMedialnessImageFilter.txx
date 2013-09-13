// File : itkMultiscaleMedialnessImageFilter.txx
// Author : Iván Macía (imacia@vicomtech.org)
// Description : Calculates the multiscale medialness response of the given dataset.


#ifndef _itkMultiscaleMedialnessImageFilter_txx
#define _itkMultiscaleMedialnessImageFilter_txx


#include "itkNumericTraits.h"
#include "itkProgressReporter.h"
#include "itkArray.h"
#include "itkNeighborhoodAlgorithm.h"


namespace ivan
{

/**
 *
 */
template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
MultiscaleMedialnessImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::MultiscaleMedialnessImageFilter() :
  m_FilterByEigenValues(false),
  m_RadiusFactor(1.7320508) // = sqrt(3.0)
{
  this->m_DoNotComputeZeroPixels = true; // this filter is computationally quite expensive
  
  m_OutputThreshold = NumericTraits<OutputImagePixelType>::Zero;
  m_SymmetryCoefficient = NumericTraits<double>::One;
  
  
  // Allocate here the scale filter we need to use
  
  this->m_HessianGaussian = HessianFilterType::New();
  this->m_HessianGaussian->SetNormalizeAcrossScale( true );
  
  this->m_GradientMagnitudeGaussian = GradientMagnitudeFilterType::New();
  this->m_GradientMagnitudeGaussian->SetNormalizeAcrossScale( false ); // !!! PURPOSELY SET TO FALSE
  // The normalization is done in this filter manually (on a pixel basis). There is a problem with the 
  // normalization factor that does not correspond to what I would expect
  // See a detailed description of the problem here
  // http://public.kitware.com/pipermail/insight-users/2006-May/017900.html
    
  m_EigenAnalysis.SetOrderEigenValues(true);
  m_EigenAnalysis.SetDimension(ImageDimension); 
  
  m_GradientInterpolator = InterpolatorType::New();
  
  // This filter has 3 outputs instead of 2
  this->ProcessObject::SetNumberOfRequiredOutputs(3);
  
  // Create the vector image that will be used as output for the estimated normals 
  typename  VectorImageType::Pointer normalsImage = VectorImageType::New();
  this->ProcessObject::SetNthOutput(2, normalsImage.GetPointer());  
    
}


/**
 *
 */
template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleMedialnessImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::SetFilterByEigenValues( bool filterByEigenValues )
{
  m_FilterByEigenValues = filterByEigenValues;
  this->Modified();
}


/**
 *
 */
template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleMedialnessImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "OutputThreshold: "
     << static_cast<typename NumericTraits<OutputImagePixelType>::PrintType>(m_OutputThreshold)
     << std::endl;
  os << indent << "SymmetryCoefficient: "
     << static_cast<typename NumericTraits<double>::PrintType>(m_SymmetryCoefficient)
     << std::endl;
  os << indent << "RadiusFactor: "
     << static_cast<typename NumericTraits<double>::PrintType>(m_RadiusFactor)
     << std::endl; 
  os << indent << "FilterByEigenValues: "
     << static_cast<typename NumericTraits<bool>::PrintType>(m_FilterByEigenValues)
     << std::endl;
 
}



/**
 *
 */
template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleMedialnessImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::EnlargeOutputRequestedRegion( itk::DataObject *output )
{
  Superclass::EnlargeOutputRequestedRegion(output);
  
  TOutputImage *out = dynamic_cast<TOutputImage*>(output);
  
  VectorImageType *normalsOutput = this->GetOutputNormals();
  
  if( normalsOutput )
    normalsOutput->SetRequestedRegion( out->GetLargestPossibleRegion() ); 
}



/**
 *
 */
template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleMedialnessImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::PrepareData()
{
  Superclass::PrepareData();
  
  typename VectorImageType::Pointer normalsPtr = dynamic_cast< VectorImageType * >
    ( this->ProcessObject::GetOutput(2) );
  normalsPtr->SetBufferedRegion(normalsPtr->GetRequestedRegion());
  normalsPtr->Allocate();
  normalsPtr->FillBuffer( itk::NumericTraits<ScalePixelType>::Zero ); 
}




/**
 *
 */
template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleMedialnessImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::GenerateDataAtScale( const ScalePixelType currentScale )
{
  itkDebugMacro(<< "MultiscaleMedialnessImageFilter generating data at scale " << currentScale);

  // Get the input and output pointers
  typename InputImageType::ConstPointer inputPtr  = this->GetInput();
  typename OutputImageType::Pointer outputPtr = this->GetOutput(0);
  typename ScaleImageType::Pointer scalesPtr = dynamic_cast< ScaleImageType * >
    ( this->ProcessObject::GetOutput(1) );
  typename VectorImageType::Pointer normalsPtr = dynamic_cast< VectorImageType * >
    ( this->ProcessObject::GetOutput(2) );
  
  m_GradientInterpolator->SetInputImage( this->m_GradientMagnitudeGaussian->GetOutput() );    
  
  // Current eigenvalues, eigenvectors and related variables
  typename TensorScalePixelType::EigenVectorsMatrixType   eigenVectors; // current eigenvalues
  typename TensorScalePixelType::EigenValuesArrayType     eigenValues; // current eigenvectors
  itk::Array<double>    firstEigenVector;
  itk::Array<double>    secondEigenVector;
  firstEigenVector.SetSize(ImageDimension);
  secondEigenVector.SetSize(ImageDimension);
  
  // The estimated normal will be the third eigenvector but we need it in the form of an itk::Vector
  VectorPixelType thirdEigenVector;
    
  // Arrays for medialness calculations
  itk::Array<double>  sinArray; // store sin values
  itk::Array<double>  cosArray; // store cos values
  itk::Array<double>  radialMedialness; // this stores all values of gradient in the circle with given radius
  itk::Array<double>  radialMedialnessWeights;  // this stores all weighting factors in the circle with given radius
      
  // Variables involved in medialness calculation
  double radius; // current radius for medialness calculation
  unsigned int samples; // number of samples in a circle
  unsigned int calculatedSamples; // number of calculated samples in a circle
  double angleInc; // angle increment
  double angle; // current angle
  double medialness; // calculated medialness at current point and scale
  double weightExponent; // exponent of weighting factors for medialness calculations
  double averageRadialMedialness; // measure of average medialness without weighting
  double centralBoundariness; // boundariness at the current point
  typename InterpolatorType::PointType currentCirclePoint;
  typename InterpolatorType::IndexType currentCircleIndex;
  
  // Some other necessary variables
  long pixelCount = 0;
  typename InputImageType::IndexType currentIndex;
  typename InputImageType::PointType currentPoint;
  typename InputImageType::SpacingType spacing = inputPtr->GetSpacing();
      
  // Current radius 
  radius = m_RadiusFactor * currentScale;
  
  // Set the number of samples in  a circle depending on the current scale
  samples = static_cast<unsigned int>( floor( 2.0 * vnl_math::pi * currentScale + 1.0 + 0.5 ) ); // +0.5 and floor used for rounding
      
  // Set the angle increment depending on the number of samples
  angleInc = vnl_math::pi * 2.0 / static_cast<double>( samples ); 
  
  // Set the size of the arrays for storing radial boundariness values depending on the number of samples
  sinArray.SetSize( samples );
  cosArray.SetSize( samples );
  radialMedialness.SetSize( samples );
  radialMedialnessWeights.SetSize( samples );
  
  // Initialize the arrays
  radialMedialness.Fill( 0.0f );
  radialMedialnessWeights.Fill( 0.0f );
  
  // Precalculate sin and cos values as these calculations are costly
  
  angle = 0.0;
  
  for ( unsigned int i=0; i<samples; ++i )
  {
    sinArray[i] = sin( angle );
    cosArray[i] = cos( angle );
    angle += angleInc;
  }     

  // Get the minimum spacing
  
  double minSpacing = itk::NumericTraits<double>::max();
  
  for( unsigned int i=0; i<ImageDimension; ++i )
  {
    if ( spacing[i] < minSpacing )
      minSpacing = spacing[i];
  }
  
  // Create a list of boundary faces. This is to avoid bounds checking in the inner region
    
  typedef typename NeighborhoodIterator<TInputImage>::RadiusType  RadiusType;
  RadiusType faceListRadius;
  typename RadiusType::SizeValueType faceListRadiusSize = 
    static_cast<typename RadiusType::SizeValueType>( ceil( radius / minSpacing ) ) ;

  faceListRadius.Fill( static_cast<typename RadiusType::SizeValueType>( faceListRadiusSize ) );
  
  typedef itk::NeighborhoodAlgorithm::
    ImageBoundaryFacesCalculator<TInputImage>   FaceCalculatorType;
  
  FaceCalculatorType faceCalculator;
  typename FaceCalculatorType::FaceListType faceList;
  typename FaceCalculatorType::FaceListType::iterator fit;
  bool checkBounds;
      
  faceList = faceCalculator( inputPtr, outputPtr->GetRequestedRegion(), faceListRadius );
    
  // Create the iterators
  InputConstIterator it;
  OutputIterator out;
  ScalesImageIterator scalesIt;
  GradientImageIterator gradientIt;
  TensorImageIterator hessianIt;
  VectorImageIterator normalsIt;
  
  // Iterate through the regions of the face list first
  
  for( fit = faceList.begin(); fit != faceList.end(); ++fit )
  {
    it = InputConstIterator( inputPtr, *fit );
    out =  OutputIterator( outputPtr, *fit );
    scalesIt = ScalesImageIterator( scalesPtr, *fit );
    gradientIt = GradientImageIterator( this->m_GradientMagnitudeGaussian->GetOutput(), *fit );
    hessianIt = TensorImageIterator( this->m_HessianGaussian->GetOutput(), *fit );
    normalsIt = VectorImageIterator( normalsPtr, *fit );
  
    if( fit == faceList.begin() ) // first face is the inner region so don't check bounds
      checkBounds = false;
    else
      checkBounds = true;
  
  
    // Now iterate in each region of the face list
    
    for ( it.GoToBegin(), out.GoToBegin(), gradientIt.GoToBegin(), hessianIt.GoToBegin(), scalesIt.GoToBegin(), normalsIt.GoToBegin();  
      !out.IsAtEnd(); ++it, ++out, ++gradientIt, ++hessianIt, ++scalesIt, ++normalsIt )
    {
        currentIndex = out.GetIndex();

        // Check that the hessian matrix is not a zero matrix. This avoids problems with NaN eigenvalues and eigenvectors
        if( hessianIt.Get().GetTrace() < 1e-3 )
          continue;
    
        // Calculate eigenvalues and eigenvectors at the current location
        m_EigenAnalysis.ComputeEigenValuesAndVectors( hessianIt.Get(), eigenValues, eigenVectors );
    
        // Recover eigenvectors from the matrix (first two rows)
        for( unsigned int i=0; i<ImageDimension; ++i )
        {
          firstEigenVector[i]  = eigenVectors( 0, i );
          secondEigenVector[i] = eigenVectors( 1, i );
          thirdEigenVector[i]  = eigenVectors( 2, i );
        }

        // Do not process those pixels whose first two eigenvalues are not negative
        
        if( m_FilterByEigenValues )
        {
          if( eigenValues[0] >= 0.0f || eigenValues[1] >= 0.0f )
            continue;
        }
        
      
        //////////////////////////
        // Medialness calculation
  
        // Get the physical coordinates of the current index
        inputPtr->TransformIndexToPhysicalPoint( currentIndex, currentPoint );
        
        calculatedSamples = 0;
  
        // Calculate points in a circle defined by the two eigenvectors with r = radius
        for ( unsigned int i=0; i<samples; ++i )
        {
          // Evaluate and store the value of the gradient in a circle 
          
          for( unsigned int k=0; k<ImageDimension; ++k )
            currentCirclePoint[k] = currentPoint[k] + radius * ( cosArray[i] * firstEigenVector[k] + sinArray[i] * secondEigenVector[k] );
            
          m_GradientInterpolator->ConvertPointToNearestIndex( currentCirclePoint, currentCircleIndex );
          
          if( checkBounds )
          {
            if( m_GradientInterpolator->IsInsideBuffer( currentCirclePoint ) )
            {
              radialMedialness[i] = m_GradientInterpolator->Evaluate( currentCirclePoint );
              ++calculatedSamples;
            }
            else
              radialMedialness[i] = 0.0;
          }
          
          else // checkBounds
          {
            radialMedialness[i] = m_GradientInterpolator->Evaluate( currentCirclePoint );
            ++calculatedSamples;
          }
        
        }
  
        medialness = 0.0;
  
        if( calculatedSamples > 0 )
        {
          if( m_SymmetryCoefficient == 1.0 ) // speed-up calculations for the simplest (default) case
          {
            for ( unsigned int i=0; i<samples; ++i )
              medialness += radialMedialness[i];  
          }
          
          else
          {
            averageRadialMedialness = radialMedialness.one_norm() / static_cast<double>( calculatedSamples );
              
            for ( unsigned int i=0; i<samples; ++i )
            {
              if( !radialMedialness[i] )
                continue;
              
              // Calculate b coefficients
              weightExponent = ( 1.0 - radialMedialness[i] / averageRadialMedialness ) / m_SymmetryCoefficient;
              radialMedialnessWeights[i] = exp( -0.5 * weightExponent * weightExponent );
              medialness += radialMedialnessWeights[i] * radialMedialness[i];
              //medialness += radial_medialness[i];
            }
          
          }
  
          medialness /= static_cast<float>( calculatedSamples );
        
        }
  
                  
        // Now calculate final medialness using an adaptative threshold that depends on the central boundariness
        // (value of gradient at the central point). Vessel centers are supposed to have a low value of boundariness
        // and thus this is penalized. A threshold is introduced here for noise etc.
        
        centralBoundariness = gradientIt.Get();
    
        // Normalization factor. Here we have disabled AccrosScaleNormalization in the gradient
        // calculation but now we multiply by sigma to get sigma-normalized derivatives (see Krissian)
        medialness *= currentScale;
        centralBoundariness *= currentScale;
        
        // !!! FUNCIONA MEJOR ASI, xq si no muchos valores dan negativo y los ponemos a cero, sin embargo pueden ser
        // significativos. También es una manera de que la intensidad de la respuesta sea mayor
        if( medialness - centralBoundariness < m_OutputThreshold ) 
          medialness = 0.0f;
    
        if( medialness > out.Get() ) 
        {
          pixelCount++;
          out.Set( medialness ); // store medialness value
          scalesIt.Set( currentScale );
          normalsIt.Set( thirdEigenVector );
        }
    
    }
        
  }
  
  
}



/** Here the mask is used in two ways : 
 *  - Pixels outside the mask are directly not processed.
 *  - At the time of calculating the radial medialness of a pixel, the contributions of pixels 
 *    outside the mask are not considered
 */
template <class TInputImage, class TOutputImage, class TScalePixel, class TMaskImage>
void 
MultiscaleMedialnessImageFilter<TInputImage,TOutputImage,TScalePixel,TMaskImage>
::GenerateDataUsingMaskAtScale( const ScalePixelType currentScale )
{
  itkDebugMacro(<< "MultiscaleMedialnessImageFilter generating data at scale " << currentScale);

  // Get the input and output pointers
  typename InputImageType::ConstPointer inputPtr  = this->GetInput();
  typename OutputImageType::Pointer outputPtr = this->GetOutput(0);
  typename MaskImageType::ConstPointer maskPtr = this->GetMaskImage();
  typename ScaleImageType::Pointer scalesPtr = dynamic_cast< ScaleImageType * >
    ( this->ProcessObject::GetOutput(1) );
  typename VectorImageType::Pointer normalsPtr = dynamic_cast< VectorImageType * >
    ( this->ProcessObject::GetOutput(2) );

  m_GradientInterpolator->SetInputImage( this->m_GradientMagnitudeGaussian->GetOutput() );    
  
  // Current eigenvalues, eigenvectors and related variables
  typename TensorScalePixelType::EigenVectorsMatrixType   eigenVectors; // current eigenvalues
  typename TensorScalePixelType::EigenValuesArrayType     eigenValues; // current eigenvectors
  itk::Array<double>    firstEigenVector;
  itk::Array<double>    secondEigenVector;
  firstEigenVector.SetSize(ImageDimension);
  secondEigenVector.SetSize(ImageDimension);
  
  // The estimated normal will be the third eigenvector but we need it in the form of an itk::Vector
  VectorPixelType thirdEigenVector;
    
  // Arrays for medialness calculations
  itk::Array<double>  sinArray; // store sin values
  itk::Array<double>  cosArray; // store cos values
  itk::Array<double>  radialMedialness; // this stores all values of gradient in the circle with given radius
  itk::Array<double>  radialMedialnessWeights;  // this stores all weighting factors in the circle with given radius
    
  // Variables involved in medialness calculation
  double radius; // current radius for medialness calculation
  unsigned int samples; // number of samples in a circle
  unsigned int calculatedSamples;
  double angleInc; // angle increment
  double angle; // current angle and its sin and cos
  double medialness; // calculated medialness at current point and scale
  double weightExponent; // exponent of weighting factors for medialness calculations
  double averageRadialMedialness; // measure of average medialness without weighting
  double centralBoundariness; // boundariness at the current point
  typename InterpolatorType::PointType currentCirclePoint;
  typename InterpolatorType::IndexType currentCircleIndex;
  
  // Some other necessary variables
  long pixelCount = 0;
  typename InputImageType::IndexType currentIndex;
  typename InputImageType::PointType currentPoint;
  typename InputImageType::SpacingType spacing = inputPtr->GetSpacing();
      
  // Current radius 
  radius = m_RadiusFactor * currentScale;
    
  // Set the number of samples in  a circle depending on the current scale
  samples = static_cast<unsigned int>( floor( 2.0 * vnl_math::pi * currentScale + 1.0 + 0.5 ) ); // +0.5 and floor used for rounding
    
  // Set the angle increment depending on the number of samples
  angleInc = vnl_math::pi * 2.0 / static_cast<double>( samples ); 
  
  // Set the size of the arrays for storing radial boundariness values depending on the number of samples
  sinArray.SetSize( samples );
  cosArray.SetSize( samples );
  radialMedialness.SetSize( samples );
  radialMedialnessWeights.SetSize( samples );
  
  // Initialize the arrays
  radialMedialness.Fill( 0.0f );
  radialMedialnessWeights.Fill( 0.0f );
  
  // Precalculate sin and cos values as these calculations are very costly
  
  angle = 0.0;
  
  for ( unsigned int i=0; i<samples; ++i )
  {
    sinArray[i] = sin( angle );
    cosArray[i] = cos( angle );
    angle += angleInc;
  }     
  
  // Get the minimum spacing
  
  double minSpacing = itk::NumericTraits<double>::max();
  
  for( unsigned int i=0; i<ImageDimension; ++i )
  {
    if ( spacing[i] < minSpacing )
      minSpacing = spacing[i];
  }
  
  // Create a list of boundary faces. This is to avoid bounds checking in the inner region
    
  typedef typename NeighborhoodIterator<TInputImage>::RadiusType  RadiusType;
  RadiusType faceListRadius;
  typename RadiusType::SizeValueType faceListRadiusSize = 
    static_cast<typename RadiusType::SizeValueType>( ceil( radius / minSpacing ) ) ;

  faceListRadius.Fill( static_cast<typename RadiusType::SizeValueType>( faceListRadiusSize ) );
  
  typedef itk::NeighborhoodAlgorithm::
    ImageBoundaryFacesCalculator<TInputImage>   FaceCalculatorType;
  
  FaceCalculatorType faceCalculator;
  typename FaceCalculatorType::FaceListType faceList;
  typename FaceCalculatorType::FaceListType::iterator fit;
  bool checkBounds;
      
  faceList = faceCalculator( inputPtr, outputPtr->GetRequestedRegion(), faceListRadius );
  
  // Create the iterators
  InputConstIterator it;
  OutputIterator out;
  MaskImageIterator maskIt;
  ScalesImageIterator scalesIt;
  GradientImageIterator gradientIt;
  TensorImageIterator hessianIt;
  VectorImageIterator normalsIt;
  
  // Iterate through the regions of the face list first
  
  for( fit = faceList.begin(); fit != faceList.end(); ++fit )
  {
    it = InputConstIterator( inputPtr, *fit );
    out =  OutputIterator( outputPtr, *fit );
    maskIt = MaskImageIterator( maskPtr, *fit );
    scalesIt = ScalesImageIterator( scalesPtr, *fit );
    gradientIt = GradientImageIterator( this->m_GradientMagnitudeGaussian->GetOutput(), *fit );
    hessianIt = TensorImageIterator( this->m_HessianGaussian->GetOutput(), *fit );
    normalsIt = VectorImageIterator( normalsPtr, *fit );
  
    if( fit == faceList.begin() ) // first face is the inner region so don't check bound
      checkBounds = false;
    else
      checkBounds = true;


    for ( it.GoToBegin(), out.GoToBegin(), maskIt.GoToBegin(), gradientIt.GoToBegin(), hessianIt.GoToBegin(),
      scalesIt.GoToBegin(), normalsIt.GoToBegin();  !out.IsAtEnd(); ++it, ++out, ++maskIt, 
      ++gradientIt, ++hessianIt, ++scalesIt, ++normalsIt )
    {
        if( !maskIt.Get() )
          continue; // process next pixel
        
        currentIndex = out.GetIndex();

        // Check that the hessian matrix is not a zero matrix. This avoids problems with NaN eigenvalues and eigenvectors
        if( hessianIt.Get().GetTrace() < 1e-3 )
          continue;
    
        // Calculate eigenvalues and eigenvectors at the current location
        m_EigenAnalysis.ComputeEigenValuesAndVectors( hessianIt.Get(), eigenValues, eigenVectors );

        // Recover eigenvectors from the matrix (first two rows)
        for( unsigned int i=0; i<ImageDimension; ++i )
        {
          firstEigenVector[i]  = eigenVectors( 0, i );
          secondEigenVector[i] = eigenVectors( 1, i );
          thirdEigenVector[i]  = eigenVectors( 2, i );
        }

        // Do not process those pixels whose first two eigenvalues are not negative
        
        if( m_FilterByEigenValues )
        {
          if( eigenValues[0] >= 0.0f || eigenValues[1] >= 0.0f )
            continue;
        }
        
      
        //////////////////////////
        // Medialness calculation

        // Get the physical coordinates of the current index
        inputPtr->TransformIndexToPhysicalPoint( currentIndex, currentPoint );
        
        calculatedSamples = 0;

        // Calculate points in a circle defined by the two eigenvectors with r = radius
        for ( unsigned int i=0; i<samples; ++i )
        {
          // Evaluate and store the value of the gradient in a circle 
          
          for( int k=0; k<ImageDimension; ++k )
            currentCirclePoint[k] = currentPoint[k] + radius * ( cosArray[i] * firstEigenVector[k] + sinArray[i] * secondEigenVector[k] );
            
          m_GradientInterpolator->ConvertPointToNearestIndex( currentCirclePoint, currentCircleIndex );
          
          
          if( checkBounds )
          {
            if( m_GradientInterpolator->IsInsideBuffer( currentCirclePoint ) && 
              maskPtr->GetPixel( currentCircleIndex ) )
            {
              radialMedialness[i] = m_GradientInterpolator->Evaluate( currentCirclePoint );
              ++calculatedSamples;
            }
            else
              radialMedialness[i] = 0.0;
          }
          
          else // checkBounds
          {
            if( maskPtr->GetPixel( currentCircleIndex ) )
            {
              radialMedialness[i] = m_GradientInterpolator->Evaluate( currentCirclePoint );
              ++calculatedSamples;
            }
            else
              radialMedialness[i] = 0.0;
          }
          
        }
        
        medialness = 0.0;
        
        if( calculatedSamples > 0 )
        {
          if( m_SymmetryCoefficient == 1.0 ) // speed-up calculations for the simplest (default) case
          {
            for ( unsigned int i=0; i<samples; ++i )
              medialness += radialMedialness[i];  
          }
          
          else
          {
            averageRadialMedialness = radialMedialness.one_norm() / static_cast<double>( calculatedSamples );
              
            for ( unsigned int i=0; i<samples; ++i )
            {
              if( !radialMedialness[i] )
                continue;
              
              // Calculate b coefficients
              weightExponent = ( 1.0 - radialMedialness[i] / averageRadialMedialness ) / m_SymmetryCoefficient;
              radialMedialnessWeights[i] = exp( -0.5 * weightExponent * weightExponent );
              medialness += radialMedialnessWeights[i] * radialMedialness[i];
              //medialness += radial_medialness[i];
            }
          
          }
  
          medialness /= static_cast<float>( calculatedSamples );
        
        }

                
        // Now calculate final medialness using an adaptative threshold that depends on the central boundariness
        // (value of gradient at the central point). Vessel centers are supposed to have a low value of boundariness
        // and thus this is penalized. A threshold is introduced here for noise etc.
        
        centralBoundariness = gradientIt.Get();

        // Normalization factor. Here we have disabled AccrosScaleNormalization in the gradient
        // calculation but now we multiply by sigma to get sigma-normalized derivatives (see Krissian)
        medialness *= currentScale;
        centralBoundariness *= currentScale;
        
        
        // !!! FUNCIONA MEJOR ASI, xq si no muchos valores dan negativo y los ponemos a cero, sin embargo pueden ser
        // significativos. También es una manera de que la intensidad de la respuesta sea mayor
        if( medialness - centralBoundariness < m_OutputThreshold ) 
          medialness = 0.0f;

        if( medialness > out.Get() ) 
        {
          pixelCount++;
          out.Set( medialness ); // store medialness value
          scalesIt.Set( currentScale );
          normalsIt.Set( thirdEigenVector );
        }
    }
  }
}

} // end namespace ivan

#endif
