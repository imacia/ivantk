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
// File: VTKVesselTrackerPlotter.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2009/02/10
// Description: reads an input image and displays calculated vessel points and normals


#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "itkLinearInterpolateImageFunction.h"
#include "itkConstNeighborhoodIterator.h"
#include "itkImageRegionIterator.h"

#include "itkCastImageFilter.h"
#include "itkImageToVTKImageFilter.h"
#include "itkMinimumMaximumImageCalculator.h"
#include "itkGradientMagnitudeRecursiveGaussianImageFilter.h"
#include "itkHessianRecursiveGaussianImageFilter.h"
#include "itkSymmetricSecondRankTensor.h"
#include "itkSymmetricEigenAnalysis.h"
#include "itkArray.h"

#include "itkOnePlusOneEvolutionaryOptimizer.h"
#include "itkNormalVariateGenerator.h" 

#include "ivanImageBasedVesselSectionFitCostFunction.h"

#include "vtkPlaneSource.h"
#include "vtkAppendPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkActor.h"
#include "vtkProperty.h"
#include "vtkRenderWindowInteractor.h"

#include "vtkVolume.h"
#include "vtkVolumeRayCastMIPFunction.h"
#include "vtkVolumeRayCastIsosurfaceFunction.h"
#include "vtkFixedPointVolumeRayCastMapper.h"
#include "vtkColorTransferFunction.h"
#include "vtkPiecewiseFunction.h"
#include "vtkVolumeProperty.h"
#include "vtkImagePlaneWidget.h"
#include "vtkDataObjectWriter.h"

#include "vnl/vnl_cross.h"
#include "vnl/vnl_vector.h"

#include <string>
#include <fstream>

#define PLANE_HALF_WIDTH 10.0f

int main( int argc, char *argv[] )
{
  if( argc < 3 )
    {
			std::cerr << "Usage: sourceImage vesselTextFile" << std::endl;
			return EXIT_FAILURE;
		}
  
  const   unsigned int        Dimension       = 3;
  typedef unsigned char       PixelType;
	typedef unsigned short			VolumePixelType;
  
  typedef itk::Image<PixelType, Dimension>   						  ImageType;
  typedef itk::Image<VolumePixelType, Dimension>   			  VolumeImageType;
  
 	// Readers / writers
 	typedef itk::ImageFileReader<ImageType>  							  ReaderType;
 	 	
 	// Filter types
 	typedef itk::ImageToVTKImageFilter<VolumeImageType>			ITKToVTKImageAdapterType;
 	typedef itk::CastImageFilter<ImageType,VolumeImageType>	CastFilterType;
 	typedef itk::MinimumMaximumImageCalculator<ImageType>	  MinMaxCalculatorType;
 	
  
  ReaderType::Pointer sourceReader = ReaderType::New();
  sourceReader->SetFileName( argv[1] );
	
	try
  {
    sourceReader->Update();
  }
  catch ( itk::ExceptionObject &err)
  {
    std::cout << "ExceptionObject caught !" << std::endl; 
    std::cout << err << std::endl; 
    return -1;
  } 
  
  ImageType::Pointer sourceImage = sourceReader->GetOutput();
  sourceImage->DisconnectPipeline();
  sourceReader = 0;
  

  // Read the input data  

  typedef itk::FixedArray<double,Dimension>   ArrayType;
  typedef std::vector<ArrayType>              ArrayContainerType;
    
  ArrayContainerType centers;
  ArrayContainerType normals;
  
  ArrayType currentCenter, currentNormal;
  
  char buffer[1024];
  unsigned int idx;
  
  std::ifstream filein;
  filein.open( argv[2] );
  filein.ignore( 1024, '\n' ); // ignore first line
  //fileout.getline( buffer, 1023 );

  while( !filein.eof() )
  {
    filein.getline( buffer, 1023 );
    sscanf( buffer, "%d [%lf,%lf,%lf] [%lf,%lf,%lf]\n", &idx, &currentCenter[0], &currentCenter[1], &currentCenter[2],
      &currentNormal[0], &currentNormal[1], &currentNormal[2] );
    centers.push_back( currentCenter );
    normals.push_back( currentNormal );
  }
  
  filein.close();
  
	double a, b, c;
  itk::Array<double>    normalVector;
	itk::Array<double>		firstBaseVector; 
	itk::Array<double>	 	secondBaseVector;
  normalVector.SetSize( Dimension );
	firstBaseVector.SetSize( Dimension );
	secondBaseVector.SetSize( Dimension );

	vtkAppendPolyData *appendPolyData = vtkAppendPolyData::New();
	appendPolyData->UserManagedInputsOff();
  	
	for( unsigned int i=0; i<centers.size(); ++i )
	{
	  // Calculate an orthonormal base of vectors in the plane of the section.
		// The plane equation can be expressed as :
		// a*(x-x0) + b*(y-y0) + c(z-z0) = 0 where (a,b,c) is the plane normal and (x0,y0,z0) is a point in the 
		// plane, for example the center. We are interested in calculating a unit vector ( (x-x0),(y-y0),(z-z0) ).
		// From the equations :
		// 1) a*(x-x0) + b*(y-y0) + c(z-z0) = 0
		// 2) (x-x0)^2 + (y-y0)^2 + (z-z0)^2 = 1 (unit vector)
		// 3) a^2 + b^2 + c^2 = 1 (normal is unit vector)
		// we get
		// (z-z0) = ( -2bc(y-y0) +- sqrt( 4 * (b^2+c^2-1) * ((y-y0)^2+b^2-1) ) ) / 2(1-b^2)
		// We arbitrarily choose y-y0 = 1-b^2 to make the discriminant zero resulting (z-z0) = -b*c / (y-y0)
		// and (x-x0) = ( -b(y-y0) -c(z-z0) ) / a 
				
		a = normals[i][0];
		b = normals[i][1];
		c = normals[i][2];

    normalVector.SetData( normals[i].GetDataPointer() );
		
    // Avoid degenerated cases
    if( vnl_math_abs( a ) > 5e-3 && vnl_math_abs( b*b - 1.0 ) > 5e-3 )
    {
		  firstBaseVector[1] = sqrt( 1.0 - b*b );
  	  firstBaseVector[2] = ( - b*c ) / firstBaseVector[1];
	    firstBaseVector[0] = ( - b*firstBaseVector[1] - c*firstBaseVector[2] ) / a;
    }
    // if( vnl_math_abs( a ) <= 5e-3 ) normal is in YZ plane, take axis X as firstBaseVector
    // if( vnl_math_abs( b*b - 1.0 ) <= 5e-3 ), normal is Y axis, also take axis X as firstBaseVector 
    else
    {
      firstBaseVector[0] = 1.0;
      firstBaseVector[1] = 0.0;
      firstBaseVector[2] = 0.0;
    }

		secondBaseVector = vnl_cross_3d( firstBaseVector, normalVector );
	  
		// Create a plane and append it to the group of polydata for the coarse (eigenanalysis based) 
		// radius and section estimation
		
		vtkPlaneSource *plane = vtkPlaneSource::New();
		plane->SetOrigin( centers[i][0], centers[i][1], centers[i][2] );
		plane->SetPoint1( centers[i][0] + PLANE_HALF_WIDTH * firstBaseVector[0], centers[i][1] + 
      PLANE_HALF_WIDTH * firstBaseVector[1], centers[i][2] + PLANE_HALF_WIDTH * firstBaseVector[2] );
		plane->SetPoint2( centers[i][0] + PLANE_HALF_WIDTH * secondBaseVector[0], centers[i][1] + 
      PLANE_HALF_WIDTH * secondBaseVector[1], centers[i][2] + PLANE_HALF_WIDTH * secondBaseVector[2] );
		plane->SetCenter( centers[i][0], centers[i][1], centers[i][2] ); // performs a translation
			
#if VTK_MAJOR_VERSION < 6
    appendPolyData->AddInput( plane->GetOutput() );
#else // VTK_MAJOR_VERSION < 6
    appendPolyData->AddInputData( plane->GetOutput() );
#endif // VTK_MAJOR_VERSION < 6
					
		plane->Delete(); // this deletes just this reference, otherwise we have a memory leak each loop cycle
  }
	  
	MinMaxCalculatorType::Pointer minmax = MinMaxCalculatorType::New();
	minmax->SetImage( sourceImage );
	
	try
	{
		minmax->Compute();
	}
	catch ( itk::ExceptionObject &err)
	{
		std::cout << "ExceptionObject caught !" << std::endl; 
		std::cout << err << std::endl; 
		return -1;
	}
	
	ImageType::PixelType min = minmax->GetMinimum();
	ImageType::PixelType max = minmax->GetMaximum();
		
	std::cout << "Min value: " << min << std::endl;
	std::cout << "Max value: " << max << std::endl;
	
	CastFilterType::Pointer caster = CastFilterType::New();
	caster->SetInput( sourceImage );

	try
	{
		caster->Update();
	}
	catch ( itk::ExceptionObject &err)
	{
		std::cout << "ExceptionObject caught !" << std::endl; 
		std::cout << err << std::endl; 
		return -1;
	}

	ITKToVTKImageAdapterType::Pointer adapter = ITKToVTKImageAdapterType::New();
	adapter->SetInput( caster->GetOutput() );
			
	try
	{
		adapter->Update();
	}
	catch ( itk::ExceptionObject &err)
	{
		std::cout << "ExceptionObject caught !" << std::endl; 
		std::cout << err << std::endl; 
		return -1;
	}
	
	vtkFixedPointVolumeRayCastMapper *rayCastMapper = vtkFixedPointVolumeRayCastMapper::New();
#if VTK_MAJOR_VERSION < 6
	rayCastMapper->SetInput( adapter->GetOutput() );
#else // VTK_MAJOR_VERSION < 6
  rayCastMapper->SetInputData( adapter->GetOutput() );
#endif // VTK_MAJOR_VERSION < 6

	//vtkVolumeRayCastIsosurfaceFunction *isoFunction = vtkVolumeRayCastIsosurfaceFunction::New();
	//isoFunction->SetIsoValue( ( max - min ) / 3.0f );
	
  rayCastMapper->SetBlendModeToMaximumIntensity();	
	//rayCastMapper->SetVolumeRayCastFunction( isoFunction );	
		
	vtkColorTransferFunction *colorTransferFunc = vtkColorTransferFunction::New();	
	colorTransferFunc->AddRGBSegment( min * 0.2, 1.0, 1.0, 1.0, max * 0.8, 1.0, 1.0, 1.0 ); // grayscale
	
	/*colorTransferFunc->AddRGBPoint( 0.0, 0.0, 0.0, 0.0);
	colorTransferFunc->AddRGBPoint( 0.353 * max, 1.000, 0.784, 0.816);
	colorTransferFunc->AddRGBPoint( 0.482 * max, 0.588, 0.165, 0.235);
	colorTransferFunc->AddRGBPoint( 0.631 * max, 0.710, 0.204, 0.231);
	colorTransferFunc->AddRGBPoint( 0.643 * max, 0.784, 0.557, 0.396);
	colorTransferFunc->AddRGBPoint( 0.733 * max, 0.796, 0.745, 0.706);
	colorTransferFunc->AddRGBPoint( 0.792 * max, 0.929, 0.776, 0.765);
	colorTransferFunc->AddRGBPoint( 0.871 * max, 0.804, 0.765, 0.706);
	colorTransferFunc->AddRGBPoint( max, 1.0, 1.0, 1.0);*/

	vtkPiecewiseFunction *scalarOpacityTransferFunc = vtkPiecewiseFunction::New();
	scalarOpacityTransferFunc->ClampingOn();
	scalarOpacityTransferFunc->AddPoint( min +( max - min ) * 0.4, 0.0 );
	scalarOpacityTransferFunc->AddPoint( static_cast<float>( max - min ), 1.0 );
	
	vtkVolumeProperty *volumeProperty = vtkVolumeProperty::New();
	volumeProperty->SetColor( colorTransferFunc );
	volumeProperty->SetScalarOpacity( scalarOpacityTransferFunc );
	volumeProperty->SetInterpolationTypeToLinear();//
		
	vtkVolume *volume = vtkVolume::New();
	volume->SetMapper( rayCastMapper );
	volume->SetProperty( volumeProperty );
					
	vtkPolyDataMapper *planeMapper = vtkPolyDataMapper::New();
#if VTK_MAJOR_VERSION < 6
	planeMapper->SetInput( appendPolyData->GetOutput() );
#else // VTK_MAJOR_VERSION < 6
  planeMapper->SetInputData( appendPolyData->GetOutput() );
#endif // VTK_MAJOR_VERSION < 6
  	
	vtkActor *planeActor = vtkActor::New();
	planeActor->SetMapper( planeMapper );
  planeActor->GetProperty()->SetColor( 1.0f, 0.0f, 0.0f );
	//planeActor->GetProperty()->SetInterpolationToGouraud();
	//planeActor->GetProperty()->SetRepresentationToWireframe();
	planeActor->GetProperty()->BackfaceCullingOff();
  
	vtkRenderer* renderer = vtkRenderer::New();
  renderer->AddVolume( volume );
	renderer->AddActor( planeActor );
  renderer->LightFollowCameraOn();
  renderer->TwoSidedLightingOn();
  	
	/*PlaneCollectionType::const_iterator it;

	for( it = planeCollection.begin(); it != planeCollection.end(); ++it )
	{
		vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
		mapper->SetInput( (*it)->GetOutput() );
		   
		vtkActor *actor = vtkActor::New();
		actor->GetProperty()->SetColor( 0.0f, 1.0f, 0.0f );
		//actor->GetProperty()->SetInterpolationToGouraud();
		actor->GetProperty()->SetRepresentationToWireframe();
		actor->GetProperty()->BackfaceCullingOff();
		actor->SetMapper( mapper );
		
		renderer->AddActor( actor );

		mapper->Delete();
		actor->Delete();
		
	}*/

	vtkRenderWindow* window = vtkRenderWindow::New();
	window->AddRenderer( renderer );
	window->SetSize( 1024, 768 );
	window->SetPosition( 256, 128 );

	vtkRenderWindowInteractor* interactor = vtkRenderWindowInteractor::New();
	interactor->SetRenderWindow( window );

/*	vtkImagePlaneWidget *planeWidget = vtkImagePlaneWidget::New();
	planeWidget->SetInteractor( interactor );
	planeWidget->SetInput( adapter->GetOutput() );
	planeWidget->EnabledOn();
	//planeWidget->InteractionOn();
	planeWidget->TextureVisibilityOn();
	
	VolumeImageType::PointType origin = sourceImage->GetOrigin();
	VolumeImageType::RegionType region = sourceImage->GetLargestPossibleRegion();
	VolumeImageType::RegionType::SizeType size = region.GetSize();

	planeWidget->SetPlaneOrientationToZAxes();
	planeWidget->SetOrigin( origin[0], origin[1], origin[2] );
	planeWidget->SetPoint1( origin[0] + size[0] * spacing[0], origin[1], origin[2] );
	planeWidget->SetPoint2( origin[0], origin[1] + size[1] * spacing[1], origin[2] );
	planeWidget->SetCurrentRenderer( renderer );
	planeWidget->SetSliceIndex(0);
	planeWidget->SetSlicePosition( 0.0 );
	planeWidget->RestrictPlaneToVolumeOn();
	planeWidget->SetLeftButtonAction( vtkImagePlaneWidget::SLICE_MOTION_ACTION );*/
		

	// Write the vtk data to file
	vtkDataObjectWriter *planesWriter = vtkDataObjectWriter::New();
#if VTK_MAJOR_VERSION < 6
	planesWriter->SetInput( appendPolyData->GetOutput() );
#else // VTK_MAJOR_VERSION < 6
  planesWriter->SetInputData( appendPolyData->GetOutput() );
#endif // VTK_MAJOR_VERSION < 6
	planesWriter->SetFileName( "SectionPlanes.vtk" );
	planesWriter->SetFileTypeToASCII();
	planesWriter->Update();
	planesWriter->Delete();

	window->Render();
	interactor->Start();


	//vtkImageViewer2 *imageViewer = vtkImageViewer2::New();
	//imageViewer->SetInput(
	
	
	// Delete vtk objects
	
		
	interactor->Delete();
	window->Delete();
	//planeWidget->Delete();
	renderer->Delete();
	planeActor->Delete();
	planeMapper->Delete();
	appendPolyData->Delete();
	volume->Delete();
	volumeProperty->Delete();
	scalarOpacityTransferFunc->Delete();
	colorTransferFunc->Delete();
//	mipFunction->Delete();
//	isoFunction->Delete();
	rayCastMapper->Delete();
	//imageViewer->Delete();
		
	std::cout << "Finished." << std::endl;
  
  return EXIT_SUCCESS;
}
