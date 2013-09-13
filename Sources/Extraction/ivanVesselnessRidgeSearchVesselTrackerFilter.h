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
// File: ivanVesselnessRidgeSearchVesselTrackerFilter.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2012/02/12


#ifndef __ivanVesselnessRidgeSearchVesselTrackerFilter_h
#define __ivanVesselnessRidgeSearchVesselTrackerFilter_h

#include "ivanVesselnessBasedSearchVesselTrackerFilter.h"

#include "itkArray.h"


namespace ivan
{

/** \class VesselnessRidgeSearchVesselTrackerFilter
 * \brief VesselTracker that finds local ridge of vesselness to guide the search stage.
 *
 * VesselnessRidgeSearchVesselTrackerFilter searches for the local ridges of vesselness 
 * in order to find the best plane center and normal.
 *
 */

template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
class ITK_EXPORT VesselnessRidgeSearchVesselTrackerFilter : 
  public VesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
{
public:
	
  /** Standard class typedefs. */
  typedef VesselnessRidgeSearchVesselTrackerFilter
    <TInputImage,TOutputVessel,TVesselnessFunction>      Self;
  typedef VesselnessBasedSearchVesselTrackerFilter
    <TInputImage,TOutputVessel,TVesselnessFunction>      Superclass;
  typedef itk::SmartPointer<Self>                        Pointer;
  typedef itk::SmartPointer<const Self>                  ConstPointer;
  
  /** Some convenient typedefs. */
  typedef TInputImage                                    InputImageType;
  typedef typename Superclass::InputImagePointer         InputImagePointer;
  typedef typename Superclass::InputImageConstPointer    InputImageConstPointer;
  typedef typename Superclass::InputImageRegionType      InputImageRegionType; 
  typedef typename Superclass::InputImagePixelType       InputImagePixelType;
  typedef typename Superclass::InputImagePointType       InputImagePointType;
  typedef typename Superclass::InputImageIndexType       InputImageIndexType; 
  	
  /** Some convenient typedefs. */
  typedef TOutputVessel                                  OutputVesselType;
  typedef typename OutputVesselType::Pointer             OutputVesselPointer;
  
  typedef typename Superclass::CenterlineType            CenterlineType;
  typedef typename Superclass::SectionType               SectionType;

  typedef itk::Image<typename TVesselnessFunction::OutputType, 
    InputImageType::ImageDimension>                      VesselnessImageType;
      
  typedef TVesselnessFunction                            VesselnessFunctionType;
  typedef typename VesselnessFunctionType::Pointer       VesselnessFunctionPointer;
  typedef typename VesselnessFunctionType::OutputType    VesselnessValueType;

public:
  
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselnessRidgeSearchVesselTrackerFilter, VesselnessBasedSearchVesselTrackerFilter );
  
  /** Set/Get the maximum distance to search for a vesselness maximum from the initial center point. */
  itkSetMacro( MaximumSearchDistance, double );
  itkGetConstMacro( MaximumSearchDistance, double );
  
  /** Set/Get use adaptative sampling. In adaptative sampling, the number of samples in the circle
    * depends on the current radius value, and the radial resolution is ignored. If this flag is set
    * to one, make sure that the centerline model stores the estimated radius values. */
  virtual void SetAdaptativeSampling( bool adaptative );
  itkGetConstMacro( AdaptativeSampling, bool );
  itkBooleanMacro( AdaptativeSampling );
  
  /** Set/Get the radial resolution. */
  itkSetMacro( RadialResolution, unsigned int );
  itkGetConstMacro( RadialResolution, unsigned int );
     
  /** Set/Get the angular resolution. */
  virtual void SetAngularResolution( const unsigned int resolution );
  itkGetConstMacro( AngularResolution, unsigned int );
  
protected:
  
  VesselnessRidgeSearchVesselTrackerFilter();
  ~VesselnessRidgeSearchVesselTrackerFilter() {}
  
  /** Pre-computes some calculations used in the sampling. */ 
  void ComputeIntervals();
  
  /** Reimplemente the search for the optimum as a local optimization of the vesselness function. */
  virtual void Search();
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  VesselnessRidgeSearchVesselTrackerFilter(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
  
protected:

  /** Maximum distance to search for a vesselness maximum from the initial center point. */
  double               m_MaximumSearchDistance;
  
  /** Number of points taken radially. */
  unsigned int         m_RadialResolution;
  
  /** Number of points for each radius value. */
  unsigned int         m_AngularResolution;

  /** In adaptative sampling, the number of samples in the circle
    * depends on the current scale, and the radial resolution is ignored. */
  bool                 m_AdaptativeSampling;
  
  /** Cached angular values. */
  itk::Array<double>   m_CosArray;
  itk::Array<double>   m_SinArray;
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanVesselnessRidgeSearchVesselTrackerFilter.hxx"
#endif

#endif
