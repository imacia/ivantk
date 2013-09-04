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
// File     : itkRadiusAndSectionEstimationVesselNetworkFilter.h
// Author   : Iv�n Mac�a (imacia@vicomtech.org)
// Date     : January 2007
// Revision : 1 
// VICOMTech, Spain
// http://www.vicomtech.org


#ifndef __ivanRadiusAndSectionEstimationVesselNetworkFilter_h
#define __ivanRadiusAndSectionEstimationVesselNetworkFilter_h

#include "ivanVesselNetworkSource.h"
#include "ivanVesselNetwork.h"

namespace ivan
{
  
/** \class RadiusAndSectionEstimationVesselNetworkFilter
 * \brief Estimates radius and section at every point of a VesselNetwork using optimization.
 *
 * This class estimates both the radius and vessel section at every
 * point of a vessel network using an optimization scheme. The optimization scheme
 * is based on the medialness values calculated which is estimated by the sum of 
 * weighted values of the gradient at a circle with the given radius.
 * 
 * This filter requires two inputs, the input vessel network and the input image whose
 * values will be used to evaluate the medialness values. 
 *
 */

template <class TInputNetwork, class TOutputNetwork, 
  class TSourceImage>
class ITK_EXPORT RadiusAndSectionEstimationVesselNetworkFilter 
: public VesselNetworkToVesselNetworkFilter<TInputNetwork,TOutputNetwork>
{
public:
	
  /** Standard class typedefs. */
  typedef RadiusAndSectionEstimationVesselNetworkFilter
    <TInputNetwork,TOutputNetwork,TSourceImage>   Self;
  typedef VesselNetworkToVesselNetworkFilter
    <TInputNetwork,TOutputNetwork>   Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
  
  /** Image types. */
  typedef TSourceImage   SourceImageType;
	
  	
public:
  
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(RadiusAndSectionEstimationVesselNetworkFilter,
    VesselNetworkToVesselNetworkFilter);
    
  /** Set/Get the source image.  */
  void SetSourceImage( const SourceImageType * sourceImage );
  const SourceImageType * GetSourceImage() const;
  
protected:
  RadiusAndSectionEstimationVesselNetworkFilter();
  ~RadiusAndSectionEstimationVesselNetworkFilter() {}
  
  void GenerateData();
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  RadiusAndSectionEstimationVesselNetworkFilter(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
  
private:
	
	
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "itkRadiusAndSectionEstimationVesselNetworkFilter.hxx"
#endif

#endif
