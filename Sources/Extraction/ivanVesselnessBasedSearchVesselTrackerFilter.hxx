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
// File: VesselnessBasedSearchVesselTrackerFilter.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2011/01/21


#ifndef __ivanVesselnessBasedSearchVesselTrackerFilter_hxx
#define __ivanVesselnessBasedSearchVesselTrackerFilter_hxx

#include "ivanVesselnessBasedSearchVesselTrackerFilter.h"
#include "ivanVesselBranchNode.h"


namespace ivan
{
  
/**
 *
 */
template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
VesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::VesselnessBasedSearchVesselTrackerFilter()
{
  // This can be provided externally, but at least create a default one
  this->m_VesselnessFunction = VesselnessFunctionType::New();
    
  this->m_VesselnessFunctionInitializer = VesselnessFunctionInitializerType::New(); // provide at least default implementation
}


template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
void 
VesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::SetInput( const InputImageType *image )
{
  Superclass::SetInput( image );
  
  // !!! NOOOOOOOO The type of the vesselness function input and the image type may not
  // necessarily be the same
  //this->m_VesselnessFunction->SetInputImage( image );
}


template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
void 
VesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::SetInput( unsigned int idx, const InputImageType * image )
{
  Superclass::SetInput( idx, image );
  
  // !!! NOOOOOOOO The type of the vesselness function input and the image type may not
  // necessarily be the same
  //this->m_VesselnessFunction->SetInputImage( image );
}


/**
 *
 */
template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
typename VesselnessBasedSearchVesselTrackerFilter
  <TInputImage,TOutputVessel,TVesselnessFunction>::VesselnessValueType
VesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::GetVesselnessValueAtCurrentPoint()
{
  if( this->GetVesselnessImageFunction()->IsInsideBuffer( this->GetCurrentPoint() ) )
    return this->GetVesselnessImageFunction()->Evaluate( this->GetCurrentPoint() );
  else
    return 0.0;
}


/**
 *
 */
template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
typename VesselnessBasedSearchVesselTrackerFilter
  <TInputImage,TOutputVessel,TVesselnessFunction>::VesselnessValueType
VesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::GetVesselnessValueAtPreviousPoint()
{
  if( this->GetVesselnessImageFunction()->IsInsideBuffer( this->GetPreviousPoint() ) )
    return this->GetVesselnessImageFunction()->Evaluate( this->GetPreviousPoint() ); 
  else
    return 0.0;
}


/**
 *
 */
template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
void 
VesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::PrintSelf(std::ostream& os, itk::Indent indent) const
{
  Superclass::PrintSelf(os, indent);
  
}

} // end namespace ivan

#endif
