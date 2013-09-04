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
// File: ivanVesselDataObjectSource.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/05/27


#ifndef __ivanVesselDataObjectSource_hxx
#define __ivanVesselDataObjectSource_hxx

#include "ivanVesselDataObjectSource.h"


namespace ivan
{

/**
 *
 */
template <class TOutputVessel>
VesselDataObjectSource<TOutputVessel>
::VesselDataObjectSource()
{
  // Create the output. We use static_cast<> here because we know the default
  // output must be of type TOutputVessel
  OutputVesselPointer output
    = static_cast<TOutputVessel*>(this->MakeOutput(0).GetPointer()); 

  this->ProcessObject::SetNumberOfRequiredOutputs(1);
  this->ProcessObject::SetNthOutput( 0, output.GetPointer() );

  // Initialize VesselDataObjectSource member data
}


/**
 *
 */
template <class TOutputVessel>
typename VesselDataObjectSource<TOutputVessel>::OutputVesselType *
VesselDataObjectSource<TOutputVessel>
::GetOutput(void)
{
  if (this->GetNumberOfOutputs() < 1)
    {
    return 0;
    }
  
  return static_cast<TOutputVessel*>(this->ProcessObject::GetOutput(0));
}

  
/**
 *
 */
template <class TOutputVessel>
typename VesselDataObjectSource<TOutputVessel>::OutputVesselType *
VesselDataObjectSource<TOutputVessel>
::GetOutput(unsigned int idx)
{
  return static_cast<TOutputVessel*>(this->ProcessObject::GetOutput(idx));
}


/**
 *
 */
template <class TOutputVessel>
typename VesselDataObjectSource<TOutputVessel>::DataObjectPointer
VesselDataObjectSource<TOutputVessel>
::MakeOutput(unsigned int)
{
  return static_cast<itk::DataObject*>( TOutputVessel::New().GetPointer() );
}


/**
 *
 */
template <class TOutputVessel>
void 
VesselDataObjectSource<TOutputVessel>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
}

} // end namespace ivan

#endif
