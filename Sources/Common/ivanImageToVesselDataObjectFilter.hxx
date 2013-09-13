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
// File: ivanImageToVesselDataObjectFilter.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2009/02/06


#ifndef __ivanImageToVesselDataObjectFilter_hxx
#define __ivanImageToVesselDataObjectFilter_hxx

#include "ivanImageToVesselDataObjectFilter.h"

namespace ivan
{
  
/**
 *
 */
template <class TInputImage, class TOutputVessel>
ImageToVesselDataObjectFilter<TInputImage,TOutputVessel>
::ImageToVesselDataObjectFilter()
{
  // Modify superclass default values, can be overridden by subclasses
  this->SetNumberOfRequiredInputs(1);
}

/**
 *
 */
template <class TInputImage, class TOutputVessel>
void 
ImageToVesselDataObjectFilter<TInputImage,TOutputVessel>
::SetInput(const InputImageType *image)
{
  // Process object is not const-correct so the const_cast is required here
  this->itk::ProcessObject::SetNthInput( 0, const_cast< InputImageType * >( image ) );
}


/**
 * Connect one of the operands for pixel-wise addition
 */
template <class TInputImage, class TOutputVessel>
void 
ImageToVesselDataObjectFilter<TInputImage,TOutputVessel>
::SetInput( unsigned int index, const TInputImage * image ) 
{
  // Process object is not const-correct so the const_cast is required here
  this->itk::ProcessObject::SetNthInput( index, const_cast< TInputImage *>( image ) );
}


/**
 *
 */
template <class TInputImage, class TOutputVessel>
const typename ImageToVesselDataObjectFilter<TInputImage,TOutputVessel>::InputImageType *
ImageToVesselDataObjectFilter<TInputImage,TOutputVessel>
::GetInput(void) 
{
  if ( this->GetNumberOfInputs() < 1)
    {
    return 0;
    }
  
  return static_cast<const TInputImage * >
    ( this->itk::ProcessObject::GetInput(0) );
}
  
/**
 *
 */
template <class TInputImage, class TOutputVessel>
const typename ImageToVesselDataObjectFilter<TInputImage,TOutputVessel>::InputImageType *
ImageToVesselDataObjectFilter<TInputImage,TOutputVessel>
::GetInput(unsigned int idx)
{
  return static_cast< const TInputImage * >( this->itk::ProcessObject::GetInput(idx) );
}


/**
 *
 */
template <class TInputImage, class TOutputVessel>
void 
ImageToVesselDataObjectFilter<TInputImage,TOutputVessel>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf(os, indent);
}

} // end namespace ivan

#endif
