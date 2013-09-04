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
// File: ivanLinearVesselCenterlineInterpolator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06

#ifndef __ivanLinearVesselCenterlineInterpolator_hxx
#define __ivanLinearVesselCenterlineInterpolator_hxx

#include "ivanLinearVesselCenterlineInterpolator.h"


namespace ivan
{

template <class TCenterline>
LinearVesselCenterlineInterpolator<TCenterline>::LinearVesselCenterlineInterpolator()
{

}


template <class TCenterline>
LinearVesselCenterlineInterpolator<TCenterline>::~LinearVesselCenterlineInterpolator()
{

}


template <class TCenterline>
LinearVesselCenterlineInterpolator<TCenterline>::CenterlinePointer 
LinearVesselCenterlineInterpolator<TCenterline>::Evaluate( CenterlineConstPointer input )
{
  
  
}

template <class TCenterline>
void LinearVesselCenterlineInterpolator<TCenterline>::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  
}

} // end namespace ivan

#endif // __ivanLinearVesselCenterlineInterpolator_hxx
