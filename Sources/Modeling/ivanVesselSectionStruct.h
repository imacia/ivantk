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
// File: ivanNullVesselSectionStruct.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanNullVesselSectionStruct_h
#define __ivanNullVesselSectionStruct_h

#include "ivanVesselSectionStruct.h"


namespace ivan
{
  
/** \class NullVesselSectionStruct
 *  \brief Struct-like object for storing properties/metrics for vessel sections
 *
 * This serves as a template parameter for vessel sections that don't need to 
 * calculate/store any measurement or property.
 *
 * \ingroup 
 */

class ITK_EXPORT NullVesselSectionStruct
{
public:

  typedef NullVesselSectionStruct   Self;
  typedef VesselSectionStruct       Superclass;
  
  NullVesselSectionStruct();
  ~NullVesselSectionStruct();
  
  /** Measurements are struct-like classes that need to define copy ctors. */
  NullVesselSectionStruct( const Self & other ) {}
  
  /** Measurements are struct-like classes that need to define assignment operators. */
  Self & operator = ( const Self & other )
    { return (*this); }
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const {}
};

} // end namespace ivan

#endif
