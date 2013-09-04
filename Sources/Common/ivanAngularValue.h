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
// File: ivanAngularValue.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanAngularValue_h
#define __ivanAngularValue_h


#include "itkNumericTraits.h"

#include "vnl/vnl_math.h"


namespace ivan
{
  
template <class T>
class ITK_EXPORT AngularValueInDegrees
{
public:

  typedef AngularValueInDegrees  Self;
  typedef T                      ValueType;
  
  itkStaticConstMacro( ZeroTurn, T, static_cast<T>(0) );
  itkStaticConstMacro( QuarterTurn, T, static_cast<T>(90) );
  itkStaticConstMacro( HalfTurn, T, static_cast<T>(180) );
  itkStaticConstMacro( FullTurn, T, static_cast<T>(360) );
};


template <class T>
class ITK_EXPORT AngularValueInRadians
{
public:

  typedef AngularValueInRadians  Self;
  typedef T                      ValueType;
  
  itkStaticConstMacro( ZeroTurn, T, static_cast<T>( 0.0 ) );
  itkStaticConstMacro( QuarterTurn, T, static_cast<T>( 0.5 * vnl_math::pi ) );
  itkStaticConstMacro( HalfTurn, T, static_cast<T>( vnl_math::pi ) );
  itkStaticConstMacro( FullTurn, T, static_cast<T>( 2.0 * vnl_math::pi ) );
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanAngularValue.hxx"
#endif

#endif
