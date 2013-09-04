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
// File: ivanLinearPropertyInterpolator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanLinearPropertyInterpolator_h
#define __ivanLinearPropertyInterpolator_h

#include "itkArray.h"
#include "itkVector.h"
#include "itkCovariantVector.h"


namespace ivan
{
  
/** \class LinearPropertyInterpolator
 *  \brief Template class for linear interpolation
 *
 * This class is a helper class for linearly interpolating integral or other types
 * through template specialization
 *
 * \ingroup 
 */

template <class T>
class ITK_EXPORT LinearPropertyInterpolator
{
public:

  typedef LinearPropertyInterpolator   Self;
  
  T Interpolate( const T & value1, const T & value2, double pos1, double pos2, double pos )
    {
      return static_cast<T>( value1 + ( ( pos - pos1 ) / ( pos2 - pos1 ) )( value2 - value1 ) ); 
    }
};

// Specialization for itk::Array
template <class T,unsigned int VDimension>
class ITK_EXPORT LinearPropertyInterpolator<itk::Array<T,VDimension> >
{
  typedef LinearPropertyInterpolator   Self;
  typedef itk::Array<T,VDimension>     ArrayType;
  
  ArrayType Interpolate( const ArrayType & vec1, const ArrayType & vec2, double pos1, 
    double pos2, double pos )
    {
      double multi = ( pos - pos1 ) / ( pos2 - pos1 );
      ArrayType vec;
      
      for( unsigned int i=0; i<VDimension; ++i )
        vec[i] = vec1[i] + multi * ( vec2[i] - vec1[i] );
        
      return vec; 
    }
};


// Specialization for itk::Vector
template <class T,unsigned int VDimension>
class ITK_EXPORT LinearPropertyInterpolator<itk::Vector<T,VDimension> >
{
  typedef LinearPropertyInterpolator   Self;
  typedef itk::Vector<T,VDimension>    VectorType;
  
  VectorType Interpolate( const VectorType & vec1, const VectorType & vec2, double pos1, 
    double pos2, double pos )
    {
      double multi = ( pos - pos1 ) / ( pos2 - pos1 );
      VectorType vec;
      
      for( unsigned int i=0; i<VDimension; ++i )
        vec[i] = vec1[i] + multi * ( vec2[i] - vec1[i] );
        
      return vec; 
    }
};

// Specialization for itk::CovariantVector
template <class T,unsigned int VDimension>
class ITK_EXPORT LinearPropertyInterpolator<itk::CovariantVector<T,VDimension> >
{
  typedef LinearPropertyInterpolator            Self;
  typedef itk::CovariantVector<T,VDimension>    VectorType;
  
  VectorType Interpolate( const VectorType & vec1, const VectorType & vec2, double pos1, double pos2, double pos )
    {
      double multi = ( pos - pos1 ) / ( pos2 - pos1 );
      VectorType vec;
      
      for( unsigned int i=0; i<VDimension; ++i )
        vec[i] = vec1[i] + multi * ( vec2[i] - vec1[i] );
        
      return vec; 
    }
};

} // end namespace ivan

#endif
