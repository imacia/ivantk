/*=========================================================================

Image-based Vascular Analysis Toolkit (IVAN)

Copyright (c) 2012-2013, Ivan Macia Oliver
Vicomtech Foundation, San Sebastian - Donostia (Spain)
University of the Basque Country, San Sebastian - Donostia (Spain)

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
// File: ivanVesselGlobals.h
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: global functions used in the library
// Date: 2009/11/05


#ifndef __ivanVesselGlobals_h
#define __ivanVesselGlobals_h


#include "vnl/vnl_cross.h"


namespace ivan
{

/** Compute two base vectors for a plane given normal. The class requires either a vnl_vector ,
  * a vnl_vector_fixed or subclasses .*/
template <class TVector>
void ComputePlaneBasisVectorsFromNormal( const TVector & normalVector, TVector & firstBasisVector, TVector & secondBasisVector )
{
  // Calculate an orthonormal basis of vectors in the plane of the section.
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
	  
  double a, b, c; // better use this for clarity
        
  // Recover normal vector of the section plane
  a = normalVector[0];
  b = normalVector[1];
  c = normalVector[2];
  
  // Avoid degenerated cases
  if( vnl_math_abs( a ) > 5e-3 && vnl_math_abs( b*b - 1.0 ) > 5e-3 )
  {
	  firstBasisVector[1] = sqrt( 1.0 - b*b );
	  firstBasisVector[2] = ( - b*c ) / firstBasisVector[1];
    firstBasisVector[0] = ( - b*firstBasisVector[1] - c*firstBasisVector[2] ) / a;
  }
  // if( vnl_math_abs( a ) <= 5e-3 ) normal is in YZ plane, take axis X as firstBaseVector
  // if( vnl_math_abs( b*b - 1.0 ) <= 5e-3 ), normal is Y axis, also take axis X as firstBaseVector 
  else
  {
    firstBasisVector[0] = 1.0;
    firstBasisVector[1] = 0.0;
    firstBasisVector[2] = 0.0;
  }

	secondBasisVector = vnl_cross_3d( firstBasisVector, normalVector );  
}

} // end namespace ivan

#endif
