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
// File: ivanIcosahedronMeshSource.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates an icosahedron in the form of an itk::Mesh
// Date: 2010/08/18

#ifndef __ivanIcosahedronMeshSource_hxx
#define __ivanIcosahedronMeshSource_hxx

#include "ivanIcosahedronMeshSource.h"

namespace ivan
{
  
static const double c = 0.5;
static const double d = 0.30901699;

static double IcosaPoints[] = {
   0.0,d,-c, 0.0,d,c,  0.0,-d,c, -d,c,0.0, 
  -d,-c,0.0, d,c,0.0,  d,-c,0.0,  0.0,-d,-c,
   c,0.0,d, -c,0.0,d, -c,0.0,-d,  c,0.0,-d
};

static unsigned long IcosaVerts[] = {
  0,5,3, 1,3,5, 1,2,9, 1,8,2, 0,7,11, 0,10,7, 2,6,4, 7,4,6, 3,9,10,
  4,10,9, 5,11,8, 6,8,11, 1,9,3, 1,5,8, 0,3,10, 0,11,5, 7,10,4, 7,6,11,
  2,4,9, 2,8,6
};

static const double IcosaScale = 1.0 / 0.58778524999243;

/**
 *
 */
template<class TPixel>
IcosahedronMeshSource<TPixel>
::IcosahedronMeshSource()
{

}

/*
 *
 */
template<class TPixel>
void
IcosahedronMeshSource<TPixel>
::GenerateData()
{
  //this->GenerateMesh< TriangleCellType > ( IcosaScale, IcosaPoints, IcosaVerts );
  // GCC requires this strange syntax to call a function template
  this->template GenerateMesh< TriangleCellType > ( IcosaScale, IcosaPoints, IcosaVerts );
}


template<class TPixel>
void
IcosahedronMeshSource<TPixel>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );


}

} // end namespace ivan

#endif
