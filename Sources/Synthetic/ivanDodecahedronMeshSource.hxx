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
// File: ivanDodecahedronMeshSource.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates an Dodecahedron in the form of an itk::Mesh
// Date: 2010/08/18

#ifndef __ivanDodecahedronMeshSource_hxx
#define __ivanDodecahedronMeshSource_hxx

#include "itkDodecahedronMeshSource.h"

namespace ivan
{
  
static double a_0 = 0.61803398875;
static double b = 0.381966011250;

static double DodePoints[] = {
   b, 0, 1,  -b, 0, 1,  b, 0,-1, -b, 0,-1,  0, 1,-b,
   0, 1, b,   0,-1,-b,  0,-1, b,  1, b, 0,  1,-b, 0,
  -1, b, 0,  -1,-b, 0, -a_0, a_0, a_0,  a_0,-a_0, a_0, -a_0,-a_0,-a_0,
   a_0, a_0,-a_0,   a_0, a_0, a_0, -a_0, a_0,-a_0, -a_0,-a_0, a_0,  a_0,-a_0,-a_0
};

static unsigned long DodeVerts[] = {
  0,16,5,12,1, 1,18,7,13,0, 2,19,6,14,3, 3,17,4,15,2, 4,5,16,8,15,
  5,4,17,10,12, 6,7,18,11,14, 7,6,19,9,13, 8,16,0,13,9, 9,19,2,15,8,
  10,17,3,14,11, 11,18,1,12,10
};

static const double DodeScale = 1.0 / 1.070466269319;

/**
 *
 */
template<class TPixel>
DodecahedronMeshSource<TPixel>
::DodecahedronMeshSource()
{

}

/*
 *
 */
template<class TPixel>
void
DodecahedronMeshSource<TPixel>
::GenerateData()
{
  this->GenerateMesh<PolygonCellType>( DodeScale, DodePoints, DodeVerts );
}


template<class TPixel>
void
DodecahedronMeshSource<TPixel>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );


}

} // end namespace ivan

#endif
