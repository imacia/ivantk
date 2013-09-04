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
// File: ivanOctahedronMeshSource.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates an Octahedron in the form of an itk::Mesh
// Date: 2010/08/18

#ifndef __ivanOctahedronMeshSource_hxx
#define __ivanOctahedronMeshSource_hxx

#include "itkOctahedronMeshSource.h"

namespace ivan
{
  
static double OctaPoints[] = {
  -1.0,-1.0,0.0, 1.0,-1.0,0.0, 1.0,1.0,0.0, -1.0,1.0,0.0,
  0.0,0.0,-1.4142135623731, 0.0,0.0,1.4142135623731
};
static unsigned long OctaVerts[] = {
  4,1,0, 4,2,1, 4,3,2, 4,0,3, 0,1,5, 1,2,5, 2,3,5, 3,0,5
};

static const double OctaScale = 1.0 / vcl_sqrt(2.0);

/**
 *
 */
template<class TPixel>
OctahedronMeshSource<TPixel>
::OctahedronMeshSource()
{

}

/*
 *
 */
template<class TPixel>
void
OctahedronMeshSource<TPixel>
::GenerateData()
{
  this->GenerateMesh<TriangleCellType>( OctaScale, OctaPoints, OctaVerts );
}


template<class TPixel>
void
OctahedronMeshSource<TPixel>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );


}

} // end namespace ivan

#endif
