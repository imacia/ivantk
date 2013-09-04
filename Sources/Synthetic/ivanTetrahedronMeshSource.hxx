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
// File: ivanTetrahedronMeshSource.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates an Tetrahedron in the form of an itk::Mesh
// Date: 2010/08/18

#ifndef __ivanTetrahedronMeshSource_hxx
#define __ivanTetrahedronMeshSource_hxx

#include "itkTetrahedronMeshSource.h"

namespace ivan
{
  
static double TetraPoints[] = {
  1.0,1.0,1.0, -1.0,1.0,-1.0, 1.0,-1.0,-1.0, -1.0,-1.0,1.0
};
static unsigned long TetraVerts[] = {
  0,1,2, 1,3,2, 0,2,3, 0,3,1
};

static const double TetraScale = 1.0 / vcl_sqrt(3.0);

/**
 *
 */
template<class TPixel>
TetrahedronMeshSource<TPixel>
::TetrahedronMeshSource()
{

}

/*
 *
 */
template<class TPixel>
void
TetrahedronMeshSource<TPixel>
::GenerateData()
{
  this->GenerateMesh<TriangleCellType>( TetraScale, TetraPoints, TetraVerts );
}


template<class TPixel>
void
TetrahedronMeshSource<TPixel>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );


}

} // end namespace ivan

#endif
