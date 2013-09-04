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
// File: ivanCubeMeshSource.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a cube in the form of an itk::Mesh
// Date: 2010/08/18

#ifndef __ivanCubeMeshSource_hxx
#define __ivanCubeMeshSource_hxx

#include "itkCubeMeshSource.h"

namespace ivan
{

static double CubePoints[] = {
  -1.0,-1.0,-1.0, 1.0,-1.0,-1.0, 1.0,1.0,-1.0, -1.0,1.0,-1.0,
  -1.0,-1.0,1.0, 1.0,-1.0,1.0, 1.0,1.0,1.0, -1.0,1.0,1.0
};

static unsigned long CubeVerts[] = {
  0,1,5,4, 0,4,7,3, 4,5,6,7, 3,7,6,2, 1,2,6,5, 0,3,2,1
};

static const double CubeScale = 1.0 / vcl_sqrt(3.0);

/**
 *
 */
template<class TPixel>
CubeMeshSource<TPixel>
::CubeMeshSource()
{

}

/*
 *
 */
template<class TPixel>
void
CubeMeshSource<TPixel>
::GenerateData()
{
  this->GenerateMesh<QuadrilateralCellType>( CubeScale, CubePoints, CubeVerts );
}


template<class TPixel>
void
CubeMeshSource<TPixel>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );


}

} // end namespace ivan

#endif
