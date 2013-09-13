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
// File: ivanPlatonicSolidMeshSource.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a platonic solid in the form of an itk::Mesh
// Date: 2010/08/18

#ifndef __ivanPlatonicSolidMeshSource_hxx
#define __ivanPlatonicSolidMeshSource_hxx

#include "ivanPlatonicSolidMeshSource.h"

namespace ivan
{

/**
 *
 */
template<class TPixel>
PlatonicSolidMeshSource<TPixel>
::PlatonicSolidMeshSource() :
  m_Radius( 1.0 )
{
  /**
   * Create the output
   */
  typename OutputMeshType::Pointer output = OutputMeshType::New();
  this->itk::ProcessObject::SetNumberOfRequiredOutputs(1);
  this->itk::ProcessObject::SetNthOutput(0, output.GetPointer());

  m_Center.Fill(0);
}


template <class TPixel>
template <class TCell>
void
PlatonicSolidMeshSource<TPixel>
::GenerateMesh( double solidScale, const double *pointData, const unsigned long *vertexData )
{
  typename OutputMeshType::Pointer outputMesh = this->GetOutput();
   
  outputMesh->GetPoints()->Reserve( this->GetNumberOfVertices() );
  //outputMesh->SetCellsAllocationMethod( OutputMeshType::CellsAllocatedDynamicallyCellByCell );
  // It seems that the CellAutoPointer will take care of deleting the cell itself
  outputMesh->SetCellsAllocationMethod( OutputMeshType::CellsAllocatedAsStaticArray );
    
  // Point data
  
  typename OutputMeshType::PointType point;
  unsigned int i;
  const double *pptr;
    
  for( i=0, pptr = pointData; i<this->GetNumberOfVertices(); ++i, pptr += 3 )
  {
    point[0] = this->m_Radius * solidScale * ( pptr[0] );
    point[1] = this->m_Radius * solidScale * ( pptr[1] );
    point[2] = this->m_Radius * solidScale * ( pptr[2] );
    outputMesh->SetPoint( i, point );
  }
    
  // Cell data
  
  CellAutoPointer newCell;
  const unsigned long *cptr;
  
  for( i=0, cptr = vertexData; i<this->GetNumberOfCells(); ++i, cptr += this->GetCellSize() )
  {
    newCell.TakeOwnership( new TCell );
    newCell->SetPointIds( cptr, cptr + this->GetCellSize() );
    
    outputMesh->SetCell( i, newCell );
  } 
}


template<class TPixel>
void
PlatonicSolidMeshSource<TPixel>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );

  os << indent << "Center: " << m_Center << std::endl;
  os << indent << "Radius: " << m_Radius << std::endl; 
}

} // end namespace ivan

#endif
