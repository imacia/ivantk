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
// File: ivanRegularSphereMeshSource2.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a regular sphere in the form of an itk::Mesh by decimating an icosahedron 
// Date: 2010/08/20

#ifndef __ivanRegularSphereMeshSource2_hxx
#define __ivanRegularSphereMeshSource2_hxx

#include "ivanRegularSphereMeshSource2.h"
#include "ivanIcosahedronMeshSource.h"

#include <vnl/vnl_c_vector.h>

#include <list>
#include <algorithm>


namespace ivan
{

/**
 *
 */
template<class TPixel>
RegularSphereMeshSource2<TPixel>
::RegularSphereMeshSource2() :
  m_Radius( 1.0 ),
  m_Decimation( 1 )
{
  /**
   * Create the output
   */
  typename OutputMeshType::Pointer output = OutputMeshType::New();
  this->ProcessObject::SetNumberOfRequiredOutputs(1);
  this->ProcessObject::SetNthOutput(0, output.GetPointer());

  m_Center.Fill(0);
}


template <class TPixel>
void
RegularSphereMeshSource2<TPixel>
::GenerateData()
{
  typename OutputMeshType::Pointer outputMesh = this->GetOutput();
   
  //outputMesh->GetPoints()->Reserve( this->GetNumberOfVertices() );
  //outputMesh->SetCellsAllocationMethod( OutputMeshType::CellsAllocatedDynamicallyCellByCell );
  // It seems that the CellAutoPointer will take care of deleting the cell itself
  outputMesh->SetCellsAllocationMethod( OutputMeshType::CellsAllocatedAsStaticArray );
    
  typedef ivan::IcosahedronMeshSource<TPixel>      IcosahedronSourceType;
  IcosahedronSourceType::Pointer icosahedronSource = IcosahedronSourceType::New();
  icosahedronSource->SetCenter( this->m_Center );
  icosahedronSource->SetRadius( this->m_Radius );
  icosahedronSource->Update();
  
  typedef typename IcosahedronSourceType::OutputMeshType  IcosahedronMeshType;
  IcosahedronMeshType::Pointer icosahedronMesh = icosahedronSource->GetOutput();
  typename IcosahedronMeshType::PointsContainer *oldPoints = icosahedronMesh->GetPoints();
    
  unsigned long numberOfPoints = oldPoints->Size();
  unsigned long numberOfEdges  = 30;	// Known for an icosahedron
  unsigned long numberOfCells  = icosahedronMesh->GetNumberOfCells();
  
  typename OutputMeshType::Pointer sourceMesh, targetMesh;
    
  for( unsigned int iter = 0; iter < m_Decimation; ++iter )
  {  
    if( !iter )
      sourceMesh = icosahedronMesh;
    else
      sourceMesh = targetMesh;
      
    if( iter == m_Decimation - 1 )
      targetMesh = outputMesh;
    else
    {
      targetMesh = OutputMeshType::New();
      targetMesh->SetCellsAllocationMethod( OutputMeshType::CellsAllocatedAsStaticArray );
    }    
    
    // Copy old points
    
    oldPoints = sourceMesh->GetPoints();     
    PointsContainer::Pointer newPoints = PointsContainer::New();
    newPoints->Reserve( oldPoints->Size() );
    
    targetMesh->SetPoints( newPoints.GetPointer() );
    
    typename OutputMeshType::PointsContainerIterator  pointIterator = oldPoints->Begin();
    unsigned int idx = 0, nextIdx = 0;
    
    while( pointIterator !=  oldPoints->End() )
    {
      newPoints->SetElement( idx, pointIterator.Value() );
      
      ++pointIterator;
      ++idx;
    }    
     
    // Create a list of edges. Each edge will hold a new point in the middle. The identifier
    // of the edge will coincide with the identifier of the new point. From a triangle we will
    // obtain 4 new triangle cells from the points in the middle. The distance of this point
    // to the center will be adjusted by normalization, so it does not lie exactly in the middle
    // but in the unit sphere
      
    typedef std::list<EdgeStruct>         EdgeListType;
    typedef EdgeListType::const_iterator  EdgeListConstIterator;
     
    EdgeListType    edgeList;
    unsigned long   edgeId = numberOfPoints;
  	OutputPointType mid;
  	OutputPointType p1, p2;
  	double          norm;
  
  	// Traverse the cells and record edges while inserting midpoints if the edge 
  	// does not exist in the edge list
  	
  	typedef OutputMeshType::CellsContainer::ConstIterator  OutputCellIterator;
  	typedef OutputMeshType::CellType                       OutputCellInterfaceType;
  	  
  	OutputCellIterator cellIterator = sourceMesh->GetCells()->Begin();
        
    while( cellIterator != sourceMesh->GetCells()->End() ) 
    {
      OutputCellInterfaceType * cell = cellIterator.Value();
      
      if( cell->GetType() != OutputCellInterfaceType::TRIANGLE_CELL )
        continue;
      
      TriangleCellType * triangleCell = static_cast<TriangleCellType *>( cell );
              
      for( idx=0; idx<triangleCell->GetNumberOfPoints(); ++idx )
      {
        nextIdx = (idx+1) % triangleCell->GetNumberOfPoints();
        
        EdgeStruct currentEdge( 0,  *( triangleCell->PointIdsBegin() + idx ), 
          *( triangleCell->PointIdsBegin() + nextIdx ) );
              
        if( std::find( edgeList.begin(), edgeList.end(), currentEdge ) == edgeList.end() )
        {
          sourceMesh->GetPoint( *( triangleCell->PointIdsBegin() + idx ), &p1 );
  				sourceMesh->GetPoint( *( triangleCell->PointIdsBegin() + nextIdx ), &p2 );
  				mid[0] = ( p1[0] + p2[0]) / 2.0;
  				mid[1] = ( p1[1] + p2[1]) / 2.0;
  				mid[2] = ( p1[2] + p2[2]) / 2.0;
  
  				// Normalize the this midpoint vector to the unit sphere
  				// by dividing by the norm
  				if( ( norm = vnl_c_vector<float>::two_norm( mid.GetDataPointer(), 3 ) ) == 0 )
  					norm = 1.0;
  				
  				mid[0] /= norm;
  				mid[1] /= norm;
  				mid[2] /= norm;

          mid[0] *= m_Radius;
          mid[1] *= m_Radius;
          mid[2] *= m_Radius;
  				
  				newPoints->InsertElement( edgeId, mid );
  				
  				// The edge is not in the table, so insert it!
  				currentEdge.edgeId = edgeId;
  				edgeList.push_back( currentEdge );
  				edgeList.sort();
  				
  				++edgeId;   
        }      
      }    
           
      ++cellIterator;
    }
    
    
    // Now, create the new cells using the new points. The new surface will subdivide each triangular
    // cell into 4 new triangular cells
  	
  	
  	PointIdentifier        vertex[3], finalVertex[3];
  	EdgeListConstIterator  edgeListIt[3];
  	unsigned long          cellId = 0;
  	
  	cellIterator = sourceMesh->GetCells()->Begin();
      
    while( cellIterator != sourceMesh->GetCells()->End() ) 
  	{
  		OutputCellInterfaceType * cell = cellIterator.Value();
      
      if( cell->GetType() != OutputCellInterfaceType::TRIANGLE_CELL )
        continue;
      
      TriangleCellType * triangleCell = static_cast<TriangleCellType *>( cell );
  		
  		// First retrieve edges and corresponding point ids
  		 
  		vertex[0] = *( triangleCell->PointIdsBegin() );
  		vertex[1] = *( triangleCell->PointIdsBegin() + 1 );
  		vertex[2] = *( triangleCell->PointIdsBegin() + 2 );

      EdgeStruct currentEdge;
  		
  	  currentEdge.first  = vertex[0];
  	  currentEdge.second = vertex[1];
      edgeListIt[0] = std::find( edgeList.begin(), edgeList.end(), currentEdge );
  		
  		if( edgeListIt[0] == edgeList.end() )
  		  itkExceptionMacro( "Edge not found." ); // should not reach this point
  		  
  		currentEdge.first  = vertex[1];
  	  currentEdge.second = vertex[2];
  		edgeListIt[1] = std::find( edgeList.begin(), edgeList.end(), currentEdge );
  		
  		if( edgeListIt[1] == edgeList.end() )
  		  itkExceptionMacro( "Edge not found." ); // should not reach this point
  		  
  		currentEdge.first  = vertex[2];
  	  currentEdge.second = vertex[0];
  		edgeListIt[2] = std::find( edgeList.begin(), edgeList.end(), currentEdge );
  		
  		if( edgeListIt[2] == edgeList.end() )
  		  itkExceptionMacro( "Edge not found." ); // should not reach this point
  		
  		// Now create the new cells
  		
  		CellAutoPointer newCells[4];
  		
  		finalVertex[0] = vertex[0];
  		finalVertex[1] = edgeListIt[0]->edgeId;
  		finalVertex[2] = edgeListIt[2]->edgeId;
  		
  		newCells[0].TakeOwnership( new TriangleCellType );
  		newCells[0]->SetPointIds( finalVertex, finalVertex + 3 );
      
      targetMesh->SetCell( cellId++, newCells[0] );
      
      
      finalVertex[0] = edgeListIt[0]->edgeId;
  		finalVertex[1] = vertex[1];
  		finalVertex[2] = edgeListIt[1]->edgeId;
  		
  		newCells[1].TakeOwnership( new TriangleCellType );
  		newCells[1]->SetPointIds( finalVertex, finalVertex + 3 );
      
      targetMesh->SetCell( cellId++, newCells[1] );
      
      
      finalVertex[0] = edgeListIt[2]->edgeId;
  		finalVertex[1] = edgeListIt[1]->edgeId;
  		finalVertex[2] = vertex[2];
  		
  		newCells[2].TakeOwnership( new TriangleCellType );
  		newCells[2]->SetPointIds( finalVertex, finalVertex + 3 );
      
      targetMesh->SetCell( cellId++, newCells[2] );
      
      
      finalVertex[0] = edgeListIt[0]->edgeId;
  		finalVertex[1] = edgeListIt[1]->edgeId;
  		finalVertex[2] = edgeListIt[2]->edgeId;
  		
  		newCells[3].TakeOwnership( new TriangleCellType );
  		newCells[3]->SetPointIds( finalVertex, finalVertex + 3 );
      
      targetMesh->SetCell( cellId++, newCells[3] );

      ++cellIterator;
   	}
   	
  	// Update the point, cell, and edge counts
  	numberOfPoints = newPoints->Size();	                 // numberOfPoints = oldNumberOfPoints + numberOfEdges
  	numberOfEdges  = 2*numberOfEdges + 3*numberOfCells;	 // numberOfEdges = 2*oldNumberOfEdges + 3*oldNumberOfCells
  	numberOfCells  = targetMesh->GetNumberOfCells();     // numberOfCells = 4*oldNumberOfCells
  }  
}


template<class TPixel>
void
RegularSphereMeshSource2<TPixel>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );

  os << indent << "Center: " << m_Center << std::endl;
  os << indent << "Radius: " << m_Radius << std::endl;
  os << indent << "Decimation: " << m_Decimation << std::endl;
}

} // end namespace ivan

#endif
