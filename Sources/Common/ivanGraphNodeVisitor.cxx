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
// File: ivanGraphNodeVisitor.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : implements the GOF Visitor pattern for graph nodes

#include "ivanGraphNodeVisitor.h"


namespace ivan
{

GraphNodeVisitor::GraphNodeVisitor() :
  m_TraversalMode( TraverseAllChildren ),
  m_TraversalMask( 0xffffffff )
{

}


GraphNodeVisitor::~GraphNodeVisitor()
{

}


void GraphNodeVisitor::Visit( GraphNode * node )
{
  // Call the appropiate Apply() method via dispatcher

  DispatcherMapType::iterator it = m_DispatcherMap.find( node->GetNameOfClass() );

  if( it != m_DispatcherMap.end() )
    it->second->Apply( this, node );
}



void GraphNodeVisitor::Traverse( GraphNode * node )
{	
  if( m_TraversalMode == TraverseAllParents )
    node->Ascend( this );
  else if( m_TraversalMode != TraverseNone )
    node->Traverse( this );
}


void GraphNodeVisitor::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );

  os << indent << "TraversalMode: " << m_TraversalMode << std::endl;
  os << indent << "TraversalMask: " << m_TraversalMask << std::endl;
}

} // end namespace ivan
