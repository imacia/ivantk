// File: ivanGraphCountNodeVisitor2.txx
// Author: Iván Macía (imacia@vicomtech.org)
// Description:


#ifndef __ivanGraphCountNodeVisitor2_txx
#define __ivanGraphCountNodeVisitor2_txx

#include "ivanGraphCountNodeVisitor.h"


namespace ivan
{
  
template <class TNode>
GraphCountNodeVisitor2<TNode>::GraphCountNodeVisitor2() :
  m_Count(0)
{

}


template <class TNode>
GraphCountNodeVisitor2<TNode>::~GraphCountNodeVisitor2()
{

}


template <class TNode>
void GraphCountNodeVisitor2<TNode>::Visit( GraphNode * node )
{
  if( TNode* tnode = dynamic_cast<TNode*>( node ) )
    this->Apply( tnode );
    
  this->Traverse( node ); // continue with traversal
}


template <class TNode>
void GraphCountNodeVisitor2<TNode>::Apply( NodeType * node )
{
  ++m_Count;
}


template <class TNode>
void GraphCountNodeVisitor2<TNode>::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Count: " << m_Count << std::endl;
}

} // end namespace ivan

#endif
