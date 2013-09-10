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
// File: ivanCompositeNode.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanCompositeNode_hxx
#define __ivanCompositeNode_hxx


#include "ivanCompositeNode.h"


namespace ivan
{  

CompositeNode::CompositeNode()
{
  
}


CompositeNode::~CompositeNode()
{

}


void CompositeNode::AddNode( GraphNode *node )
{
  if( !m_Nodes.size() )
  {
    // Simply add all parents and children of this node to the supernode
    
    for( unsigned int i=0; i<node->GetNumberOfParents(); ++i )
      Superclass::AddParent( node->GetParent(i) );
      
    for( unsigned int i=0; i<node->GetNumberOfChildren(); ++i )
      Superclass::AddChild( node->GetChild(i) );
    
    return;     
  }
  
  // In other case, we need to check if any of the parents and children are already part
  // of this node, in order not to include them. 
  // Maybe this could be implemented in a faster way 
  
  for( unsigned int i=0; i<node->GetNumberOfParents(); ++i )
  {
    if( !this->HasNode( node->GetParent(i) ) )
      Superclass::AddParent( node->GetParent(i) );
  }
  
  for( unsigned int i=0; i<node->GetNumberOfChildren(); ++i )
  {
    if( !this->HasNode( node->GetChild(i) ) )
      Superclass::AddChild( node->GetChild(i) );
  }
}


GraphNode * CompositeNode::GetChildById( NodeIdentifier id )
{ 
  GraphNode * child;
  
  for( unsigned int i=0; i<m_Nodes.size(); ++i )
  {
    if( child = m_Nodes[i]->GetChildById( id ) )
      return child;  
  }
  
  return 0;
}


const GraphNode * CompositeNode::GetChildById( NodeIdentifier id ) const
{ 
  GraphNode * child;
  
  for( unsigned int i=0; i<m_Nodes.size(); ++i )
  {
    if( child = m_Nodes[i]->GetChildById( id ) )
      return child;  
  }
  
  return 0;
}


bool CompositeNode::AddChild( NodeIdentifier id, GraphNode *child )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == id )
    {
      (*it)->AddChild( child );
      Superclass::AddChild( child ); // add it to the composite node's list of children too
      return true;
    } 
  }
  
  return false;
}


bool CompositeNode::InsertChild( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *child )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == targetNodeId )
    {
      (*it)->InsertChild( pos, child );
      // !!! WARNING: SHOULD USE InsertChild ALSO HERE, SEARCH CORRECT POSITION IF ANY
      Superclass::AddChild( child ); // add it to the composite node's list of children too
      return true;
    } 
  }
  
  return false;  
}


bool CompositeNode::ReplaceChild( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *child )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == targetNodeId )
    {
      (*it)->ReplaceChild( pos, child );
      Superclass::ReplaceChild( pos, child ); // replace it in the composite node's list of children too
      return true;
    } 
  }
  
  return false;   
}


bool CompositeNode::ReplaceChild( NodeIdentifier targetNodeId, GraphNode *oldChild, GraphNode *newChild )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == targetNodeId )
    {
      (*it)->ReplaceChild( oldChild, newChild );
      Superclass::ReplaceChild( oldChild, newChild ); // replace it in the composite node's list of children too
      return true;
    } 
  }
  
  return false;   
}


bool CompositeNode::RemoveChild( NodeIdentifier targetNodeId, unsigned int pos )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == targetNodeId )
    {
      (*it)->RemoveChild( pos );
      Superclass::RemoveChild( pos ); // remove it in the composite node's list of children too
      return true;
    } 
  }
  
  return false;   
}


bool CompositeNode::RemoveChild( NodeIdentifier targetNodeId, GraphNode *child )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == targetNodeId )
    {
      (*it)->RemoveChild( child );
      Superclass::RemoveChild( child ); // remove it in the composite node's list of children too
      return true;
    } 
  }
  
  return false;   
}


void CompositeNode::RemoveAllChildren()
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
    (*it)->RemoveAllChildren();
  
  Superclass::RemoveAllChildren(); // remove them from our list of children too  
}


GraphNode * CompositeNode::GetParentById( NodeIdentifier id )
{ 
  GraphNode * parent;
  
  for( unsigned int i=0; i<m_Nodes.size(); ++i )
  {
    if( parent = m_Nodes[i]->GetParentById( id ) )
      return parent;  
  }
  
  return 0;
}


const GraphNode * CompositeNode::GetParentById( NodeIdentifier id ) const
{ 
  GraphNode * parent;
  
  for( unsigned int i=0; i<m_Nodes.size(); ++i )
  {
    if( parent = m_Nodes[i]->GetParentById( id ) )
      return parent;  
  }
  
  return 0;
}


bool CompositeNode::AddParent( NodeIdentifier id, GraphNode *parent )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == id )
    {
      (*it)->AddParent( parent );
      Superclass::AddParent( parent ); // add it to the composite node's list of parents too
      return true;
    } 
  }
  
  return false;
}


bool CompositeNode::InsertParent( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *parent )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == targetNodeId )
    {
      (*it)->InsertParent( pos, parent );
      // !!! WARNING: SHOULD USE InsertParent ALSO HERE, SEARCH CORRECT POSITION IF ANY
      Superclass::AddParent( parent ); // add it to the composite node's list of parents too
      return true;
    } 
  }
  
  return false;  
}


bool CompositeNode::ReplaceParent( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *parent )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == targetNodeId )
    {
      (*it)->ReplaceParent( pos, parent );
      Superclass::ReplaceParent( pos, parent ); // replace it in the composite node's list of parent too      
      return true;
    } 
  }
  
  return false;   
}


bool CompositeNode::ReplaceParent( NodeIdentifier targetNodeId, GraphNode *oldParent, GraphNode *newParent )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == targetNodeId )
    {
      (*it)->ReplaceParent( oldParent, newParent );
      Superclass::ReplaceParent( oldParent, newParent ); // replace it in the composite node's list of parent too      
      return true;
    } 
  }
  
  return false; 
}


bool CompositeNode::RemoveParent( NodeIdentifier targetNodeId, unsigned int pos )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == targetNodeId )
    {
      (*it)->RemoveParent( pos );
      Superclass::RemoveParent( pos ); // replace it in the composite node's list of parent too
      return true;
    } 
  }
  
  return false;   
}


bool CompositeNode::RemoveParent( NodeIdentifier targetNodeId, GraphNode *parent )
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
  {
    if( (*it)->GetNodeId() == targetNodeId )
    {
      (*it)->RemoveParent( parent );
      Superclass::RemoveParent( parent ); // replace it in the composite node's list of parent too
      return true;
    } 
  }
  
  return false;   
}


void CompositeNode::RemoveAllParents()
{
  for( NodeConstIterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it )
    (*it)->RemoveAllParents();
  
  Superclass::RemoveAllParents(); // remove them from our list of children too  
}


void CompositeNode::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  
}

} // end namespace ivan

#endif // __ivanCompositeNode_hxx
