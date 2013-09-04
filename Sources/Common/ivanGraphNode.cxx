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
// File: ivanGraphNode.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : generic node of an acyclic graph structure

#include "ivanGraphNode.h"
#include "ivanGraphNodeVisitor.h"


namespace ivan
{


GraphNode::GraphNode() :
	m_Mask( 0xffffffff ),
	m_NodeId(0),
	m_DepthLevel(0)
{

}


GraphNode::~GraphNode()
{

}


GraphNode * GraphNode::GetChildById( NodeIdentifier id )
{ 
  for( unsigned int i=0; i<m_Children.size(); ++i )
  {
    if( m_Children[i]->GetNodeId() == id )
      return m_Children[i];  
  }
  
  return 0;
}


const GraphNode * GraphNode::GetChildById( NodeIdentifier id ) const
{ 
  for( unsigned int i=0; i<m_Children.size(); ++i )
  {
    if( m_Children[i]->GetNodeId() == id )
      return m_Children[i];  
  }
  
  return 0;
}


unsigned int GraphNode::GetChildPosition( const GraphNode * node ) const
{
  for( unsigned int i=0; i<m_Children.size(); ++i )
  {
    if( m_Children[i].GetPointer() == node )
      return i;    
  }
  
  return this->GetNumberOfChildren();
}


void GraphNode::AddChild( GraphNode *child )
{
	if( !child )
	{
		itkWarningMacro( "Trying to insert NULL node." );
		return;
	}
	
	// Avoid recursive addition
	for( unsigned int i=0; i < m_Children.size(); ++i )
	{
	  if( m_Children[i].GetPointer() == child )
	    return;
	}
	
	m_Children.push_back( child );
	
	// Register as parent of child.
  child->AddParent( this );
  child->SetDepthLevel( this->GetDepthLevel() + 1 );
}


bool GraphNode::AddChild( NodeIdentifier id, GraphNode *child )
{
  if( m_NodeId == id )
  {
    this->AddChild( child );
    return true;
  }
  else return false;
}


void GraphNode::InsertChild( unsigned int pos, GraphNode *child )
{
	if( !child )
	{
		itkWarningMacro( "Trying to insert NULL node." );
		return;
	}
	
	if( pos >= m_Children.size() )
	{
	  itkWarningMacro( "Inserting child at position past the end of the container." );
		m_Children.push_back( child );
	}
  else
    m_Children.insert( m_Children.begin() + pos, child );
 
  // Register as parent of child.
  child->AddParent( this );
  child->SetDepthLevel( this->GetDepthLevel() + 1 );
}


bool GraphNode::InsertChild( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *child )
{
  if( this->GetNodeId() == targetNodeId )
  {
    this->InsertChild( pos, child );
    return true;
  }
  else return false;
}


void GraphNode::ReplaceChild( unsigned int pos, GraphNode *child )
{
	if( !child )
	{
		itkWarningMacro( "Trying to insert NULL node." );
		return;
	}
	
	if( pos >= m_Children.size() )
	{
	  itkWarningMacro( "Inserting child at position past the end of the container." );
		m_Children.push_back( child );
	} 
  else
  {
    m_Children[pos]->RemoveParent( this );
    m_Children[pos] = child;
  }
 
  // Register as parent of child.
  child->AddParent( this );	
}


bool GraphNode::ReplaceChild( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *child )
{
  if( this->GetNodeId() == targetNodeId )
  {
    this->ReplaceChild( pos, child );
    return true;
  }
  else return false;  
}


bool GraphNode::ReplaceChild( GraphNode *oldChild, GraphNode *newChild )
{
	NodeContainer::iterator it = m_Children.begin();
	
	while( it != m_Children.end() )
	{
		if( it->GetPointer() == oldChild )
		{
		  (*it)->RemoveParent( this );
			*it = newChild;
			newChild->SetDepthLevel( this->GetDepthLevel() + 1 );
			(*it)->AddParent( this );
			return true;
		}
			
		++it;
	}
  
  return false;
}


bool GraphNode::RemoveChild( GraphNode *child )
{
	NodeContainer::iterator it = m_Children.begin();
	
	while( it != m_Children.end() )
	{
		if( it->GetPointer() == child )
		{
			m_Children.erase( it );
			return true;
		}
			
		++it;
	}
	
	return false;
}


void GraphNode::RemoveChild( unsigned int pos )
{
	if( pos >= m_Children.size() )
	{
		itkWarningMacro( "Trying to remove child with pos " << pos << " past the end of the children list." )
		return;
	}
	
	m_Children.erase( m_Children.begin() + pos );
}


bool GraphNode::RemoveChild( NodeIdentifier targetNodeId, unsigned int pos )
{
  if( this->GetNodeId() == targetNodeId )
  {
    this->RemoveChild( pos );
    return true;
  }
  else return false;  
}


bool GraphNode::RemoveChild( NodeIdentifier targetNodeId, GraphNode *child )
{
  if( this->GetNodeId() == targetNodeId )
  {
    this->RemoveChild( child );
    return true;
  }
  else return false;  
}



void GraphNode::AddParent( GraphNode *parent )
{
	if( !parent )
	{
		itkWarningMacro( "Trying to insert NULL node." );
		return;
	}
	
	// Avoid recursive addition
	for( unsigned int i=0; i < m_Parents.size(); ++i )
	{
	  if( m_Parents[i].GetPointer() == parent )
	    return;
	}
	
	m_Parents.push_back( parent );
	
	// Register as child of parent.
  parent->AddChild( this );
}


GraphNode * GraphNode::GetParentById( NodeIdentifier id )
{ 
  for( unsigned int i=0; i<m_Parents.size(); ++i )
  {
    if( m_Parents[i]->GetNodeId() == id )
      return m_Parents[i];  
  }
  
  return 0;
}


const GraphNode * GraphNode::GetParentById( NodeIdentifier id ) const
{ 
  for( unsigned int i=0; i<m_Parents.size(); ++i )
  {
    if( m_Parents[i]->GetNodeId() == id )
      return m_Parents[i];  
  }
  
  return 0;
}


bool GraphNode::AddParent( NodeIdentifier id, GraphNode *parent )
{
  if( m_NodeId == id )
  {
    this->AddParent( parent );
    return true;
  }
  else return false;  
}


void GraphNode::ReplaceParent( unsigned int pos, GraphNode *parent )
{
	if( !parent )
	{
		itkWarningMacro( "Trying to insert NULL node." );
		return;
	}
	
	if( pos >= m_Parents.size() )
	{
	  itkWarningMacro( "Inserting parent at position past the end of the container." );
		m_Parents.push_back( parent );
	} 
  else
  {
    m_Parents[pos]->RemoveChild( this );
    m_Parents[pos] = parent;
  }
 
  // Register as child of parent.
  parent->AddChild( this );	
}


void GraphNode::InsertParent( unsigned int pos, GraphNode *parent )
{
	if( !parent )
	{
		itkWarningMacro( "Trying to insert NULL node." );
		return;
	}
	
	if( pos >= m_Parents.size() )
	{
	  itkWarningMacro( "Inserting child at position past the end of the container." );
		m_Parents.push_back( parent );
	}
  else
    m_Parents.insert( m_Parents.begin() + pos, parent );
 
  // Register as child of parent.
  parent->AddChild( this );
}


bool GraphNode::InsertParent( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *parent )
{
  if( this->GetNodeId() == targetNodeId )
  {
    this->InsertParent( pos, parent );
    return true;
  }
  else return false;  
}


bool GraphNode::ReplaceParent( GraphNode *oldParent, GraphNode *newParent  )
{
	ParentNodeContainer::iterator it = m_Parents.begin();
	
	while( it != m_Parents.end() )
	{
		if( it->GetPointer() == oldParent )
		{
		  (*it)->RemoveChild( this );
			*it = newParent;
			(*it)->AddChild( this );
			return true;
		}
			
		++it;
	}
	
	return false;
}


bool GraphNode::ReplaceParent( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *parent )
{
  if( this->GetNodeId() == targetNodeId )
  {
    this->ReplaceParent( pos, parent );
    return true;
  }
  else return false;  
}


bool GraphNode::RemoveParent( GraphNode *parent )
{
	ParentNodeContainer::iterator it = m_Parents.begin();
	
	while( it != m_Parents.end() )
	{
		if( it->GetPointer() == parent )
		{
			m_Parents.erase( it );
			return true;
		}
			
		++it;
	}
	
	return false;	
}


void GraphNode::RemoveParent( unsigned int pos )
{
	if( pos >= m_Parents.size() )
	{
		itkWarningMacro( "Trying to remove child with " << pos << " past the end of the children list." )
		return;
	}
	
	m_Parents.erase( m_Parents.begin() + pos );
}


bool GraphNode::RemoveParent( NodeIdentifier targetNodeId, unsigned int pos )
{
  if( this->GetNodeId() == targetNodeId )
  {
    this->RemoveParent( pos );
    return true;
  }
  else return false;  
}


bool GraphNode::RemoveParent( NodeIdentifier targetNodeId, GraphNode *parent )
{
  if( this->GetNodeId() == targetNodeId )
  {
    this->RemoveParent( parent );
    return true;
  }
  else return false;  
}


void GraphNode::Accept( GraphNodeVisitor * visitor )
{
	if ( visitor->IsValidMask( this ) ) 
  {
    visitor->PushOntoNodePath( this );
    visitor->Visit( this );
    visitor->PopFromNodePath();
  }
}
		

void GraphNode::Traverse( GraphNodeVisitor * visitor )
{
	for( NodeContainer::iterator it = m_Children.begin();  it!= m_Children.end(); ++it )
    (*it)->Accept( visitor );  
}


void GraphNode::Ascend( GraphNodeVisitor * visitor )
{
	for( ParentNodeContainer::iterator it = m_Parents.begin();  it!= m_Parents.end(); ++it )
    (*it)->Accept( visitor );  
}



void GraphNode::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Mask: " << m_Mask << std::endl;
  os << indent << "NodeId: " << m_NodeId << std::endl;
  os << indent << "DepthLevel: " << m_DepthLevel << std::endl;
  os << indent << "Name: " << m_Name << std::endl;
	os << indent << "Parents: " << std::endl;
  
  for( unsigned int i=0; i < m_Parents.size(); ++i )
	{
		os << "[" << i << "] : " << m_Parents[i].GetPointer() << std::endl;
		m_Parents[i]->Print( os, indent.GetNextIndent() );
	}
  
  os << indent << "Children: " << std::endl;

	for( unsigned int i=0; i < m_Children.size(); ++i )
	{
		os << "[" << i << "] : " << m_Children[i].GetPointer() << std::endl;
		m_Children[i]->Print( os, indent.GetNextIndent() );
	}

}

} // end namespace ivan

