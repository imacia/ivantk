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
// File: ivanCompositeNode.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanCompositeNode_h
#define __ivanCompositeNode_h

#include "ivanGraphNode.h"


namespace ivan
{


/** \class CompositeNode
 *  \brief Represents a group of nodes by using a single composite node or supernode.
 *
 * This class represents a supernode, consisting of a set of nodes that are grouped 
 * into a single node using a Composite Pattern.
 *
 * \ingroup 
 */
 
class ITK_EXPORT CompositeNode : public GraphNode
{

public:

  /** Standard class typedefs. */
  typedef CompositeNode             Self;
  typedef GraphNode                 Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;

  typedef Superclass::Pointer       NodePointer;
  typedef Superclass::ConstPointer  NodeConstPointer;
  
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkNodeTypeMacro( CompositeNode, GraphNode );
  
  /** Provides direct access to the list of nodes. Use with care!!! */
  NodeContainer & GetNodes()
    { return m_Nodes; }
  const NodeContainer & GetNodes() const
    { return m_Nodes; }
    
  /** Provides direct access to the nodes by index. Use with care!!! */
  GraphNode * GetNode( unsigned int pos )
    { return m_Nodes[pos]; }
  const GraphNode * GetNode( unsigned int pos ) const
    { return m_Nodes[pos]; }
    
  /** Add a node to the composite supernode. */
  virtual void AddNode( GraphNode *node );
  
  /** Check if the node is contained in this supernode. */
  inline bool HasNode( GraphNode *node )
    {
      for( unsigned int i=0; i<m_Nodes.size(); ++i )
      {
        if( m_Nodes[i].GetPointer() == node )
          return true;
      }
      return false;
    }        
    
  /** Get child at the given position by id. Returns NULL if not found. */
  virtual GraphNode * GetChildById( NodeIdentifier id );
  virtual const GraphNode * GetChildById( NodeIdentifier id ) const;
    
  /** Add a child to the given node identifier. This method will add the
    * node as child of the specific node if and only if the node exists. 
    * The function returns true if the operation suceeds. */
	virtual bool AddChild( NodeIdentifier id, GraphNode *child );
	
	/** Insert a child to the given node at the given position. This method will
	  * insert the child node of the target node at the given position if and only
	  * if the node exists. The function returns true if the operation suceeds. */
	virtual bool InsertChild( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *child );
	
	/** Replace a child of the given node at the given position. This method will
	  * replace the child node of the target node at the given position if and only
	  * if the node exists. The function returns true if the operation suceeds. */
	virtual bool ReplaceChild( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *child );
	virtual bool ReplaceChild( NodeIdentifier targetNodeId, GraphNode *oldChild, GraphNode *newChild );
	
	/** Remove a child from the given node at the given position. This method will
	  * remove the child node of the target node at the given position if and only
	  * if the node exists. The function returns true if the operation suceeds. */
	virtual bool RemoveChild( NodeIdentifier targetNodeId, unsigned int pos );
	virtual bool RemoveChild( NodeIdentifier targetNodeId, GraphNode *child );
	
	/** Remove all children of this node. */
	inline virtual void RemoveAllChildren();
			
	/** Get the number of children. */
	inline unsigned int GetNumberOfChildren() const 
		{	return m_Children.size();	}
	
	/** Get parent at the given position by id. Returns NULL if not found. */
  virtual GraphNode * GetParentById( NodeIdentifier id );
  virtual const GraphNode * GetParentById( NodeIdentifier id ) const;
		
	/** Add a parent to the given node identifier. This method will add the
    * node parent node of the specific node if and only if the node exists. 
    * The function returns true if the operation suceeds. */
	virtual bool AddParent( NodeIdentifier id, GraphNode *parent );
	
	/** Insert a parent to the given node at the given position. This method will
	  * insert the parent node of the target node at the given position if and only
	  * if the node exists. The function returns true if the operation suceeds. */
	virtual bool InsertParent( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *parent );
	
	/** Replace a parent of the given node at the given position. This method will
	  * replace the parent node of the target node at the given position if and only
	  * if the node exists. The function returns true if the operation suceeds. */
	virtual bool ReplaceParent( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *parent );
	virtual bool ReplaceParent( NodeIdentifier targetNodeId, GraphNode *oldParent, GraphNode *newParent );
	
	/** Remove a parent from the given node at the given position. This method will
	  * remove the parent node of the target node at the given position if and only
	  * if the node exists. The function returns true if the operation suceeds. */
	virtual bool RemoveParent( NodeIdentifier targetNodeId, unsigned int pos );
	virtual bool RemoveParent( NodeIdentifier targetNodeId, GraphNode *parent );
	
	/** Remove all parents of this node. */
	virtual void RemoveAllParents();
	          
protected:

  virtual void AddChild( GraphNode *parent )
    { itkWarningMacro( "Calling incorrect AddChild() for CompositeNode. Use alternative AddChild() instead." ); }
    
  virtual void InsertChild( unsigned int pos, GraphNode *child )
    { itkWarningMacro( "Calling incorrect InsertChild() for CompositeNode. Use alternative InsertChild() instead." ); }
	
	virtual void ReplaceChild( unsigned int pos, GraphNode *child )
	  { itkWarningMacro( "Calling incorrect ReplaceChild() for CompositeNode. Use alternative ReplaceChild() instead." ); }
  
  virtual bool ReplaceChild( GraphNode *oldChild, GraphNode *newChild )
	  { 
      itkWarningMacro( "Calling incorrect ReplaceChild() for CompositeNode. Use alternative ReplaceChild() instead." );
      return false;
    }
  
  virtual void RemoveChild( unsigned int pos )
	  { itkWarningMacro( "Calling incorrect ReplaceChild() for CompositeNode. Use alternative ReplaceChild() instead." ); }

  virtual bool RemoveChild( GraphNode *child )
	  { 
      itkWarningMacro( "Calling incorrect ReplaceChild() for CompositeNode. Use alternative ReplaceChild() instead." );
      return false;
    }
	  
	virtual void AddParent( GraphNode *parent )
    { itkWarningMacro( "Calling incorrect AddParent() for CompositeNode. Use alternative AddParent() instead." ); }
    
  virtual void InsertParent( unsigned int pos, GraphNode *parent )
    { itkWarningMacro( "Calling incorrect InsertParent() for CompositeNode. Use alternative InsertParent() instead." ); }
	
	virtual void ReplaceParent( unsigned int pos, GraphNode *parent )
	  { itkWarningMacro( "Calling incorrect ReplaceParent() for CompositeNode. Use alternative ReplaceParent() instead." ); }
  
  virtual bool ReplaceParent( GraphNode *oldParent, GraphNode *newParent )
	  { 
      itkWarningMacro( "Calling incorrect ReplaceParent() for CompositeNode. Use alternative ReplaceParent() instead." );
      return false;
    }
  
  virtual void RemoveParent( unsigned int pos )
	  { itkWarningMacro( "Calling incorrect RemoveParent() for CompositeNode. Use alternative RemoveParent() instead." ); }
  
  virtual bool RemoveParent( GraphNode *parent )
	  { 
      itkWarningMacro( "Calling incorrect RemoveParent() for CompositeNode. Use alternative RemoveParent() instead." );
      return false;
    }
    

  CompositeNode();
  ~CompositeNode();
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  CompositeNode(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  NodeContainer    m_Nodes;
};

} // end namespace ivan

#endif
