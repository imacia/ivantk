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
// File: ivanGraphNode.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : generic node of an acyclic graph structure

#ifndef __ivanGraphNode_h
#define __ivanGraphNode_h

#include "itkObject.h"
#include "itkObjectFactory.h"
#include "itkWeakPointer.h"

#include <vector>
#include <cassert>


// NOTE: We need to declare some static methods for nodes
// I miss this in the itkTypeMacro()

#define itkNodeTypeMacro( classType, superclassType ) \
  itkTypeMacro( classType, superclassType ); \
  static const char * GetNameOfClassStatic() { return #classType; }
  

namespace ivan
{

/** \class GraphNode
 *  \brief Generic node of an acyclic graph structure.
 *
 * This is the base class for nodes of a graph structure. Every node can have
 * any number of children. In the common case that a node has a single parent, 
 * the graph is equivalent to a tree. However, there is support for more than 
 * one parent.
 * 
 * Operations on GraphNode objects can be performed through a GraphNodeVisitor
 * which recursively visits nodes of the graph and perform custom operations
 * via inheritance.
 *
 * \ingroup 
 */
 
class GraphNodeVisitor;
 
class ITK_EXPORT GraphNode : public itk::Object
{

public:

  /** Standard class typedefs. */
  typedef GraphNode       				 Self;
  typedef Object                   Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;
  typedef itk::WeakPointer<Self>        WPointer;
  typedef itk::WeakPointer<const Self>  ConstWPointer;
  
  /** Container of nodes. */ 
  typedef std::vector<Pointer>		       NodeContainer;
  typedef NodeContainer::iterator        NodeIterator;
  typedef NodeContainer::const_iterator  NodeConstIterator;
  
  /** Container of parent nodes. These are stored as weak references. */
  typedef std::vector<WPointer>	               ParentNodeContainer;
  typedef ParentNodeContainer::iterator        ParentNodeIterator;
  typedef ParentNodeContainer::const_iterator  ParentNodeConstIterator;
  
  /** Node path, that is the sorthest path from one node to a descendant. */
  typedef NodeContainer    NodePathType;  
  typedef unsigned int     NodeIdentifier;  
  typedef unsigned int     NodeMaskType;
     
public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkNodeTypeMacro( GraphNode, itk::Object );
  
  itkSetMacro( Mask, NodeMaskType );
  itkGetConstMacro( Mask, NodeMaskType );
  
  itkSetMacro( NodeId, NodeIdentifier );
  itkGetConstMacro( NodeId, NodeIdentifier );
  
  itkSetMacro( DepthLevel, int );
  itkGetConstMacro( DepthLevel, int );
  
  itkSetMacro( Name, std::string & );
  itkGetConstMacro( Name, const std::string & );
    
  /** Get child at the given position. */
  virtual GraphNode * GetChild( unsigned int pos )
    { 
      assert( pos < m_Children.size() );
      return m_Children[pos];
    }
  virtual const GraphNode * GetChild( unsigned int pos ) const
    { 
      assert( pos < m_Children.size() );
      return m_Children[pos];
    }

  /** Get child at the given position by id. Returns NULL if not found. */
  virtual GraphNode * GetChildById( NodeIdentifier id );
  virtual const GraphNode * GetChildById( NodeIdentifier id ) const;
  
  /** Get the position of the provided child in the container (0-based index). If not found, the
    * number of children is returned. */  
  unsigned int GetChildPosition( const GraphNode * node ) const;
 
  /** Add a child at the end of the children list. */
	virtual void AddChild( GraphNode *child );
	
	/** Add a child to the given node identifier. This method is provided mainly to
	  * support Composite nodes with the same interface. This version will add the node 
	  * to the current one only if id coincides with the current node identifier. 
	  * The function returns true if the operation suceeds. */
	virtual bool AddChild( NodeIdentifier id, GraphNode *child );
	
	/** Insert a child before the given position. */
	virtual void InsertChild( unsigned int pos, GraphNode *child );
	
	/** Insert a child to the given node at the given position. This method is provided mainly to
	  * support Composite nodes with the same interface. This version will insert the node 
	  * to the current one only if id coincides with the current node identifier. 
	  * The function returns true if the operation suceeds. */
	virtual bool InsertChild( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *child );
	
	/** Replace child at the given position. */
	virtual void ReplaceChild( unsigned int pos, GraphNode *child );
		
	/** Replace child searching its position. Return true if the node is found and replaced. */
	virtual bool ReplaceChild( GraphNode *oldChild, GraphNode *newChild );

	/** Replace child of given node at the given position. This method is provided mainly to
	  * support Composite nodes with the same interface. This version will replace the node 
	  * to the current one only if id coincides with the current node identifier. 
	  * The function returns true if the operation suceeds. */
	virtual bool ReplaceChild( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *child );
	
	/** Remove a parent from the list. Returns true if the node is found and removed. */
	virtual bool RemoveChild( GraphNode *child );
	
	/** Remove a parent from the list by position. */
	virtual void RemoveChild( unsigned int pos );
	
	/** Remove child of given node at the given position. This method is provided mainly to
	  * support Composite nodes with the same interface. This version will remove the node 
	  * to the current one only if id coincides with the current node identifier. 
	  * The function returns true if the operation suceeds. */
	virtual bool RemoveChild( NodeIdentifier targetNodeId, unsigned int pos );
	virtual bool RemoveChild( NodeIdentifier targetNodeId, GraphNode *child );
	
	/** Remove all children of this node. */
	inline virtual void RemoveAllChildren()
		{	m_Children.clear();	}
			
	/** Get the number of children. */
	inline virtual unsigned int GetNumberOfChildren() const 
		{	return m_Children.size();	}
		
  /** Get children at the given position. */
  virtual GraphNode * GetParent( unsigned int pos )
    { return m_Parents[pos]; }
  virtual const GraphNode * GetParent( unsigned int pos ) const
    { return m_Parents[pos]; }
    
  /** Get parent at the given position by id. Returns NULL if not found. */
  virtual GraphNode * GetParentById( NodeIdentifier id );
  virtual const GraphNode * GetParentById( NodeIdentifier id ) const;
  
  /** Return the first parent. */
  GraphNode * GetParent()
    { return m_Parents[0]; }
  const GraphNode * GetParent() const
    { return m_Parents[0]; }
    
  /** Add a parent at the end of the children list. */
	virtual void AddParent( GraphNode *parent );
	
	/** Add a parent to the given node identifier. This method is provided mainly to
	  * support Composite nodes with the same interface. This version will add the parent 
	  * to the current one only if id coincides with the current node identifier. 
	  * The function returns true if the operation suceeds. */
	virtual bool AddParent( NodeIdentifier id, GraphNode *parent );

	/** Insert a parent before the given position. */
	virtual void InsertParent( unsigned int pos, GraphNode *parent );
	
	/** Insert a parent to the given node at the given position. This method is provided mainly to
	  * support Composite nodes with the same interface. This version will insert the parent node 
	  * to the current one only if id coincides with the current node identifier. 
	  * The function returns true if the operation suceeds. */
	virtual bool InsertParent( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *parent );
	
	/** Replace parent at the given position. */
	virtual void ReplaceParent( unsigned int pos, GraphNode *parent );
	
	/** Replace a parent to the given node at the given position. This method is provided mainly to
	  * support Composite nodes with the same interface. This version will insert the parent node 
	  * to the current one only if id coincides with the current node identifier. 
	  * The function returns true if the operation suceeds. */
	virtual bool ReplaceParent( NodeIdentifier targetNodeId, unsigned int pos, GraphNode *parent );
	
	/** Replace parent searching its position. Return true if the node is found and replaced. */
	virtual bool ReplaceParent( GraphNode *oldParent, GraphNode *newParent );
	
	/** Remove a parent from the list. Returns true if the node is found and removed. */
	virtual bool RemoveParent( GraphNode *parent );
	
	/** Remove a parent node by position. */
	virtual void RemoveParent( unsigned int pos );
	
	/** Remove a parent to the given node at the given position. This method is provided mainly to
	  * support Composite nodes with the same interface. This version will remove the parent node 
	  * to the current one only if id coincides with the current node identifier. 
	  * The function returns true if the operation suceeds. */
	virtual bool RemoveParent( NodeIdentifier targetNodeId, unsigned int pos );
	virtual bool RemoveParent( NodeIdentifier targetNodeId, GraphNode *parent );
	
	/** Remove all parents of this node. */
	inline virtual void RemoveAllParents()
		{	m_Parents.clear();	}
	
	/** Get the number of parents. */
	inline unsigned int GetNumberOfParents() const 
		{	return m_Parents.size();	}
		
	/** Visitor pattern. Accept a node visitor to start the traversal. This is called the first time to 
	  * start the traversal on the current node. */
	virtual void Accept( GraphNodeVisitor * visitor );
		
  /** Visitor pattern. Traverse downwards (this node and its children). This is called during the
	  * traversal process which starts with a call to Visit using a GraphNodeVisitor. NOTE: this 
	  * method is not intended to be called directly by the user. */
	virtual void Traverse( GraphNodeVisitor * visitor );
	
	/** Visitor pattern. Traverse upwards (this node and its parents). This is called during the
	  * traversal process which starts with a call to Visit using a GraphNodeVisitor. NOTE: this 
	  * method is not intended to be called directly by the user. */
	virtual void Ascend( GraphNodeVisitor * visitor );
		
protected:
  
  GraphNode();
  ~GraphNode();
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  GraphNode(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

	/** Parent nodes (weak references). */
	ParentNodeContainer	  m_Parents;

	/** Children nodes. */
	NodeContainer		m_Children;
	
	/** Mask that can be used for several purposes, for example visiting
	  * a node or not. */
	NodeMaskType    m_Mask;
	
	/** Label that can be used for classification or counting purposes. */
	NodeIdentifier  m_NodeId;
	
	/** Semantic name of the node. */
	std::string     m_Name;
	  
	/** This is an inverse order from root to the current node, that is, the
	  * number of intermediate nodes. This is best assigned by a specific visitor. */	
	int             m_DepthLevel;
};

} // end namespace ivan

#endif
