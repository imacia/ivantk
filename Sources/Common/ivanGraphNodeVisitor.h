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
// File: ivanGraphNodeVisitor.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : implements the GOF Visitor pattern for graph nodes

#ifndef __ivanGraphNodeVisitor_h
#define __ivanGraphNodeVisitor_h

#include "itkObject.h"
#include "ivanGraphNode.h"
#include "ivanGraphNodeVisitorDispatcher.h"

#include <map>
#include <string>

namespace ivan
{

/** \class GraphNodeVisitor
 *  \brief Implements the Visitor Pattern to perform operations on nodes of a graph
 *
 * GraphNodeVisitor is the base class of the objects that traverse the graph and 
 * perform operations on the graph nodes. Specific GraphNodeVisitor subclasses implements
 * the operators in specific types of nodes. This allows to separate the operations from
 * the node definitions.
 *
 * GraphNodeVisitor implements a double-dispatch mechanism that is able to call the
 * appropiate Apply() method in order to perform specific operations depending both on
 * the visitor and node type. For example, a specific visitor may only operate on certain
 * types on nodes, and does nothing on the rest.
 * 
 * Typically the double-dispatch mechanism requires declaring all types of possible nodes
 * in the GraphNode and GraphNodeVisitor classes. However, we want to provide flexibility
 * in order to allow any types of nodes to be defined without altering corresponding base
 * class interfaces. In order to do this, an alternative, although slower mechanism is 
 * provided. The dispatch is delegated into a template based dispatcher object that calls
 * an appropiate non-virtual Apply() method for the given node types, that must be defined 
 * on the specific visitor. There is a dispatcher object for every node type that the 
 * visitor may be applied to. These objects are declared and constructed when the visitor
 * is created and stored in a map. When the visitor receives a given node type, it checks
 * in the map if a dispatcher is available for that node type (since the visitor is the
 * current object) and if so, the corresponding method is applied.
 *
 * Alternatively, subclasses may use other mechanisms such as RTTI (dynamic_cast<> or 
 * typeid operators) in order to call the appropiate non-virtual Apply()�methods. In any
 * case, these calls must be performed from a reimplementation of Visit(). 
 *
 * \ingroup 
 */

class ITK_EXPORT GraphNodeVisitor : public itk::Object
{

public:

  /** Standard class typedefs. */
  typedef GraphNodeVisitor       	      Self;
  typedef itk::LightObject              Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;
  
  /** List of nodes. */
  typedef std::vector<Pointer>		 NodeContainer;
  typedef GraphNode::NodePathType  NodePathType;
  typedef GraphNode::NodeMaskType  NodeMaskType;
  
  /** Map that allows to call the appropiate Apply() method defined in subclasses
    * depending on the node type. This avoids declaring all node types here and 
    * allows flexibility for operating with new node types in the double-dispatch
    * mechanism. The cost is that, for every Apply() call, we need to search the 
    * node type in the map. */
  typedef std::map<std::string,GraphNodeVisitorDispatcherBase::Pointer>   DispatcherMapType;
    
  enum TraversalModeType
  {
    TraverseNone = 0,
    TraverseAllChildren = 1,
    TraverseActiveChildren = 2,
    TraverseAllParents = 3,
    TraverseActiveParents = 4
  };

public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( GraphNodeVisitor, itk::LightObject );
  
  itkSetMacro( TraversalMask, NodeMaskType );
  itkGetConstMacro( TraversalMask, NodeMaskType );
    
  /** Reset the visitor. Useful to reuse the visitor if it accumulates state during 
    * a traversal and we plan to reuse it. */
  virtual void Reset() {}

  /** Start visiting given node. */
	virtual void Visit( GraphNode * node );
	
	/** Generic apply method. This is called from the dispatcher. */
  void Apply( GraphNode * node ) {}
	
  /** Method called by GraphNode::Accept() method in order to visit or not the
    * current node and its descendants or ancestors, depending on the traversal mode.
    * It returns the result of the bit-wise operation between the visitor's traversal
    * mask and the node's node mask. */
  inline bool IsValidMask( const GraphNode * node ) const
    { return ( this->GetTraversalMask() & node->GetMask() ) != 0; }
		
	/** Method called by GraphNode::Accept() before a call to the GraphNodeVisitor::Visit()
	  * method. The back of the path will be the current node being visited inside the 
	  * Visit() method and the rest of the path will be the parental sequence of nodes 
    * from the top most node applied down the graph to the current node. NOTE: this 
	  * method is not intended to be called directly by the user but internally during
	  * the traversal process. */
  inline void PushOntoNodePath( GraphNode* node ) 
    { 
      if ( m_TraversalMode != TraverseAllParents && m_TraversalMode != TraverseActiveParents )
        m_NodePath.push_back( node ); // push at the back
      else
        m_NodePath.insert( m_NodePath.begin(), node ); // insert at the front
    }
        
  /** Method called by GraphNode::Accept() method after a call to GraphNodeVisitor::Visit().
    * It pops the current node from the node path. NOTE: this method is not intended to 
    * be called directly by the user but internally during the traversal process. */
  inline void PopFromNodePath()
    { 
      if ( m_TraversalMode != TraverseAllParents && m_TraversalMode != TraverseActiveParents )
        m_NodePath.pop_back();
      else 
        m_NodePath.erase( m_NodePath.begin() );
    }
      
protected:

  GraphNodeVisitor();
  ~GraphNodeVisitor();
  
  /** Add a new dispatcher. Subclasses must register their visitor-node pairs at construction.
    * They must also define corresponding non-virtual Apply() methods for supported node types. */
  template <class TVisitor, class TNode>
  void AddDispatcher();
  
  /** Traverse the current node. */
	virtual void Traverse( GraphNode * node );
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  GraphNodeVisitor(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  /** Help map that redirects calls to appropiate Apply() method depending on node type. 
    * Subclasses must register their own visitor-node pair types here. */
  DispatcherMapType    m_DispatcherMap;

  /** Traversal type. Default is TraverseAllChildren. */
  TraversalModeType    m_TraversalMode;

  /** Mask that can be used during traversal. */
  NodeMaskType         m_TraversalMask;
    
  /** Current node path. */
  NodePathType         m_NodePath;
};


template <class TVisitor, class TNode>
void GraphNodeVisitor::AddDispatcher()
{
  m_DispatcherMap[ TNode::GetNameOfClassStatic() ] = GraphNodeVisitorDispatcher<TVisitor,TNode>::New();
}


} // end namespace ivan

#endif
