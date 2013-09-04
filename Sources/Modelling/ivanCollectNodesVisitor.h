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
// File: ivanCollectNodesVisitor.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanCollectNodesVisitor_h
#define __ivanCollectNodesVisitor_h

#include "ivanGraphNodeVisitor.h"

#include <list>

namespace ivan
{
  
/** \class CollectNodesVisitor
 *  \brief Collect nodes of the given type.
 *
 * This visitor collects nodes of the given type, which is the template
 * parameter.
 *
 * \ingroup 
 */


template <class TNode>
class ITK_EXPORT CollectNodesVisitor : public GraphNodeVisitor
{

public:

  /** Standard class typedefs. */
  typedef CollectNodesVisitor         Self;
  typedef GraphNodeVisitor            Superclass;
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;
 
  typedef TNode                       NodeType;
  typedef typename NodeType::Pointer  NodePointer;
    
  typedef std::list<NodePointer>      NodeContainer;
  
public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( CollectNodesVisitor, GraphNodeVisitor );
  
  /** Reset the visitor. Useful to reuse the visitor if it accumulates state during 
    * a traversal and we plan to reuse it. */
  virtual void Reset() 
    { m_Nodes.clear(); }
  
  /** Start visiting given node. */
	virtual void Visit( GraphNode *node );
	
	NodeContainer & GetNodeContainer()
	  { return m_Nodes; }
  const NodeContainer & GetNodeContainer() const
	  { return m_Nodes; }
	  
	void GetNumberOfNodes() const
	  { return m_Nodes.size(); }
		
protected:

  CollectNodesVisitor();
  ~CollectNodesVisitor();
  
  void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:

  CollectNodesVisitor(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  NodeContainer    m_Nodes;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanCollectNodesVisitor.hxx"
#endif

#endif
