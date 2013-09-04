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
// File: ivanSearchByIdNodeVisitor.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description:
// Date: 2010/07/29

#ifndef __ivanSearchByIdNodeVisitor_h
#define __ivanSearchByIdNodeVisitor_h


#include "ivanVesselNodeVisitor.h"
#include "ivanGraphNodeVisitorDispatcher.h"


namespace ivan
{
  
/** \class 
 *  \brief 
 *
 * 
 *
 * \ingroup 
 */
 
class ITK_EXPORT SearchByIdNodeVisitor : public GraphNodeVisitor
{
public:

  /** Standard class typedefs. */
  typedef SearchByIdNodeVisitor     Self;
  typedef GraphNodeVisitor          Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
      
public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( SearchByIdNodeVisitor, GraphNodeVisitor );
  
  itkGetConstMacro( Node, const GraphNode * );
  
  itkSetMacro( NodeId, GraphNode::NodeIdentifier );
  itkGetConstMacro( NodeId, GraphNode::NodeIdentifier );
    
  /** Increment the counter for this node type only. */
  void Apply( GraphNode * node );
	
protected:

  SearchByIdNodeVisitor();
  ~SearchByIdNodeVisitor();
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  SearchByIdNodeVisitor(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  GraphNode::Pointer           m_Node;
   
  GraphNode::NodeIdentifier    m_NodeId;
};

} // end namespace ivan

#endif
