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
// File: ivanVesselGraph.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2009/02/06


#ifndef __ivanVesselGraph_h
#define __ivanVesselGraph_h

#include "ivanVesselDataObject.h"
#include "ivanVesselBranchNode.h"

namespace ivan
{
  
/** \class VesselGraph
 *  \brief Represents a VesselObject in the form of a graph
 *
 * VesselGraph represents a VesselObject whose branches are organized in
 * the form of a graph. The graph has a single root VesselNode. 
 *
 * \ingroup 
 *
 * 
 */
 
template <class TCenterline>
class ITK_EXPORT VesselGraph : public VesselDataObject
{

public:

  /** Standard class typedefs. */
  typedef VesselGraph       		          Self;
  typedef VesselDataObject                Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;
  
  typedef VesselBranchNode<TCenterline>         BranchNodeType;
  typedef typename BranchNodeType::Pointer      BranchNodePointer;
       
public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselGraph, VesselDataObject );
  
  /** Check if this is a vessel graph or not. */
  virtual bool IsGraph()
    { return true; }
  
  virtual void SetRootNode( VesselNode *node )
    { m_RootNode = node; }
  
  VesselNode::Pointer GetRootNode() 
    { return m_RootNode; }
  VesselNode::ConstPointer GetRootNode() const
    { return m_RootNode; }
  		
protected:
  
  VesselGraph() {}
  ~VesselGraph() {}
    
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const
    {
      Superclass::PrintSelf( os, indent );
      
      os << indent << "RootNode: " << m_RootNode.GetPointer() << std::endl;
      if( m_RootNode.IsNotNull() )
        m_RootNode->Print( os, indent.GetNextIndent() );  
    }
    
private:

  VesselGraph(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  VesselNode::Pointer   m_RootNode;
};

} // end namespace ivan

#endif
