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
// File: ivanVesselGraphCountNodeVisitor2.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 

#ifndef __ivanVesselGraphCountNodeVisitor2_h
#define __ivanVesselGraphCountNodeVisitor2_h


#include "ivanVesselNodeVisitor.h"
#include "ivanGraphNodeVisitorDispatcher.h"


namespace ivan
{
  
/** \class GraphCountNodeVisitor2
 *  \brief Node visitor for counting nodes implemented via dynamic_cast<>
 *
 * 
 *
 * \ingroup 
 */
 
template <class TNode>
class ITK_EXPORT GraphCountNodeVisitor2 : public GraphNodeVisitor
{
public:

  /** Standard class typedefs. */
  typedef GraphCountNodeVisitor2    Self;
  typedef GraphNodeVisitor          Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef TNode     NodeType;
  
public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( GraphCountNodeVisitor2, GraphNodeVisitor );
  
  /** Reimplement in order to provide the alternative implementation. */
  virtual void Visit( GraphNode * node );
  
  /** Reset the visitor. Useful to reuse the visitor if it accumulates state during 
    * a traversal and we plan to reuse it. */
  virtual void Reset() 
    { m_Count = 0; }
    
  itkGetConstMacro( Count, unsigned int );
    
  /** Increment the counter for this node type only. */
  void Apply( NodeType * node );
	
protected:

  GraphCountNodeVisitor2();
  ~GraphCountNodeVisitor2();
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  GraphCountNodeVisitor2(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  unsigned int m_Count;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanGraphCountNodeVisitor2.txx"
#endif

#endif
