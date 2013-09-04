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
// File: ivanGraphNodeVisitorDispatcher.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : implements the GOF Visitor pattern for graph nodes

#ifndef __ivanGraphNodeVisitorDispatcher_h
#define __ivanGraphNodeVisitorDispatcher_h

#include "itkLightObject.h"
//#include "ivanGraphNode.h"


namespace ivan
{
  
class GraphNode;
class GraphNodeVisitor;

/** \class GraphNodeVisitorDispatcher
 *  \brief Helper class for the dispatch mechanism used by visitors
 *
 * 
 *
 * \ingroup 
 */

class ITK_EXPORT GraphNodeVisitorDispatcherBase : public itk::LightObject
{
public:

  typedef GraphNodeVisitorDispatcherBase 	Self;
  typedef itk::LightObject                Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;

public:

  /** Run-time type information (and related methods). */
  itkTypeMacro( GraphNodeVisitorDispatcherBase, itk::LightObject );

  virtual void Apply( GraphNodeVisitor * visitor, GraphNode * node ) = 0;
};


template <class TVisitor, class TNode>
class ITK_EXPORT GraphNodeVisitorDispatcher : public GraphNodeVisitorDispatcherBase
{
public:
  
  typedef GraphNodeVisitorDispatcher    	Self;
  typedef GraphNodeVisitorDispatcherBase 	Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( GraphNodeVisitorDispatcher, GraphNodeVisitorDispatcherBase );
  
  virtual void Apply( GraphNodeVisitor * visitor, GraphNode * node )
    {
      static_cast<TVisitor*>( visitor )->Apply( static_cast<TNode*>( node ) );
    }
};

} // end namespace ivan

#endif
