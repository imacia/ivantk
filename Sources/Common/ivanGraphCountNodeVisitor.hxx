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
// File: ivanGraphCountNodeVisitor.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description:


#ifndef __ivanGraphCountNodeVisitor_hxx
#define __ivanGraphCountNodeVisitor_hxx

#include "ivanGraphCountNodeVisitor.h"

namespace ivan
{
  
template <class TNode>
GraphCountNodeVisitor<TNode>::GraphCountNodeVisitor() :
  m_Count(0)
{
  this->AddDispatcher<Self,TNode>();
}


template <class TNode>
GraphCountNodeVisitor<TNode>::~GraphCountNodeVisitor()
{

}


template <class TNode>
void GraphCountNodeVisitor<TNode>::Apply( NodeType * node )
{
  ++m_Count;
  
  this->Traverse( node ); // continue with traversal
}


template <class TNode>
void GraphCountNodeVisitor<TNode>::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Count: " << m_Count << std::endl;
}

} // end namespace ivan

#endif
