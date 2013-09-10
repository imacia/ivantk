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
// File: ivanVesselNodeVisitor.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description:


#ifndef __ivanVesselNodeVisitor_hxx
#define __ivanVesselNodeVisitor_hxx

#include "ivanVesselNodeVisitor.h"
#include "ivanVesselBranchNode.h"
#include "ivanVesselBifurcationNode.h"
#include "ivanVesselFeatureNode.h"


namespace ivan
{
  
template <class TCenterline>
VesselNodeVisitor<TCenterline>::VesselNodeVisitor()
{

}


template <class TCenterline>
VesselNodeVisitor<TCenterline>::~VesselNodeVisitor()
{

}

/*
template <class TCenterline>
void VesselNodeVisitor<TCenterline>::Visit( VesselNode * node )
{
  this->Visit( static_cast<GraphNode*>( node ) ); 
}


template <class TCenterline>
void VesselNodeVisitor<TCenterline>::Visit( BranchNodeType * node )
{
  this->Visit( static_cast<GraphNode*>( node ) ); 
}


template <class TCenterline>
void VesselNodeVisitor<TCenterline>::Visit( BifurcationNodeType * node )
{	
  this->Visit( static_cast<GraphNode*>( node ) ); 
}


template <class TCenterline>
void VesselNodeVisitor<TCenterline>::Visit( VesselFeatureNode * node )
{
  this->Visit( static_cast<GraphNode*>( node ) ); 
}
*/

template <class TCenterline>
void VesselNodeVisitor<TCenterline>::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );

}

} // end namespace ivan

#endif
