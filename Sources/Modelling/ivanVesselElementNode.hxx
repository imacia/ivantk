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
// File: ivanVesselBifurcationNode.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanVesselBifurcationNode_hxx
#define __ivanVesselBifurcationNode_hxx


#include "ivanVesselBifurcationNode.h"
#include "ivanVesselBranchNode.h"


namespace ivan
{

template <unsigned int VDimension>
VesselBifurcationNode<VDimension>::VesselBifurcationNode()
{
  
}


template <unsigned int VDimension>
VesselBifurcationNode<VDimension>::~VesselBifurcationNode()
{

}


template <unsigned int VDimension>
VesselBranchNode * VesselBifurcationNode<VDimension>::GetParentBranch()
{
  return dynamic_cast<VesselBranchNode*>( this->GetParent() );
}


template <unsigned int VDimension>
const VesselBranchNode * VesselBifurcationNode<VDimension>::GetParentBranch() const
{
  return dynamic_cast<VesselBranchNode*>( this->GetParent() );  
}


template <unsigned int VDimension>
std::vector<VesselBranchNode*> VesselBifurcationNode<VDimension>::GetChildBranches()
{
  std::vector<VesselBranchNode*> branches;
  VesselBranchNode *current;
  
  for( unsigned int i=0; i<this->GetNumberOfChildren(); ++i )
  {
    if( ( current = dynamic_cast<VesselBranchNode*>( this->m_Children[i] ) ) )
      branches.push_back( current );
  }
  
  return branches;
}


template <unsigned int VDimension>
std::vector<VesselBranchNode*> GetChildBranches() const
{
  std::vector<VesselBranchNode*> branches;
  VesselBranchNode *current;
  
  for( unsigned int i=0; i<this->GetNumberOfChildren(); ++i )
  {
    if( ( current = dynamic_cast<VesselBranchNode*>( this->m_Children[i] ) ) )
      branches.push_back( current );
  }
  
  return branches;  
}


template <unsigned int VDimension>
void VesselBifurcationNode<VDimension>::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  
}

} // end namespace ivan

#endif // __ivanVesselBifurcationNode_hxx
