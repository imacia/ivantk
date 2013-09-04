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
// File: ivanVesselNodeVisitorTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: test the GraphNodeVisitor and subclasses

#include "ivanVesselGraph.h"
#include "ivanGraphCountNodeVisitor.h"
#include "ivanVesselBranchNode.h"
#include "ivanVesselBifurcationNode.h"
#include "ivanVesselCenterline.h"
#include "ivanVesselSection.h"
#include "itkPoint.h"

#include <stdlib.h>
#include <ctime>


int main( int argc, const char *argv[] )
{
    typedef itk::Point<double,3>           PointType;
  typedef ivan::VesselSection
    <PointType>                          VesselSectionType;
  typedef ivan::VesselCenterline
    <unsigned int, VesselSectionType>    CenterlineType;

  typedef ivan::VesselNode                             VesselNodeType;    
  typedef ivan::VesselBranchNode<CenterlineType>       BranchNodeType;
  typedef ivan::VesselBifurcationNode<CenterlineType>  BifurcationNodeType;
  typedef ivan::VesselGraph<CenterlineType>            GraphType;
    
  std::vector<VesselNodeType::Pointer> nodes;
  BranchNodeType::Pointer vesselBranch;

  for( unsigned int i=0; i<5; ++i )
  {
    vesselBranch = BranchNodeType::New();
    vesselBranch->SetNodeId(i+1);
    nodes.push_back( vesselBranch.GetPointer() );
  }

  BifurcationNodeType::Pointer vesselBifurcation;

  for( unsigned int i=5; i<10; ++i )
  {
    vesselBifurcation = BifurcationNodeType::New();
    vesselBifurcation->SetNodeId(i+1);
    nodes.push_back( vesselBifurcation.GetPointer() );
  }

  GraphType::Pointer graph = GraphType::New();
  graph->SetRootNode( nodes[0] );

  nodes[0]->AddChild( nodes[1] );
  nodes[0]->AddChild( nodes[5] );

  nodes[1]->AddChild( nodes[2] );
  nodes[1]->AddChild( nodes[6] );
  nodes[1]->AddChild( nodes[7] );

  nodes[5]->AddChild( nodes[8] );
  nodes[5]->AddChild( nodes[3] );

  nodes[2]->AddChild( nodes[9] );
  nodes[2]->AddChild( nodes[4] );

  // Currently the visitor is not able to visit nodes based on their superclass
  // since the dispatcher checks the actual class name, not the superclass
  /*typedef itk::GraphCountNodeVisitor<VesselNodeType> NodeCounterType;
  NodeCounterType::Pointer nodeCounter = NodeCounterType::New();
  graph->GetRootNode()->Accept( nodeCounter );*/

  //if( nodeCounter->GetCount() != 10 )
    //return EXIT_FAILURE;

  typedef ivan::GraphCountNodeVisitor<BranchNodeType> BranchNodeCounterType;
  BranchNodeCounterType::Pointer branchNodeCounter = BranchNodeCounterType::New();
  graph->GetRootNode()->Accept( branchNodeCounter );

  if( branchNodeCounter->GetCount() != 5 )
    return EXIT_FAILURE;

  typedef ivan::GraphCountNodeVisitor<BifurcationNodeType> BifurcationNodeCounterType;
  BifurcationNodeCounterType::Pointer bifurcationNodeCounter = BifurcationNodeCounterType::New();
  graph->GetRootNode()->Accept( bifurcationNodeCounter );

  if( bifurcationNodeCounter->GetCount() != 5 )
    return EXIT_FAILURE;
  
 
  // Print the result

  //vesselBranch->Print( std::cout );

  return EXIT_SUCCESS; 
}
