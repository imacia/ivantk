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
#include "ivanGraphCountNodeVisitor2.h"
#include "ivanVesselBranchNode.h"
#include "ivanVesselCenterline.h"
#include "ivanVesselSection.h"

#include "itkPoint.h"
#include "itkRealTimeClock.h"

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

  const unsigned int NumNodes = 1e5;

  // Create a large bunch of nodes
  for( unsigned int i=0; i<NumNodes; ++i )
  {
    vesselBranch = BranchNodeType::New();
    vesselBranch->SetNodeId(i+1);
    nodes.push_back( vesselBranch.GetPointer() );
  }

  GraphType::Pointer graph = GraphType::New();
  graph->SetRootNode( nodes[0] );

  unsigned int j=0, k=0;

  for( unsigned int i=1; i<NumNodes; ++i, ++k )
  {
    if( k>3 ) // add 3 children to each node
    {
      k = 0;
      ++j;
    }

    nodes[j]->AddChild( nodes[i] );
  }

  typedef ivan::GraphCountNodeVisitor<BranchNodeType> BranchNodeCounterType;
  BranchNodeCounterType::Pointer branchNodeCounter = BranchNodeCounterType::New();
    
/* !!! FIGURE OUT HOW TO MAKE THIS BACKWARD COMPATIBLE WITH ITK 3.x
  itk::RealTimeClock::Pointer clock = itk::RealTimeClock::New();
  double time = clock->GetTimeStamp();
*/
  graph->GetRootNode()->Accept( branchNodeCounter );
  
  //time = clock->GetTimeStamp() - time;
  std::cout << "Number of nodes visited: " << branchNodeCounter->GetCount() << std::endl;
  //std::cout << "Visiting time: " << time << std::endl;


  typedef ivan::GraphCountNodeVisitor2<BranchNodeType> BranchNodeCounter2Type;
  BranchNodeCounter2Type::Pointer branchNodeCounter2 = BranchNodeCounter2Type::New();
    
  //itk::RealTimeClock::Pointer clock1 = itk::RealTimeClock::New();
  //time = clock->GetTimeStamp();
  
  graph->GetRootNode()->Accept( branchNodeCounter2 );
  
  //time = clock->GetTimeStamp() - time;
  std::cout << "Number of nodes visited: " << branchNodeCounter2->GetCount() << std::endl;
  std::cout << "Visiting time: " << time << std::endl;

 
  // Print the result

  //vesselBranch->Print( std::cout );

  return EXIT_SUCCESS; 
}
