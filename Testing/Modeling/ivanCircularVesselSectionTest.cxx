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
// File: ivanCircularVesselSectionTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description:

#include "ivanVesselBranchNode.h"
#include "ivanVesselCenterline.h"
#include "ivanCircularVesselSection.h"
#include "ivanVesselNodeVisitor.h"

#include <stdlib.h>
#include <ctime>


int main( int argc, const char *argv[] )
{
  typedef ivan::CircularVesselSection<>   VesselSectionType;
  typedef ivan::VesselCenterline
    <unsigned int, VesselSectionType>         CenterlineType;
    
  typedef ivan::VesselBranchNode<CenterlineType>   VesselBranchNodeType;

  VesselBranchNodeType::Pointer vesselBranch1 = VesselBranchNodeType::New();
  CenterlineType::Pointer centerline1 = vesselBranch1->GetCenterline();
  
  typedef VesselSectionType::CenterPointType  CenterPointType;
  CenterPointType point;
  VesselSectionType::Pointer section;
  srand( (unsigned) time( NULL ) );
 
  for( unsigned int i=0; i<20; ++i )
  {    
    point[0] = ( rand() % 100 );
    point[1] = ( rand() % 100 );
    point[2] = ( rand() % 100 );
    
    section = VesselSectionType::New();
    section->SetCenter( point );

    centerline1->push_back( section );
  }

  // Print the result

  vesselBranch1->Print( std::cout );

  return EXIT_SUCCESS; 
}
