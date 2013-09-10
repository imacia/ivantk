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
// File: ivanVesselRegion.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : generic node of an acyclic graph structure
// Date: 2009/02/06


#include "ivanVesselRegion.h"


namespace ivan
{

VesselRegion::VesselRegion()
{

}


VesselRegion::~VesselRegion()
{

}


VesselRegion::VesselRegion( BranchIdType branchId )
{
   m_BranchRegions.insert( BranchRegionContainerType::value_type( branchId, VesselRegion::GetFullBranchRange() ) );
}


VesselRegion::VesselRegion( const std::vector<BranchIdType> & branchIds )
{
  m_BranchRegions.resize( branchIds.size() );
  for( unsigned int i=0; i<branchIds.size(); ++i )
    m_BranchRegions.insert_noresize( BranchRegionContainerType::value_type( branchIds[i], 
      VesselRegion::GetFullBranchRange() ) );
}


void VesselRegion::Insert( BranchIdType branchId )
{
  m_BranchRegions.insert( BranchRegionContainerType::value_type( branchId, VesselRegion::GetFullBranchRange() ) );
}

  
void VesselRegion::Insert( BranchIdType branchId, PointIdType startPoint, PointIdType endPoint )
{
  m_BranchRegions.insert( BranchRegionContainerType::value_type
    ( branchId, std::make_pair( startPoint, endPoint ) ) );
}
 

void VesselRegion::Insert( const std::vector<BranchIdType> & branchIds )
{
  m_BranchRegions.resize( m_BranchRegions.size() + branchIds.size() );
  for( unsigned int i=0; i<branchIds.size(); ++i )
    m_BranchRegions.insert_noresize( BranchRegionContainerType::value_type( branchIds[i], 
      VesselRegion::GetFullBranchRange() ) );
}


void VesselRegion::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  //for( unsigned int i=0; i<m_BranchRegions.size(); ++i )
  //  os << indent << "Branch 
}

} // end namespace ivan
