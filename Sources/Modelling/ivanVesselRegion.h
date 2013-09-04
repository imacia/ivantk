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
// File: ivanVesselRegion.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : 
// Date: 2009/02/06


#ifndef __ivanVesselRegion_h
#define __ivanVesselRegion_h

#include "ivanVesselCommon.h"

#include "itkRegion.h"
#include "itkNumericTraits.h"

#if ITK_VERSION_MAJOR < 4
 #include "itk_hash_map.h"
  #define ITK_HASH_MULTIMAP itk::hash_multimap
#else
 #include "itksys/hash_map.hxx"
  #define ITK_HASH_MULTIMAP itksys::hash_multimap
#endif

#include <vector>


namespace ivan
{
  
/** \class VesselRegion
 *  \brief Describes a region for processing in a vascular networks.
 *
 * A vessel region consists of a group of branches. For each branch,
 * start and end indexes are defined, which correspond to the range of
 * indexes corresponding to centerline points that are covered by the 
 * region (both start and end index inclusive). Several subregions can
 * be defined for each branch.
 *
 * \ingroup 
 */
 
class ITK_EXPORT VesselRegion : public itk::Region
{
public:

  /** Standard class typedefs. */
  typedef VesselRegion                   Self;
  typedef itk::Region                    Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef std::pair<PointIdType,PointIdType>                  BranchRegionType;
  typedef ITK_HASH_MULTIMAP<BranchIdType,BranchRegionType>    BranchRegionContainerType;
        
public:

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselRegion, itk::Region );
  
  /** Constructor. VesselRegion is a lightweight object that is not reference
    * counted, so the constructor is public. */
  VesselRegion();

  /** Destructor. VesselRegion is a lightweight object that is not reference
    * counted, so the destructor is public. */
  virtual ~VesselRegion();
  
  /** Alternative constructor specifying a whole single branch. */
  VesselRegion( BranchIdType branchId );
      
  /** Alternative constructor specifying a collection of whole branches by corresponding ids. */
  VesselRegion( const std::vector<BranchIdType> & branchIds );
    
  /** Alternative constructor that directly specifies the container. */
  VesselRegion( const BranchRegionContainerType & branchRegions )
    { m_BranchRegions = branchRegions; }
    
  /** Copy constructor. VesselRegion is a lightweight object that is not
    * reference counted, so the copy constructor is public. */
  VesselRegion( const Self & region ) : Region( region ), m_BranchRegions( region.m_BranchRegions ) {}
    
  /** operator=. VesselRegion is a lightweight object that is not reference
    * counted, so operator= is public. */
  void operator =( const Self & region )
    { m_BranchRegions = region.m_BranchRegions; }
  
  /** Return the region type. Vessel regions are unstructured. */
  virtual Superclass::RegionType GetRegionType() const
    { return Superclass::ITK_UNSTRUCTURED_REGION; }
  
  /** Insert a whole branch for the current region. */
  void Insert( BranchIdType branchId );
  
  /** Insert a branch region defined by start and end point indexes (inclusive). */
  void Insert( BranchIdType branchId, PointIdType startPoint, PointIdType endPoint );
  
  /** Insert a collection of whole branches. */
  void Insert( const std::vector<BranchIdType> & branchIds );
    
  /** Get access to the undelying container. USE WITH CARE!!! */
  BranchRegionContainerType & GetBranchRegions()
    { return m_BranchRegions; }
  const BranchRegionContainerType & GetBranchRegions() const
    { return m_BranchRegions; }
    
  /** Clear all branch regions, leaving the VesselRegion empty. */  
  void Clear()
    { m_BranchRegions.clear(); }
  
  /** Make the defined branch regions consistent, so for each branch there is no overlapping
    * and branch regions become merged accordingly. */
  void MakeConsistent();

  /** Static method that returns a pair specifying the full range of a branch. This is useful
    * since we don't need to know the real size of the branch. */
  static BranchRegionType GetFullBranchRange()
    { return BranchRegionType( itk::NumericTraits<PointIdType>::max(), itk::NumericTraits<PointIdType>::max() ); } 
      
protected:
  
  /** Methods invoked by Print() to print information about the object
    * including superclasses. */
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

protected:

  BranchRegionContainerType   m_BranchRegions; 
};

} // end namespace ivan

#endif
