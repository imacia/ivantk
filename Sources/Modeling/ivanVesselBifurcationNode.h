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
// File: ivanVesselBifurcationNode.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanVesselBifurcationNode_h
#define __ivanVesselBifurcationNode_h

#include "ivanVesselNode.h"
#include "ivanVesselCenterline.h"
#include "ivanVesselBranchNode.h"


namespace ivan
{
  
/** \class VesselBifurcationNode
 *  \brief 
 *
 *
 * \ingroup 
 */
 
//template <class TCenterline>
//class VesselBranchNode;
 
template <class TCenterline>
class ITK_EXPORT VesselBifurcationNode : public VesselNode
{

public:

  /** Standard class typedefs. */
  typedef VesselBifurcationNode<TCenterline>    Self;
  typedef VesselNode                            Superclass;
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;

  typedef VesselBranchNode<TCenterline>         BranchNodeType;
  
  //itkStaticConstMacro( Dimension, unsigned int, VDimension );
       
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkNodeTypeMacro( VesselBifurcationNode, VesselNode );
  
  /** Get the parent branch. */
  BranchNodeType * GetParentBranch();
  const BranchNodeType * GetParentBranch() const;
  
  /** Get child branches. */
  std::vector<BranchNodeType*> GetChildBranches();
  std::vector<BranchNodeType*> GetChildBranches() const;
    
  /** Get the angle between two branches in radians. The ids correspond to the
    * branch ids, not to the graph positions. */
  double GetBranchAngle( VesselIdType branchId1, VesselIdType branchId2 );
      
protected:
  
  VesselBifurcationNode();
  ~VesselBifurcationNode();
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  VesselBifurcationNode(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanVesselBifurcationNode.hxx"
#endif

#endif

