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
// File: ivanVesselBranchNode.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanVesselBranchNode_h
#define __ivanVesselBranchNode_h

#include "ivanVesselNode.h"
#include "ivanVesselCommon.h"
#include "ivanVesselCenterline.h"


namespace ivan
{

/** \class VesselBranchNode
 *  \brief 
 *
 *
 * \ingroup 
 */
 
template <class TCenterline>
class ITK_EXPORT VesselBranchNode : public VesselNode
{

public:

  /** Standard class typedefs. */
  typedef VesselBranchNode          Self;
  typedef VesselNode                Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef TCenterline                            CenterlineType;
  typedef typename CenterlineType::Pointer       CenterlinePointer;
  typedef typename CenterlineType::ConstPointer  CenterlineConstPointer;
  
  typedef typename CenterlineType::SectionType   SectionType;
  typedef typename SectionType::Pointer          SectionPointer;
    
  //itkStaticConstMacro( Dimension, unsigned int, VDimension );
          
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkNodeTypeMacro( VesselBranchNode, VesselNode );
  
  /** Set/Get Strahler order of branch. */
  itkSetMacro( Order, int );
  itkGetConstMacro( Order, int );
  
  /** Set a new centerline, for example after interpolating. */
  CenterlinePointer GetCenterline()
    { return m_Centerline; }
  CenterlineConstPointer GetCenterline() const
    { return m_Centerline; }
   
  virtual void SetCenterline( CenterlineType *centerline )
    { m_Centerline = centerline; }
  
  /** Return the number of (centerline) points. */
  void GetNumberOfPoints() const
    { return m_Centerline->GetNumberOfPoints(); }
    
  /** Return the number of sections. */
  void GetNumberOfSections() const
    { return m_Centerline->GetNumberOfSections(); }
  
protected:
  
  VesselBranchNode();
  ~VesselBranchNode();
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  VesselBranchNode(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  CenterlinePointer     m_Centerline;
  
  /** Strahler order of branch. We use a signed integer for convenience, since 
    * in analysis of vessel trees, arteries take positive order numbers and 
    * veins negative. A specific visitor must be used for assigning the order. */
	int                   m_Order;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanVesselBranchNode.hxx"
#endif

#endif
