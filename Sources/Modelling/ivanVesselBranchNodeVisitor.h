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
// File: ivanVesselNodeVisitor.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 

#ifndef __ivanVesselNodeVisitor_h
#define __ivanVesselNodeVisitor_h

#include "ivanGraphNode.h"

#include "itkLightObject.h"


namespace ivan
{
  

/** \class VesselBranchNodeVisitor
 *  \brief 
 *
 * 
 *
 * \ingroup 
 */

class ITK_EXPORT VesselBranchNodeVisitor : public VesselNodeVisitor
{

public:

  /** Standard class typedefs. */
  typedef VesselBranchNodeVisitor         Self;
  typedef VesselNodeVisitor               Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
 

public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselBranchNodeVisitor, VesselNodeVisitor );
  
  /** Start visiting given node. */
	virtual void Visit( VesselBranchNode & node );
	virtual void Visit( VesselBifurcationNode & node );
	
protected:

  VesselNodeVisitor();
  ~VesselNodeVisitor();
  
  void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:

  VesselNodeVisitor(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:



};

} // end namespace ivan

#endif
