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
// File: ivanVesselCenterlineAlgorithmVisitor.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanVesselCenterlineAlgorithmVisitor_h
#define __ivanVesselCenterlineAlgorithmVisitor_h


namespace ivan
{
  
/** \class VesselCenterlineAlgorithmVisitor
 *  \brief 
 *
 * This class uses as template parameter a subclass of VesselCenterlineInterpolator.
 *
 * \ingroup 
 */

template <class TCenterlineAlgorithm>
class ITK_EXPORT VesselCenterlineAlgorithmVisitor : public VesselNodeVisitor
{

public:

  /** Standard class typedefs. */
  typedef VesselCenterlineAlgorithmVisitor  Self;
  typedef VesselNodeVisitor     Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
 
  typedef TCenterlineAlgorithm              AlgorithmType;
  typedef typename AlgorithmType::Pointer   AlgorithmPointer;
  typedef typename AlgorithmType::Pointer   AlgorithmConstPointer;
  
public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselCenterlineAlgorithmVisitor, VesselNodeVisitor );
  
  /** Set an external algorithm object. */
  void SetCenterlineAlgorithm( AlgorithmType *algorithm )
    { m_Algorithm = algorithm; }
  
  /** Get access to the centerline algorithm object. */
  AlgorithmPointer GetCenterlineAlgorithm()
    { return m_Algorithm; }
  AlgorithmConstPointer GetCenterlineAlgorithm() const
    { return m_Algorithm; }
  
  /** Start visiting given node. */
	virtual void Visit( VesselBranchNode *node );
		
protected:

  VesselCenterlineAlgorithmVisitor();
  ~VesselCenterlineAlgorithmVisitor();
  
  void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:

  VesselCenterlineAlgorithmVisitor(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  AlgorithmPointer   m_Algorithm;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanVesselCenterlineAlgorithmVisitor.hxx"
#endif

#endif
