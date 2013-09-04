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
// File: ivanVesselBranchMeasurementCalculatorVisitor.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanVesselBranchMeasurementCalculatorVisitor_h
#define __ivanVesselBranchMeasurementCalculatorVisitor_h


namespace ivan
{

/** \class VesselBranchMeasurementCalculatorVisitor
 *  \brief 
 *
 * 
 *
 * \ingroup 
 */

template <class TMetricsCalculator>
class ITK_EXPORT VesselBranchMeasurementCalculatorVisitor : public VesselNodeVisitor
{

public:

  /** Standard class typedefs. */
  typedef VesselBranchMeasurementCalculatorVisitor  Self;
  typedef VesselNodeVisitor     Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
 
  typedef TMetricsCalculator             CalculatorType;
  typedef typename CalculatorType::Pointer   CalculatorPointer;
  
public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselBranchMeasurementCalculatorVisitor, VesselNodeVisitor );
  
  /** Start visiting given node. */
	virtual void Visit( VesselBranchNode *node );
		
protected:

  VesselBranchMeasurementCalculatorVisitor();
  ~VesselBranchMeasurementCalculatorVisitor();
  
  void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:

  VesselBranchMeasurementCalculatorVisitor(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  CalculatorPointer   m_Calculator;
  
  /** Flag that tells if properties should be accumulated along branches. */
  bool                m_Accumulate;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanVesselBranchMeasurementCalculatorVisitor.hxx"
#endif

#endif
