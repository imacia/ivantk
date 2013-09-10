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
// File: ivanVesselCenterlineMetricsCalculator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanVesselCenterlineMetricsCalculator_h
#define __ivanVesselCenterlineMetricsCalculator_h


namespace ivan
{


/** \class VesselCenterlineMetricsCalculator
 *  \brief Base class for calculating measurements in branches
 *
 * This class calculates measurements in individual branches or as part of 
 * the VesselCenterlineMetricsCalculatorVisitor, which calculates properties
 * through all the branches in a graph.
 *
 * It takes as template parameter the section type, since measurements are
 * specific of the type of section and thus, corresponding calculations.
 *
 * \ingroup
 *
 */

template <class TCenterline>
class ITK_EXPORT VesselCenterlineMetricsCalculator : public itk::Object
{
public:

  /** Standard class typedefs. */
  typedef VesselCenterlineMetricsCalculator      Self;
  typedef itk::Object                            Superclass;
  typedef itk::SmartPointer<Self>                Pointer;
  typedef itk::SmartPointer<const Self>          ConstPointer;
 
  typedef TCenterline                            CenterlineType;
  typedef typename CenterlineType::Pointer       CenterlinePointer;
  
  typedef typename CenterlineType::SectionType   SectionType;
  typedef typename SectionType::Pointer          SectionPointer;
  
public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselCenterlineMetricsCalculator, itk::Object );
  
  /** Compute calculations on current branch. */
	virtual void Compute() = 0;
	
	virtual void SetCenterline( CenterlineType * centerline )
	  { m_Centerline = centerline; }
		
protected:

  VesselCenterlineMetricsCalculator() : m_VesselBranch(0) {};
  ~VesselCenterlineMetricsCalculator() {};
  
  void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:

  VesselCenterlineMetricsCalculator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  CenterlinePointer  m_Centerline;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanVesselCenterlineMetricsCalculator.hxx"
#endif

#endif
