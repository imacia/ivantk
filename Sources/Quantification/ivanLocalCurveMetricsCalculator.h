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
// File: ivanLocalCurveMetricsCalculator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanLocalCurveMetricsCalculator_h
#define __ivanLocalCurveMetricsCalculator_h


#include "ivanVesselCenterlineMetricsCalculator.h"
#include "ivanLocalCurveMetrics.h"

#include "itkConceptChecking.h"


namespace ivan
{


/** \class LocalCurveMetricsCalculator
 *  \brief Calculates curve metrics of type LocalCurveMetrics.
 *
 * For this class to work, Section
 * 
 * \ingroup
 *
 */

template <class TCenterline>
class ITK_EXPORT LocalCurveMetricsCalculator : public VesselCenterlineMetricsCalculator<TCenterline>
{
public:

  /** Standard class typedefs. */
  typedef LocalCurveMetricsCalculator            Self;
  typedef VesselCenterlineMetricsCalculator      Superclass;
  typedef itk::SmartPointer<Self>                Pointer;
  typedef itk::SmartPointer<const Self>          ConstPointer;
 
  typedef typename Superclass::CenterlineType    CenterlineType;
  typedef typename Superclass::Pointer           CenterlinePointer;
  
  typedef typename Superclass::SectionType       SectionType;
  typedef typename Superclass::SectionPointer    SectionPointer;
    
#ifdef ITK_USE_CONCEPT_CHECKING
  /** Begin concept checking */
  itkConceptMacro( CurveMetricsTypeCheck,
      ( Concept::SameType<typename SectionType::CurveMetricsType, LocalCurveMetrics> ) );
  /** End concept checking */
#endif
    
public:

	/** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( LocalCurveMetricsCalculator, VesselCenterlineMetricsCalculator );
  
  /** Compute calculations on current branch. */
	virtual void Compute();
			
protected:

  LocalCurveMetricsCalculator() : m_VesselBranch(0) {};
  ~LocalCurveMetricsCalculator() {};
  
  void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:

  LocalCurveMetricsCalculator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanLocalCurveMetricsCalculator.hxx"
#endif

#endif
