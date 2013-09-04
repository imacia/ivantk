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
// File: ivanVesselSectionFitCostFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2011/11/05


#ifndef __ivanVesselSectionFitCostFunction_h
#define __ivanVesselSectionFitCostFunction_h


#include "itkSingleValuedCostFunction.h"
#include "itkPoint.h"


namespace ivan
{
/** \class VesselSectionFitCostFunction
 *  \brief Base class of cost functions for calculationg optimized vessel sections
 *
 * This is the base class of cost functions that are to be maximized or minimized by calculating
 * an optimal set of parameters of the function. Typical parameters are the normal and center of
 * the section. The function must be designed so it gives a minimum (maximum) when the parameters
 * approximate well the real vessel section.
 *
 *   
 */
class VesselSectionFitCostFunction : public itk::SingleValuedCostFunction
{
public:
  
  /** Standard class typedefs. */
  typedef VesselSectionFitCostFunction      Self;
  typedef itk::SingleValuedCostFunction     Superclass;
  typedef itk::SmartPointer<Self>           Pointer;
  typedef itk::SmartPointer<const Self>     ConstPointer;

  typedef Superclass::MeasureType           MeasureType;
    
   typedef double                           PointValueType;
  typedef itk::Point<PointValueType,3>      PointType;
  
public:
	
	/** Run-time type information (and related methods). */
  itkTypeMacro( VesselSectionFitCostFunction, itk::SingleValuedCostFunction );
    
  /** Set/Get the section center. */
  itkSetMacro( SectionCenter, PointType );
  itkGetMacro( SectionCenter, PointType );
  itkGetConstMacro( SectionCenter, PointType );
    
protected:
  /** Constructor: */
  VesselSectionFitCostFunction() 
    { this->m_SectionCenter.Fill( 0.0 ); }

  /** Destructor: */
  virtual ~VesselSectionFitCostFunction() {};
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const
    {
      Superclass::PrintSelf( os, indent );
      os << indent << "SectionCenter: " << this->m_SectionCenter << std::endl;
    }

private:
  
  VesselSectionFitCostFunction(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented
   
protected:
   
  PointType             m_SectionCenter;
}; // end of class

} // end namespace ivan

#endif
