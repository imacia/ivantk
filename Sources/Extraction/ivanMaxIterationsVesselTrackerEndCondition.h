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
// File: ivanMaxIterationsVesselTrackerEndCondition_h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/06/05


#ifndef __ivanMaxIterationsVesselTrackerEndCondition_h
#define __ivanMaxIterationsVesselTrackerEndCondition_h

#include "ivanVesselTrackerEndCondition.h"


namespace ivan
{

/** \class MaxIterationsVesselTrackerEndCondition
 *  \brief Simple end condition based on a number of iterations.
 *
 * This class implements and end condition for vessel tracking based on a maximum number
 * of iterations. This may be useful for testing algorithms.
 *
 */

class ITK_EXPORT MaxIterationsVesselTrackerEndCondition : public VesselTrackerEndCondition
{
public:
  
  typedef MaxIterationsVesselTrackerEndCondition   Self;
  typedef VesselTrackerEndCondition                Superclass;
  typedef itk::SmartPointer<Self>                  Pointer;
  typedef itk::SmartPointer<const Self>            ConstPointer;
    
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( MaxIterationsVesselTrackerEndCondition, VesselTrackerEndCondition );
  
  virtual void Reset()
    { this->m_Iterations = 0; }
  
  itkSetMacro( MaxIterations, unsigned int );
  itkGetConstMacro( MaxIterations, unsigned int );
  
  virtual bool Finished()
    {
      if( ++this->m_Iterations >= this->m_MaxIterations )
        return true;
      else
        return false;
    }

protected:
  
  MaxIterationsVesselTrackerEndCondition() : m_MaxIterations(1), m_Iterations(0) {}
  virtual ~MaxIterationsVesselTrackerEndCondition() {}
  virtual void PrintSelf(std::ostream& os, itk::Indent indent) const
    { 
      Superclass::PrintSelf( os, indent );
      os << indent << "MaxIterations: " << this->m_MaxIterations << std::endl;
      os << indent << "Iterations: " << this->m_Iterations << std::endl;
    }
      
private:
  
  MaxIterationsVesselTrackerEndCondition(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
  
protected:
  
  unsigned int    m_MaxIterations;
  unsigned int    m_Iterations;
};

} // end namespace ivan

#endif
