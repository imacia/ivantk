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
// File: ivanVesselTrackerEndCondition.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/05/27


#ifndef __ivanVesselTrackerEndCondition_h
#define __ivanVesselTrackerEndCondition_h

#include "itkObject.h"


namespace ivan
{
  
/** \class VesselTrackerEndCondition
 *  \brief Abstract class that represents and end condition for VesselTrackerFilter.
 *
 * This abstract class represents an end condition for a VesselTrackerFilter. It allows for example 
 * to stop tracking based on several criteria such as vesselness value, distance to previous point,
 * change in vessel direction, etc.
 *
 * This class is templated over the type of vessel tracker filter, so we can access the underlying
 * data.
 *
 */

class ITK_EXPORT VesselTrackerEndCondition : public itk::Object
{
public:
  
  typedef VesselTrackerEndCondition        Self;
  typedef itk::Object                      Superclass;
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;
  
public:

  /** Method for creation through the object factory. */
  //itkNewMacro( Self );
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselTrackerEndCondition, itk::Object );
  
  virtual bool Finished() = 0;

protected:
  
  VesselTrackerEndCondition() {}
  virtual ~VesselTrackerEndCondition() {}
      
private:
  
  VesselTrackerEndCondition(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
  
protected:
  

};

} // end namespace ivan

#endif
