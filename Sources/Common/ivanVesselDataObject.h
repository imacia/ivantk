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
// File: ivanVesselObject.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/05/27


#ifndef __ivanVesselDataObject_h
#define __ivanVesselDataObject_h

#include "itkDataObject.h"

namespace ivan
{
  
/** \class VesselDataObject
 *  \brief DataObject that represents a vessel or tube-like structure
 *
 * This is the base class for all objects that represent a vessel or tube-like structure.
 * These objects may be structured differently, for example in the form of a graph or in
 * the form of a set of branches that has not been yet connected.
 *
 * \ingroup 
 */
 
class ITK_EXPORT VesselDataObject : public itk::DataObject
{

public:

  /** Standard class typedefs. */
  typedef VesselDataObject       	      Self;
  typedef itk::DataObject               Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;
       
public:

	/** Method for creation through the object factory. */
  //itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselDataObject, itk::DataObject );
  
  /** Check if this is a vessel graph or not. */
  virtual bool IsGraph()
    { return false; }
  		
protected:
  
  VesselDataObject() {}
  ~VesselDataObject() {}
    
  //void PrintSelf( std::ostream& os, itk::Indent indent ) const {}

private:

  VesselDataObject(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  
};

} // end namespace ivan

#endif
