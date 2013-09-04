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
// File: ivanVesselDataObjectSource.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/05/27


#ifndef __ivanVesselDataObjectSource_h
#define __ivanVesselDataObjectSource_h

#include "itkProcessObject.h"


namespace ivan
{
  
/** \class VesselDataObjectSource
 *  \brief Base class for all process objects that output VesselDataObject data.
 *
 * VesselDataObjectSource is the base class for all process objects that output
 * VesselDataObject data. Specifically, this class defines the GetOutput() method
 * that returns a pointer to the output VesselDataObject. The class also defines
 * some internal private data members that are used to manage streaming
 * of data.
 *
 * \ingroup DataSources
 */

template <class TOutputVessel>
class ITK_EXPORT VesselDataObjectSource : public itk::ProcessObject
{
public:

  /** Standard class typedefs. */
  typedef VesselDataObjectSource         Self;
  typedef itk::ProcessObject             Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  /** Smart Pointer type to a DataObject. */
  typedef itk::DataObject::Pointer       DataObjectPointer;
  	
  /** Some convenient typedefs. */
  typedef TOutputVessel                          OutputVesselType;
  typedef typename OutputVesselType::Pointer     OutputVesselPointer;
  
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselDataObjectSource, itk::ProcessObject );
  
  /** Get the output data of this process object.  The output of this
    * function is not valid until an appropriate Update() method has
    * been called, either explicitly or implicitly. */
  OutputVesselType * GetOutput(void);
  OutputVesselType * GetOutput(unsigned int idx);
  
  /** Make a DataObject of the correct type to used as the specified
    * output.  Every ProcessObject subclass must be able to create a
    * DataObject that can be used as a specified output. This method
    * is automatically called when DataObject::DisconnectPipeline() is
    * called.  DataObject::DisconnectPipeline, disconnects a data object
    * from being an output of its current source.  When the data object
    * is disconnected, the ProcessObject needs to construct a replacement
    * output data object so that the ProcessObject is in a valid state.
    * So DataObject::DisconnectPipeline eventually calls
    * ProcessObject::MakeOutput. Note that MakeOutput always returns a
    * itk::SmartPointer to a DataObject. If a subclass of VesselDataObjectSource has
    * multiple outputs of different types, then that class must provide
    * an implementation of MakeOutput(). */
  virtual DataObjectPointer MakeOutput(unsigned int idx);

protected:
  
  VesselDataObjectSource();
  virtual ~VesselDataObjectSource() {}
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;
  
  // Inherit the empty ProcessObject::GenerateData()
  
  // Inherit ProcessObject::PrepareOutputs(), which calls Initialize()
  // (Image replaces w/ empty function)
  
private:
  VesselDataObjectSource(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanVesselDataObjectSource.hxx"
#endif

#endif
