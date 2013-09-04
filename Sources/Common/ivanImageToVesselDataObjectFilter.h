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
// File: ivanImageToVesselDataObjectFilter.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/05/27


#ifndef __ivanImageToVesselDataObjectFilter_h
#define __ivanImageToVesselDataObjectFilter_h

#include "ivanVesselDataObjectSource.h"


namespace ivan
{
  
/** \class ImageToVesselDataObjectFilter
 * \brief Base class for filters that take an image as input and produce a VesselDataObject as output.
 *
 * ImageToVesselDataObjectFilter is the base class for all process objects that output
 * VesselDataObject data and require an image as input. Specifically, this class
 * defines the SetInput() method for defining the input to a filter.
 *
 *
 */

template <class TInputImage, class TOutputVessel>
class ITK_EXPORT ImageToVesselDataObjectFilter : public VesselDataObjectSource<TOutputVessel>
{
public:
	
  /** Standard class typedefs. */
  typedef ImageToVesselDataObjectFilter           Self;
  typedef VesselDataObjectSource<TOutputVessel>   Superclass;
  typedef itk::SmartPointer<Self>                      Pointer;
  typedef itk::SmartPointer<const Self>                ConstPointer;
  
  /** Some convenient typedefs. */
  typedef TInputImage                             InputImageType;
  typedef typename InputImageType::Pointer        InputImagePointer;
  typedef typename InputImageType::ConstPointer   InputImageConstPointer;
  typedef typename InputImageType::RegionType     InputImageRegionType; 
  typedef typename InputImageType::PixelType      InputImagePixelType; 
  	
  /** Some convenient typedefs. */
  typedef TOutputVessel                           OutputVesselType;
  typedef typename OutputVesselType::Pointer      OutputVesselPointer;
  
  /** ImageDimension constants */
  itkStaticConstMacro(InputImageDimension, unsigned int,
                      TInputImage::ImageDimension);
  
public:
  
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( ImageToVesselDataObjectFilter, VesselDataObjectSource );

  /** Set/Get the image input of this process object.  */
  virtual void SetInput( const InputImageType *image);
  virtual void SetInput( unsigned int, const InputImageType * image);
  const InputImageType * GetInput(void);
  const InputImageType * GetInput(unsigned int idx);

protected:
  
  ImageToVesselDataObjectFilter();
  ~ImageToVesselDataObjectFilter() {}
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  ImageToVesselDataObjectFilter(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanImageToVesselDataObjectFilter.hxx"
#endif

#endif
