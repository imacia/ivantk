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
// File: ivanImageBasedVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2011/11/04


#ifndef __ivanImageBasedVesselSectionEstimator_h
#define __ivanImageBasedVesselSectionEstimator_h


#include "ivanVesselSectionEstimator.h"


namespace ivan
{

/** \class ImageBasedVesselSectionEstimator
 *  \brief Abstract class estimating the vessel section based on image content
 *
 *
 * \ingroup 
 */

template <class TImage, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT ImageBasedVesselSectionEstimator : 
  public VesselSectionEstimator<TCenterline,TMetricsCalculator>
{

public:

  /** Standard class typedefs. */
  typedef ImageBasedVesselSectionEstimator
    <TImage, TCenterline, TMetricsCalculator>   Self;
  typedef VesselSectionEstimator
    <TCenterline,TMetricsCalculator>            Superclass;
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;
  
  typedef TCenterline                           CenterlineType;
  typedef typename CenterlineType::Pointer      CenterlinePointer;
  
  typedef typename CenterlineType::SectionType  SectionType;
  typedef typename SectionType::Pointer         SectionPointer;

  typedef TImage                                ImageType;
  typedef typename ImageType::Pointer           ImagePointer;
    
public:

  /** Method for creation through the object factory. */
  //itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( ImageBasedVesselSectionEstimator, VesselSectionEstimator );
  
  virtual void SetImage( ImageType *image );
 
protected:
  
  ImageBasedVesselSectionEstimator();
  ~ImageBasedVesselSectionEstimator();
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  ImageBasedVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  ImagePointer                  m_Image;  
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanImageBasedVesselSectionEstimator.hxx"
#endif

#endif
