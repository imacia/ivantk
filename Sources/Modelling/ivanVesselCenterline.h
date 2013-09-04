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
// File: ivanVesselCenterline.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanVesselCenterline_h
#define __ivanVesselCenterline_h


#include "itkVectorContainer.h"


namespace ivan
{
  
/** \class VesselCenterline
 *  \brief 
 *
 *
 * \ingroup 
 */

template <class TElementIdentifier, class TVesselSection>
class ITK_EXPORT VesselCenterline 
  : public itk::VectorContainer<TElementIdentifier, typename TVesselSection::Pointer>
{

public:

  /** Standard class typedefs. */
  typedef VesselCenterline                  Self;
  typedef itk::VectorContainer<TElementIdentifier, 
    typename TVesselSection::Pointer>       Superclass;
  typedef itk::SmartPointer<Self>           Pointer;
  typedef itk::SmartPointer<const Self>     ConstPointer;
  
  typedef TVesselSection                    SectionType;
  typedef typename TVesselSection::Pointer  SectionPointer;
     
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselCenterline, VectorContainer );
  
  unsigned int GetNumberOfCenterlinePoints() const
    { return this->size(); }
  unsigned int GetNumberOfSections() const
    { return this->size(); }
    
  SectionType * GetFirstSection()
    { 
      if( this->size() )
        return this->at(0);
      else 
        return 0;
    }
    
  const SectionType * GetFirstSection() const
    { 
      if( this->size() )
        return this->at(0);
      else 
        return 0;
    }
      
  SectionType * GetLastSection()
    { 
      if( this->size() )
        return this->at( this->size() - 1 );
      else 
        return 0;
    }
  
  const SectionType * GetLastSection() const
    { 
      if( this->size() )
        return this->at( this->size() - 1 );
      else 
        return 0;
    }
    
  /** Compute centerline metrics. */
  virtual void ComputeMetrics();
    
protected:
  
  VesselCenterline();
  ~VesselCenterline();
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  VesselCenterline(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanVesselCenterline.hxx"
#endif

#endif
