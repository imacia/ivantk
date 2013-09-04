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
// File: ivanStenosisModel.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : 
// Date: 2009/02/06


#ifndef __ivanStenosisModel_h
#define __ivanStenosisModel_h

#include "ivanVesselFeatureModel.h"


namespace ivan
{
  
/** \class StenosisModel
 *  \brief Model describing a vessel stenosis, that is, a narrowing of the vessel.
 * 
 *
 * \ingroup 
 */
 
class ITK_EXPORT StenosisModel : public VesselFeatureModel
{

public:

  /** Standard class typedefs. */
  typedef StenosisModel                  Self;
  typedef VesselFeatureModel             Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
       
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( StenosisModel, VesselFeatureModel );
  
  /** Get they type of the feature. This is must be reimplemented by subclasses
    * by providing a string such as "Stenosis" etc. */
  virtual std::string GetFeatureType() const
    { return Self::GetFeatureTypeStatic(); }
    
  static std::string GetFeatureTypeStatic()
    { return "Stenosis"; }
  
protected:
  
  StenosisModel();
  ~StenosisModel();
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  StenosisModel(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:


};

} // end namespace ivan

#endif
