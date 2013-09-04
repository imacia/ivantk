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
// File: ivanVesselFeatureModel.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : 
// Date: 2009/02/06


#ifndef __ivanVesselFeatureModel_h
#define __ivanVesselFeatureModel_h

#include "itkObject.h"
#include "itkObjectFactory.h"


namespace ivan
{

/** \class VesselFeatureModel
 *  \brief Model describing a vessel feature such as stenoses, aneurysms, strokes, etc.
 *
 * This base class is used to describe vascular features or accidents,
 * or other areas of diagnostic interest, such as stenosis, aneurysms, 
 * strokes, regions feeding tumours, etc. The definition is very open and 
 * subclasses must implement corresponding feature models.
 *
 * VesselFeature nodes have FeatureRegion that describes the affected area.
 * This node type can be assigned as a child of several branch nodes,
 * since the FeatureRegion may affect several branches.
 *
 * \ingroup 
 */
 
class ITK_EXPORT VesselFeatureModel : public itk::Object
{

public:

  /** Standard class typedefs. */
  typedef VesselFeatureModel             Self;
  typedef itk::Object                    Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
       
public:

  /** Method for creation through the object factory. */
  //itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselFeatureModel, itk::Object );
  
  /** Get they type of the feature. This is must be reimplemented by subclasses
    * by providing a string such as "Stenosis" etc. */
  virtual std::string GetFeatureType() const = 0;
  
protected:
  
  VesselFeatureModel();
  ~VesselFeatureModel();
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  VesselFeatureModel(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

};

} // end namespace ivan

#endif
