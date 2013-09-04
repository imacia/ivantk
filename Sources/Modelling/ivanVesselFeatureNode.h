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
// File: ivanVesselFeatureNode.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description : 
// Date: 2009/02/06


#ifndef __ivanVesselFeatureNode_h
#define __ivanVesselFeatureNode_h

#include "ivanVesselNode.h"
#include "ivanVesselRegion.h"
#include "ivanVesselFeatureModel.h"


namespace ivan
{
  
/** \class VesselFeatureNode
 *  \brief Base class for vessel nodes for features such as stenoses, aneurysms, strokes, etc.
 *
 * This node type is used to hold models of vascular features or accidents,
 * or other areas of diagnostic interest, such as stenosis, aneurysms, 
 * strokes, regions feeding tumours, etc.  The feature model is stored
 * as an aggregate instead of using inheritance, in order to decouple it 
 * from the node type, and use always the same node type for this type of
 * objects, since this way it can be very open. Due to the visitor mechanism
 * we are not interested in templating this type of node either.
 *
 * VesselFeature nodes have FeatureRegion that describes the affected area.
 * This node type can be assigned as a child of several branch nodes,
 * since the FeatureRegion may affect several branches.
 *
 * \ingroup 
 */
 
class ITK_EXPORT VesselFeatureNode : public VesselNode
{

public:

  /** Standard class typedefs. */
  typedef VesselFeatureNode        Self;
  typedef VesselNode               Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;
       
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkNodeTypeMacro( VesselFeatureNode, VesselNode );
  
  /** Set/Get region corresponding to this feature. This is the area
    * affected by the feature, in one of several branches, in terms of 
    * centerline extent for each affected branch. */
  void SetFeatureRegion( const VesselRegion & region )
    { m_FeatureRegion = region; }
  const VesselRegion & GetFeatureRegion() const
    { return m_FeatureRegion; }
  
  /** Set/Get text describing the feature. */
  itkSetMacro( Description, std::string & );
  itkGetConstMacro( Description, const std::string & );
  
  /** Get the type of feature model we are using. */
  std::string GetFeatureModelType() const
    { 
      if( m_FeatureModel.IsNotNull() )
        return m_FeatureModel->GetFeatureType();
      else
        return "None";
    } 
  
  void SetFeatureModel( VesselFeatureModel *featureModel )
    { m_FeatureModel = featureModel; }
  VesselFeatureModel * GetFeatureModel()
    { return m_FeatureModel; }
  const VesselFeatureModel * GetFeatureModel() const
    { return m_FeatureModel; }
      
protected:
  
  VesselFeatureNode();
  ~VesselFeatureNode();
    
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  VesselFeatureNode(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  /** Optional feature model. */
  VesselFeatureModel::Pointer   m_FeatureModel;

  /** Describes the affected area that may consist of one or several partially
    * or totally affected branches. */
  VesselRegion    m_FeatureRegion;
  
  /** Description of the feature. This may be used for diagnostic annotations. */
  std::string     m_Description;
};

} // end namespace ivan

#endif
