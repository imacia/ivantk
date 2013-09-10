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
// File: ivanLinearVesselCenterlineInterpolator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanLinearVesselCenterlineInterpolator_h
#define __ivanLinearVesselCenterlineInterpolator_h

#include "ivanVesselCenterlineAlgorithm.h"


namespace ivan
{

/** \class LinearVesselCenterlineInterpolator
 *  \brief Linear interpolation of vessel sections.
 *
 * This class interpolates centerlines consisting of sections stored in a container. 
 * The class interpolates only the section centerline point and normal but subclasses 
 * or specializations may interpolate the whole section as well. This is useful for 
 * example if we have a ray-casted section.
 *
 * \ingroup
 *
 */


template <class TCenterline> 
class ITK_EXPORT LinearVesselCenterlineInterpolator 
  : public VesselCenterlineAlgorithm<TCenterline>
{
public:

  typedef LinearVesselCenterlineInterpolator   Self;
  typedef VesselCenterlineAlgorithm            Superclass;
  
  typedef itk::SmartPointer<Self>                   Pointer;
  typedef itk::SmartPointer<const Self>             ConstPointer;

  typedef typename Superclass::InputType       InputType;
  typedef typename Superclass::OutputType      OutputType;
  
  typedef TCenterline                               CenterlineType;
  typedef typename CenterlineType::Pointer          CenterlinePointer;
  typedef typename CenterlineType::ConstPointer     CenterlineConstPointer;
  
  typedef typename CenterlineType::SectionType      SectionType;
  typedef typename CenterlineType::PointType        PointType;

public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( LinearVesselCenterlineInterpolator, VesselCenterlineAlgorithm );
  
  /* Evaluate a new section at the specified input position. */
  CenterlinePointer Evaluate( CenterlineConstPointer input ) = 0;
    
protected:
  
  LinearVesselCenterlineInterpolator();
  ~LinearVesselCenterlineInterpolator();
  
private:

  LinearVesselCenterlineInterpolator( const Self & other );
  Self & operator = ( const Self & other );
  
protected:

  
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanLinearVesselCenterlineInterpolator.hxx"
#endif

#endif
