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
// File: ivanVesselCenterlineAlgorithm.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanVesselCenterlineAlgorithm_h
#define __ivanVesselCenterlineAlgorithm_h

#include "itkFunctionBase.h"


namespace ivan
{
  
/** \class Vessel centerline interpolator
 *  \brief Base class for interpolating centerlines.
 *
 * This is the base class for interpolating centerlines consisting of sections 
 * stored in a container. The class interpolates only the section centerline point 
 * and normal but subclasses or specializations may interpolate the whole section 
 * as well. This is useful for example if we have a ray-casted section.
 *
 * \ingroup 
 */


template <class TCenterline> 
class ITK_EXPORT VesselCenterlineAlgorithm : public FunctionBase 
  <typename TCenterline::ConstPointer, typename TCenterline::Pointer>
{
public:

  typedef VesselCenterlineAlgorithm        Self;
  typedef FunctionBase                     Superclass;
  
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;

  typedef typename Superclass::InputType   InputType;
  typedef typename Superclass::OutputType  OutputType;
  
  typedef TCenterline                               CenterlineType;
  typedef typename CenterlineType::Pointer          CenterlinePointer;
  typedef typename CenterlineType::ConstPointer     CenterlineConstPointer;
  
  typedef typename CenterlineType::SectionType      SectionType;
  typedef typename CenterlineType::PointType        PointType;
        
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselCenterlineAlgorithm, FunctionBase );
  
  /* Evaluate a new section at the specified input position. */
  CenterlinePointer Evaluate( CenterlineConstPointer input ) = 0;
    
protected:
  
  VesselCenterlineAlgorithm() {}
  ~VesselCenterlineAlgorithm() {}
  
  //virtual void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:

  VesselCenterlineInterpolator( const Self & other );
  Self & operator = ( const Self & other );
  
protected:


};

} // end namespace ivan

#endif
