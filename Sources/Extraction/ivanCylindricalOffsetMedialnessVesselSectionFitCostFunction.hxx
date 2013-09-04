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
// File: ivanCylindricalOffsetMedialnessVesselSectionFitCostFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2011/11/05

#ifndef __ivanCylindricalOffsetMedialnessVesselSectionFitCostFunction_hxx
#define __ivanCylindricalOffsetMedialnessVesselSectionFitCostFunction_hxx

#include "ivanCylindricalOffsetMedialnessVesselSectionFitCostFunction.h"

#include <vnl/vnl_vector_fixed.h>


namespace ivan
{

template <class TImage>
CylindricalOffsetMedialnessVesselSectionFitCostFunction<TImage>
::CylindricalOffsetMedialnessVesselSectionFitCostFunction() :
  m_MaxRadius( 10.0 )
{
	this->m_MedialnessFunction = MedialnessFunctionType::New();
	this->m_MedialnessFunction->AutoComputeSectionNormalOff();
	  
	this->m_SectionCenter.Fill( 0.0 );
}


template <class TImage>
CylindricalOffsetMedialnessVesselSectionFitCostFunction<TImage>
::~CylindricalOffsetMedialnessVesselSectionFitCostFunction()
{

}


template <class TImage>
void
CylindricalOffsetMedialnessVesselSectionFitCostFunction<TImage>
::Initialize()
{
  this->m_MedialnessFunction->SetInputImage( this->m_Image );
  this->m_MedialnessFunction->AutoComputeSectionNormalOff();
  this->m_MedialnessFunction->Initialize();
}



template <class TImage>
typename CylindricalOffsetMedialnessVesselSectionFitCostFunction<TImage>::MeasureType
CylindricalOffsetMedialnessVesselSectionFitCostFunction<TImage>
::GetValue( const ParametersType & parameters ) const
{
  typename MedialnessFunctionType::VectorType sectionNormal;
      
  sectionNormal[0] = parameters[0];
  sectionNormal[1] = parameters[1];
  //sectionNormal[2] = parameters[2];
  //sectionNormal.Normalize();
  //double radius = parameters[3];  
  //double lagrangeMult1 = parameters[4];
 
  // Restrict the normal to be a unit vector
  if( vnl_math_abs( parameters[0] ) > 1.0 )
    return 0.0;
  else if( vnl_math_abs( parameters[1] ) > 1.0 )
    return 0.0;
    
  sectionNormal[2] = 1.0 - sectionNormal[0] * sectionNormal[0] - sectionNormal[1] * sectionNormal[1];
  
  if( sectionNormal[2] < 0.0 ) // due to rounding errors
    sectionNormal[2] = 0.0;
  else
    sectionNormal[2] = sqrt( sectionNormal[2] );
    
  double radius = parameters[2];
  
#ifdef _DEBUG
  std::cout << "Parameters: " << parameters[0] << " " << parameters[1] << " " 
    << parameters[2] << std::endl;
#endif
 
  // Do not allow the radius to be bigger than the image region or to be negative
	
	if( radius <= 0.0 )
		return itk::NumericTraits<MeasureType>::Zero;

	// Do not allow the radius to be larger than half the minimum size in all directions
	
	if( radius >= this->m_MaxRadius )
		return itk::NumericTraits<MeasureType>::Zero;

	// Do not allow the radius to be smaller than half the minimum spacing in all directions

	const SpacingType &spacing = this->m_Image->GetSpacing();
	double maxSpacing = 0.0;

	for( unsigned int i=0; i<ImageDimension; ++i )
	{
		if( spacing[i] > maxSpacing )
			maxSpacing = spacing[i];
	}

	if( radius <= maxSpacing * 0.5 )
		return itk::NumericTraits<MeasureType>::Zero;
  
  this->m_MedialnessFunction->SetRadius( radius );  
  this->m_MedialnessFunction->SetSectionNormal( sectionNormal );
  
  // Here the medialness function is always evaluated at the same point. However, the normal plane and radius
  // vary and so does the computation of the medialness at the offset locations
  double medialness = this->m_MedialnessFunction->Evaluate( this->m_SectionCenter );
  
  // Now use a Lagrangian multiplier to implement the unit normal constraint
  //double constraint1 = lagrangeMult1 * ( 1.0 - sectionNormal[0] * sectionNormal[0] - 
    //sectionNormal[1] * sectionNormal[1] - sectionNormal[2] * sectionNormal[2] );
  
  // This assumes that the medialness is MAXIMIZED (not minimized)
  return medialness; // + constraint1;
}


template <class TImage>
void
CylindricalOffsetMedialnessVesselSectionFitCostFunction<TImage>
::PrintSelf( std::ostream& os, itk::Indent indent) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "SectionCenter: " << this->m_SectionCenter << std::endl;
  os << indent << "MaxRadius: " << this->m_MaxRadius << std::endl;
  os << indent << "MedialnessFunction: " << this->m_MedialnessFunction.GetPointer() << std::endl;
  
  this->m_MedialnessFunction->Print( os, indent.GetNextIndent() );
}

} // end namespace ivan

#endif
