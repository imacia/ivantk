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
// File: ivanLocalCurveMetrics.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanLocalCurveMetrics_hxx
#define __ivanLocalCurveMetrics_hxx

#include "ivanLocalCurveMetrics.h"


namespace ivan
{

template <unsigned int VDimension>
LocalCurveMetrics<VDimension>::LocalCurveMetrics() :
  m_Curvature(0.0),
  m_Torsion(0.0)
{
  m_ReferenceFrame.Fill(0.0);
}


template <unsigned int VDimension>
LocalCurveMetrics<VDimension>::~LocalCurveMetrics()
{

}


template <unsigned int VDimension>
LocalCurveMetrics<VDimension>::LocalCurveMetrics( const Self & other )
{
  this->m_ReferenceFrame = other->GetReferenceFrame();
  this->m_Curvature      = other->GetCurvature();
  this->m_Torsion        = other->GetTorsion();
}


template <unsigned int VDimension>
LocalCurveMetrics<VDimension> & 
LocalCurveMetrics<VDimension>::operator =( const Self & other )
{
  this->m_ReferenceFrame = other->GetReferenceFrame();
  this->m_Curvature      = other->GetCurvature();
  this->m_Torsion        = other->GetTorsion();
  
  return (*this);
}


void LocalCurveMetrics<VDimension>::SetTangent( const VectorType & tangent )
{ 
  for( unsigned int i=0; i<VDimension; ++i )
    this->m_ReferenceFrame[i][0] = tangent[i];
}


LocalCurveMetrics<VDimension>::VectorType 
LocalCurveMetrics<VDimension>::GetTangent() const
{
  VectorType tangent;
  for( unsigned int i=0; i<VDimension; ++i )
    tangent[i] = this->m_ReferenceFrame[i][0];
  
  return tangent;
}


void LocalCurveMetrics<VDimension>::SetNormal( const VectorType & normal )
{ 
  for( unsigned int i=0; i<VDimension; ++i )
    this->m_ReferenceFrame[i][0] = tangent[i];
}


LocalCurveMetrics<VDimension>::VectorType
LocalCurveMetrics<VDimension>::GetNormal() const
{
  VectorType normal;
  for( unsigned int i=0; i<VDimension; ++i )
    normal[i] = m_ReferenceFrame[i][1];
  
  return normal;
}
  

void LocalCurveMetrics<VDimension>::CalculateBinormal()
{
  if( VDimension < 3 )
    return;
  
  // Calculate cross-product of first two columns and store in third
  this->m_ReferenceFrame.GetVnlMatrix().set_column( 2, 
    vnl_cross_3d( 
      this->m_ReferenceFrame.GetVnlMatrix().get_column(0),
      this->m_ReferenceFrame.GetVnlMatrix().get_column(1),      
    )
  );
}


LocalCurveMetrics<VDimension>::VectorType
LocalCurveMetrics<VDimension>::GetBinormal() const
{
  assert( VDimension >= 3 );
  VectorType binormal;
  for( unsigned int i=0; i<VDimension; ++i )
    binormal[i] = m_ReferenceFrame[i][2];
  
  return binormal;
}


template <unsigned int VDimension>
void LocalCurveMetrics<VDimension>::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "ReferenceFrame: " << m_ReferenceFrame << std::endl;
  os << indent << "Curvature: " << m_Curvature << std::endl;
  os << indent << "Torsion: " << m_Torsion << std::endl;  
}

} // end namespace ivan

#endif // __ivanLocalCurveMetrics_hxx
