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
// File: ivanLocalCurveMetrics.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2009/02/06


#ifndef __ivanLocalCurveMetrics_h
#define __ivanLocalCurveMetrics_h

#include "ivanVesselSectionStruct.h"


namespace ivan
{
  
/** \class LocalCurveMetrics
 *  \brief Standard local metrics for the centerline curve at every point.
 *
 * This struct-like class is intended to be used as template parameter for
 * the vessel section.
 *
 * \ingroup 
 */

template <unsigned int VDimension>
class ITK_EXPORT LocalCurveMetrics : public VesselSectionStruct
{
public:

  typedef LocalCurveMetrics    Self;
  typedef VesselSectionStruct  Superclass;
  
  typedef itk::Matrix<double, VDimension, VDimension>  MatrixType;
  typedef itk::Vector<double, VDimension>              VectorType;
  
  typedef vnl_matrix_fixed<double, VDimension, VDimension>  VnlMatrixType;
  typedef vnl_vector_fixed<double, VDimension>              VnlVectorType;
  
public:
  
  /** This object is exceptionally created on the stack. */
  LocalCurveMetrics();
  ~LocalCurveMetrics();
  
  /** Measurements are struct-like classes that need to define copy ctors. */
  LocalCurveMetrics( const Self & other );
  
  /** Measurements are struct-like classes that need to define assignment operators. */
  Self & operator = ( const Self & other );
    
  void SetReferenceFrame( const MatrixType & rf )
    { m_ReferenceFrame = rf );
  itkGetMacro( ReferenceFrame, MatrixType & );
  itkGetConstMacro( ReferenceFrame, MatrixType & );
  
  /** Set/Get the tangent to the curve. This IS the section normal!!!! */
  void SetTangent( const VectorType & tangent );  
  VectorType GetTangent() const;
  
  /** Set/Get the normal to the curve. Not the section normal!!!! */
  void SetNormal( const VectorType & normal );
  VectorType GetNormal() const;
  
  /** Calculate the binormal to the curve. */  
  void CalculateBinormal();
  
  /** Get the binormal vector. If not calculated call CalculateBinormal()
    * after setting the tangent and normal vectors. */
  VectorType GetBinormal() const;
  
  itkSetMacro( Curvature, double );
  itkGetConstMacro( Curvature, double );
    
  itkSetMacro( Torsion, double );
  itkGetConstMacro( Torsion, double );
    
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;
  
private:

  /** Matrix that stores the Frenet reference frame. This stores by columns the tangent
    * and normal vector (2D), the tangent, normal and binormal vector (3D) and so on. */
  MatrixType  m_ReferenceFrame;
  
  /** Local curvature. */
  double      m_Curvature;
  
  /** Local torsion. */
  double      m_Torsion;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "itkLocalCurveMetrics.hxx"
#endif

#endif
