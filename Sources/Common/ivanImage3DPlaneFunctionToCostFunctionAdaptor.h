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
// File: ivanImage3DPlaneFunctionToCostFunctionAdaptor.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/06/12


#ifndef __ivanImage3DPlaneFunctionToCostFunctionAdaptor_h
#define __ivanImage3DPlaneFunctionToCostFunctionAdaptor_h

#include "ivanImageFunctionToCostFunctionAdaptor.h"

#include "itkVector.h"
#include "itkPoint.h"


namespace ivan
{
  
/** \class Image3DPlaneFunctionToCostFunctionAdaptor
 *  \brief Adaptor that adapts an ImageFunction so their values can be used as a CostFunction
 *
 * Image3DPlaneFunctionToCostFunctionAdaptor is an ImageFunctionToCostFunctionAdaptor that is restricted
 * to obtaining image values at a given 3D plane. The center of the plane and normal are assigned
 * previously. The parameters should be interpreted as a multiplier of an orthogonal 2D base, that
 * defines the plane, which is obtained from the normal of the plane. 
 *
 * This is intended to be used with ImageFunction values that are to be minimized or maximized in 
 * a given 3D plane.
 *
 * This works with single-valued images only.
 *
 * This class works for 3D images only.
 *
 */

template <class TImage>
class ITK_EXPORT Image3DPlaneFunctionToCostFunctionAdaptor : public ImageFunctionToCostFunctionAdaptor<TImage>
{
public:
  
  /** Standard class typedefs. */
  typedef Image3DPlaneFunctionToCostFunctionAdaptor<TImage>  Self;
  typedef ImageFunctionToCostFunctionAdaptor<TImage>         Superclass;
  typedef itk::SmartPointer<Self>                            Pointer;
  typedef itk::SmartPointer<const Self>                      ConstPointer;
  
  typedef typename Superclass::MeasureType    MeasureType;
  
  /** Image related typedefs. */
  typedef TImage                              ImageType;
  typedef typename ImageType::Pointer         ImagePointer;
    
  typedef typename ImageType::PointType       PointType;
      
  /** Image function related typedefs. */
  typedef typename Superclass::ImageFunctionType     ImageFunctionType;
  typedef typename Superclass::ImageFunctionPointer  ImageFunctionPointer;
  
  typedef itk::Vector<double, ImageType::ImageDimension>  VectorType;
  typedef itk::Matrix<double, ImageType::ImageDimension, 
    ImageType::ImageDimension-1>                          BaseMatrixType;
  
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( Image3DPlaneFunctionToCostFunctionAdaptor, ImageFunctionToCostFunctionAdaptor );
  
  /** Get the value at the desired location. The location is specified by two parameters (alpha and
    * beta). These are the parameter values for the equation of the plane, based on the center point 
    * and calculated base vectors on the plane (two vectors) as follows:
    * P(X) = P0(X) + alpha * e1(X) + beta * e2(X). */
  virtual MeasureType GetValue( const ParametersType & parameters ) const;
  
  virtual void GetDerivative( const ParametersType  &parameters, DerivativeType & derivative ) const
    { itkWarningMacro( "Derivative not implemented yet." ); }
    
  virtual unsigned int GetNumberOfParameters(void) const
    { return 2; }
    
  itkSetMacro( PlaneCenter, PointType & );
  itkGetConstMacro( PlaneCenter, const PointType & );
    
  virtual void SetPlaneNormal( const VectorType & normal );
  itkGetConstMacro( PlaneNormal, const VectorType & );
  
  itkGetConstMacro( PlaneFirstBaseVector, const VectorType & );
  itkGetConstMacro( PlaneSecondBaseVector, const VectorType & );
  
  /** Get the base vectors of the current plane as a matrix. Columns represent the base 
    * vectors for the plane (2) and rows the coordinates (3). */
  BaseMatrixType GetPlaneBaseMatrix() const;

protected:
  
  Image3DPlaneFunctionToCostFunctionAdaptor();
  virtual ~Image3DPlaneFunctionToCostFunctionAdaptor() {}
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;
      
private:
  
  Image3DPlaneFunctionToCostFunctionAdaptor(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
  
protected:
  
  PointType        m_PlaneCenter;
  
  VectorType       m_PlaneNormal;
  
  VectorType       m_PlaneFirstBaseVector;
  VectorType       m_PlaneSecondBaseVector;
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanImage3DPlaneFunctionToCostFunctionAdaptor.hxx"
#endif

#endif
