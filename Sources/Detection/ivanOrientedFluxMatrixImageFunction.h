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
// File: ivanOrientedFluxMatrixImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2012/02/09

#ifndef __ivanOrientedFluxMatrixImageFunction_h
#define __ivanOrientedFluxMatrixImageFunction_h

#include "ivanMacros.h"
#include "ivanOrientedFluxMatrixBasedVesselnessImageFunction.h"

#include "itkNumericTraitsTensorPixel.h"


namespace ivan
{
  
/**
 * \class OrientedFluxMatrixImageFunction
 * \brief Class that computes oriented flux matrix given a vector field, usually a gradient field
 *
 * This class is as alternative to using OrientedFluxMatrixBasedVesselnessImageFunction::EvaluateFluxMatrix()
 * It inherits from that class in order to provide a version of Evaluate() that returns the flux matrix
 * so as to be used by filters which use this virtual interface. 
 *
 *
 * \par REFERENCES
 * Law, M.W.K. and Chung, A.C.S. "Three Dimensional Curvilinear Structure Detection Using Optimally 
 * Oriented Flux�, The 10th European Conf. on Computer Vision (ECCV'08), LNCS 5305:368�382 (2008).
 *
 */
template <class TInputImage, class TVectorField, class TCoordRep=double>
class ITK_EXPORT OrientedFluxMatrixImageFunction :
  public OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage, TVectorField,
    itk::SymmetricSecondRankTensor<TCoordRep, ITKImageDimensionMacro( TInputImage ) >, TCoordRep>
{
public:

  typedef OrientedFluxMatrixImageFunction
    <TInputImage,TVectorField,TCoordRep>                       Self;
  typedef OrientedFluxMatrixBasedVesselnessImageFunction
    <TInputImage,TVectorField,itk::SymmetricSecondRankTensor
    <TCoordRep, ITKImageDimensionMacro( TInputImage ) >,
    TCoordRep>                                                 Superclass;
  
  /** Smart pointer typedef support */
  typedef itk::SmartPointer<Self>            Pointer;
  typedef itk::SmartPointer<const Self>      ConstPointer;
  
  typedef TInputImage  InputImageType;
  typedef TCoordRep    CoordRepType;
  
  /** This is equivalent to FluxMatrixType. */
  typedef typename Superclass::OutputType                 OutputType;
  
  typedef typename Superclass::InputPixelType             InputPixelType;
  typedef typename Superclass::IndexType                  IndexType;
  typedef typename Superclass::ContinuousIndexType        ContinuousIndexType;
  typedef typename Superclass::PointType                  PointType;
    
  typedef typename Superclass::SphereSourceType           SphereSourceType;
  typedef typename Superclass::SphereSourcePointer        SphereSourcePointer;
  typedef typename Superclass::SphereType                 SphereType;
  typedef typename Superclass::SpherePointer              SpherePointer;
  
  typedef TVectorField                                    VectorFieldType;
  typedef typename VectorFieldType::Pointer               VectorFieldPointer;
  
  typedef typename VectorFieldType::OutputType            VectorType;
  
  typedef typename Superclass::FluxMatrixType             FluxMatrixType;
  typedef typename Superclass::EigenVectorsMatrixType     EigenVectorsMatrixType;
  typedef typename Superclass::EigenValuesArrayType       EigenValuesArrayType;
    
  /** Necessary for multi-scale compatibility. */
  typedef FluxMatrixType                                  TensorType;
      
  typedef typename Superclass::NormalVectorType           NormalVectorType;
  typedef typename Superclass::NormalVectorContainerType  NormalVectorContainerType;
  typedef typename Superclass::PointContainerType         PointContainerType;
      
public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( OrientedFluxMatrixImageFunction, OrientedFluxMatrixBasedVesselnessImageFunction );
        
  /** Evalutate the function at specified point */
  virtual OutputType Evaluate( const PointType& point ) const
    { return this->EvaluateFluxMatrix( point ); }

  /** Evaluate the function at specified Index position */
  virtual OutputType EvaluateAtIndex( const IndexType & index ) const
    { return this->EvaluateFluxMatrixAtIndex( index ); }

  /** Evaluate the function at specified ContinousIndex position */
  virtual OutputType EvaluateAtContinuousIndex( const ContinuousIndexType & cindex ) const
    { return this->EvaluateFluxMatrixAtContinuousIndex( cindex ); }
    
protected:

  OrientedFluxMatrixImageFunction() {}
  virtual ~OrientedFluxMatrixImageFunction() {};

private:
  
  OrientedFluxMatrixImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

protected:

};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_OrientedFluxMatrixImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT OrientedFluxMatrixImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef OrientedFluxMatrixImageFunction< ITK_TEMPLATE_2 x > \
                                                  OrientedFluxMatrixImageFunction##y; } \
  }

#endif
