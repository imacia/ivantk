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
// File: ivanOptimallyOrientedFluxVesselnessImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/01/18

#ifndef __ivanOptimallyOrientedFluxVesselnessImageFunction_h
#define __ivanOptimallyOrientedFluxVesselnessImageFunction_h


#include "ivanOrientedFluxMatrixBasedVesselnessImageFunction.h"


namespace ivan
{

template <class TOutput = double, unsigned int VDimension = 3>
class ITK_EXPORT GeometricMeanTwoNegativeEigenvalueFunctor
{
public:

  typedef itk::SymmetricSecondRankTensor<TOutput,VDimension>   MatrixType;
  typedef typename MatrixType::EigenVectorsMatrixType          EigenVectorsMatrixType;
  typedef typename MatrixType::EigenValuesArrayType            EigenValuesArrayType;
  
  typedef TOutput    OutputType;
  
public:

  OutputType Evaluate( const EigenValuesArrayType & eigenValues, 
    const EigenVectorsMatrixType & eigenVectors ) const
    { 
      // WARNING: VERIFY THAT THIS ORDER OF EIGENVALUES IS CORRECT
      if( eigenValues[0] > 0.0 || eigenValues[1] > 0.0 )
        return 0.0;
      else
        return vcl_sqrt( vcl_abs( eigenValues[0] * eigenValues[1] ) );
    }
};

  
/**
 * \class OptimallyOrientedFluxVesselnessImageFunction
 * \brief Computes vesselness by calculating optimally oriented flux.
 *
 * This class computes vesselness using optimally oriented flux. The vesselness value is built
 * by using some function based on the eigenvalues and eigenvectors of the flux matrix. This
 * calculation is made generic by providing a template parameter TEigenvalueFunctor, which is
 * a functor that calculates the final template parameters based on the eigenvalues and 
 * eigenvectors. Several filters may be designed by changing this parameter. The output type
 * of the functor must correspond to the output type of this image function.
 *
 * \par REFERENCES
 * Law, M.W.K. and Chung, A.C.S. "Three Dimensional Curvilinear Structure Detection Using Optimally 
 * Oriented Flux�, The 10th European Conf. on Computer Vision (ECCV'08), LNCS 5305:368�382 (2008).
 *
 */
template <class TInputImage, class TVectorField, class TEigenvalueFunctor, class TOutput, class TCoordRep=double>
class ITK_EXPORT OptimallyOrientedFluxVesselnessImageFunction :
  public OrientedFluxMatrixBasedVesselnessImageFunction<TInputImage,TVectorField,TOutput,TCoordRep>
{
public:

  typedef OptimallyOrientedFluxVesselnessImageFunction
    <TInputImage,TVectorField,TEigenvalueFunctor,TOutput,TCoordRep>   Self;
  typedef OrientedFluxMatrixBasedVesselnessImageFunction
    <TInputImage,TVectorField,TOutput,TCoordRep>                      Superclass;
  
  /** Smart pointer typedef support */
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;
  
  typedef TInputImage  InputImageType;
  typedef TOutput      OutputType;
  typedef TCoordRep    CoordRepType;
  
  typedef typename Superclass::InputPixelType        InputPixelType;
  typedef typename Superclass::IndexType             IndexType;
  typedef typename Superclass::ContinuousIndexType   ContinuousIndexType;
  typedef typename Superclass::PointType             PointType;
    
  typedef typename Superclass::SphereSourceType      SphereSourceType;
  typedef typename Superclass::SphereSourcePointer   SphereSourcePointer;
  typedef typename Superclass::SphereType            SphereType;
  typedef typename Superclass::SpherePointer         SpherePointer;
  
  typedef typename Superclass::VectorFieldType       VectorFieldType;
  typedef typename Superclass::VectorFieldPointer    VectorFieldPointer;
  
  typedef typename Superclass::VectorType            VectorType;
  typedef typename Superclass::NormalVectorType      NormalVectorType;
  
  typedef typename Superclass::FluxMatrixType              FluxMatrixType;
  typedef typename Superclass::EigenVectorsMatrixType      EigenVectorsMatrixType;
  typedef typename Superclass::EigenValuesArrayType        EigenValuesArrayType;
     
  typedef typename Superclass::NormalVectorContainerType   NormalVectorContainerType;
  typedef typename Superclass::PointContainerType          PointContainerType;
  
  typedef TEigenvalueFunctor                               EigenvalueFunctorType;
    
public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( OptimallyOrientedFluxVesselnessImageFunction, 
    OrientedFluxMatrixBasedVesselnessImageFunction );
    
  /** Set/Get the vesselness functor. This is created by default, but may be useful to access it
    * to set/get properties. We do not provide a Set method since the functor is static for
    * simplicity. */
  EigenvalueFunctorType * GetEigenvalueFunctor()
    { return &this->m_EigenvalueFunctor; }
  const EigenvalueFunctorType * GetEigenvalueFunctor() const
    { return &this->m_EigenvalueFunctor; }
   
protected:

  OptimallyOrientedFluxVesselnessImageFunction();
  virtual ~OptimallyOrientedFluxVesselnessImageFunction() {};
    
  /** Evaluate vesselness given flux matrix Q. The default implementation translates the closest point
    * to a continuous index and performs the desired interpolation.  This must be reimplemented by subclasses. */
  virtual OutputType EvaluateVesselness( const FluxMatrixType & fluxMatrix, const PointType& point ) const;
     
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  OptimallyOrientedFluxVesselnessImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

protected:

  EigenvalueFunctorType     m_EigenvalueFunctor;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_OptimallyOrientedFluxVesselnessImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT OptimallyOrientedFluxVesselnessImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef OptimallyOrientedFluxVesselnessImageFunction< ITK_TEMPLATE_2 x > \
                                                  OptimallyOrientedFluxVesselnessImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanOptimallyOrientedFluxVesselnessImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanOptimallyOrientedFluxVesselnessImageFunction.hxx"
#endif

#endif
