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
// File: ivanOrientedFluxMatrixBasedVesselnessImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/01/17

#ifndef __ivanOrientedFluxMatrixBasedVesselnessImageFunction_h
#define __ivanOrientedFluxMatrixBasedVesselnessImageFunction_h

#include "ivanMacros.h"
#include "ivanSphereGridBasedImageFunction.h"

#include "itkSymmetricSecondRankTensor.h"


namespace ivan
{
  
/**
 * \class OrientedFluxMatrixBasedVesselnessImageFunction
 * \brief Base class that computes vesselness by calculating oriented flux matrix.
 *
 * Base class that computes vesselness by calculating oriented flux matrix Q. This is an alternative
 * to the Hessian matrix. Different vesselness values can be obtained by eigenanalysis of such
 * a matrix. In order to use this matrix, subclasses must reimplement EvaluateVesselness().
 *
 * The vector field parameter corresponds to an image function that returns a vector for every 
 * point of the image. It can be a specific image function that performs the calculation, or can
 * be an interpolator that obtains values from a previously calculated vector image. This usually
 * corresponds to a gradient vector field.
 *
 * This class is not abstract, since we may be interested in using the public functions 
 * EvaluateFluxMatrix() but it should be treated as abstract in terms of obtaining a vesselness
 * response. By default it returns 0.0 and a warning. 
 *
 *
 * \par REFERENCES
 * Law, M.W.K. and Chung, A.C.S. "Three Dimensional Curvilinear Structure Detection Using Optimally 
 * Oriented Flux�, The 10th European Conf. on Computer Vision (ECCV'08), LNCS 5305:368�382 (2008).
 *
 */
template <class TInputImage, class TVectorField, class TOutput, class TCoordRep=double>
class ITK_EXPORT OrientedFluxMatrixBasedVesselnessImageFunction :
  public SphereGridBasedImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  typedef OrientedFluxMatrixBasedVesselnessImageFunction
    <TInputImage,TVectorField,TOutput,TCoordRep>           Self;
  typedef SphereGridBasedImageFunction
    <TInputImage,TOutput,TCoordRep>                        Superclass;
  
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
  
  typedef TVectorField                               VectorFieldType;
  typedef typename VectorFieldType::Pointer          VectorFieldPointer;
  
  typedef typename VectorFieldType::OutputType       VectorType;
  
  typedef itk::SymmetricSecondRankTensor<TCoordRep,
    ITKImageDimensionMacro( TInputImage ) >                   FluxMatrixType;
  typedef typename FluxMatrixType::EigenVectorsMatrixType     EigenVectorsMatrixType;
  typedef typename FluxMatrixType::EigenValuesArrayType       EigenValuesArrayType;
    
  /** Necessary for multi-scale compatibility. */
  typedef FluxMatrixType                                      TensorType;
      
  typedef itk::Vector<CoordRepType,
    ITKImageDimensionMacro( TInputImage ) >                   NormalVectorType;
  
  typedef std::vector<NormalVectorType>                       NormalVectorContainerType;
  typedef std::vector<PointType>                              PointContainerType;
      
public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( OrientedFluxMatrixBasedVesselnessImageFunction, SphereGridBasedImageFunction );
        
  /** Set the input image.
   * \warning this method caches BufferedRegion information.
   * If the BufferedRegion has changed, user must call
   * SetInputImage again to update cached values. */
  virtual void SetInputImage( const InputImageType * ptr );
  
  /** Set a user provided vector field. By default one if provided, but the user may
    * set one with the desired properties. */
  virtual void SetVectorField( VectorFieldType * vectorField );
  
  /** Get the vector field in order to set/get its properties. */
  VectorFieldType * GetVectorField()
    { return m_VectorField.GetPointer(); }
  const VectorFieldType * GetVectorField() const
    { return m_VectorField.GetPointer(); }

  /** Evalutate the function at specified point */
  virtual OutputType Evaluate( const PointType& point ) const;

  /** Evaluate the function at specified Index position */
  virtual OutputType EvaluateAtIndex( const IndexType & index ) const;

  /** Evaluate the function at specified ContinousIndex position */
  virtual OutputType EvaluateAtContinuousIndex( const ContinuousIndexType & index ) const;
  
  /** Evalutate the  in the given dimension at specified point. This method is made public to allow 
    * obtaining the flux matrix, even if no vesselness value is needed. */
  FluxMatrixType EvaluateFluxMatrix( const PointType& point ) const;

  /** Evaluate the function at specified Index position. This method is made public to allow 
    * obtaining the flux matrix, even if no vesselness value is needed. */
  FluxMatrixType EvaluateFluxMatrixAtIndex( const IndexType & index ) const;

  /** Evaluate the function at specified ContinousIndex position. This method is made public to allow 
    * obtaining the flux matrix, even if no vesselness value is needed. */
  FluxMatrixType EvaluateFluxMatrixAtContinuousIndex( const ContinuousIndexType & index ) const;  
  
protected:

  OrientedFluxMatrixBasedVesselnessImageFunction();
  virtual ~OrientedFluxMatrixBasedVesselnessImageFunction() {};
    
  /** Evaluate vesselness given flux matrix Q. This version simply converts the index to a continuous
    * index and calls EvaluateVesselnessAtContinuousIndex(). Subclasses may reimplement faster
    * approacher at center locations. */
  virtual OutputType EvaluateVesselnessAtIndex( const FluxMatrixType & fluxMatrix, const IndexType & index ) const;
    
  /** Evaluate vesselness given flux matrix Q. */
  virtual OutputType EvaluateVesselnessAtContinuousIndex( const FluxMatrixType & fluxMatrix,
    const ContinuousIndexType & cindex ) const;
    
  /** Evaluate vesselness given flux matrix Q. The default implementation translates the closest point
    * to a continuous index and performs the desired interpolation.  This must be reimplemented by subclasses. */
  virtual OutputType EvaluateVesselness( const FluxMatrixType & fluxMatrix, const PointType& point ) const
    { 
      itkWarningMacro( "EvaluateVesselness() is not implemented, returning zero." ); 
      return OutputType();
    }
          
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  OrientedFluxMatrixBasedVesselnessImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

protected:

  mutable NormalVectorContainerType    m_SphereNormals;
  
  VectorFieldPointer                   m_VectorField;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_OrientedFluxMatrixBasedVesselnessImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT OrientedFluxMatrixBasedVesselnessImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef OrientedFluxMatrixBasedVesselnessImageFunction< ITK_TEMPLATE_2 x > \
                                                  OrientedFluxMatrixBasedVesselnessImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanOrientedFluxMatrixBasedVesselnessImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanOrientedFluxMatrixBasedVesselnessImageFunction.hxx"
#endif

#endif
