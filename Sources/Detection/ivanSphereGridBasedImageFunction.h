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
// File: ivanSphereGridBasedImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/01/17

#ifndef __ivanSphereGridBasedImageFunction_h
#define __ivanSphereGridBasedImageFunction_h


#include "itkImageFunction.h"
#include "ivanRegularSphereMeshSource2.h"
#include "ivanDiscreteHessianGaussianImageFunction.h"
#include "itkLinearInterpolateImageFunction.h"


namespace ivan
{
  
/**
 * \class SphereGridBasedImageFunction
 * \brief Base class for image functions that compute flux on a sphere around the given point.
 *
 * Base class for image functions that compute flux on a sphere around the given point.
 *
 * This base class performs partitioning of space in polar coordinates. Subclasses must define
 * how the flux is calculated specifically.
 *
 * TODO: encapsulate sphere calculations in a struct. Pre-compute normals for the sphere and
 * then some other calculations.
 *
 */
template <class TInputImage, class TOutput, class TCoordRep=double>
class ITK_EXPORT SphereGridBasedImageFunction :
  public itk::ImageFunction<TInputImage,TOutput,TCoordRep>
{
public:

  typedef SphereGridBasedImageFunction
    <TInputImage,TOutput,TCoordRep>   Self;
  typedef itk::ImageFunction
    <TInputImage,TOutput,TCoordRep>   Superclass;
  
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
    
  typedef ivan::RegularSphereMeshSource2<double>     SphereSourceType;
  typedef typename SphereSourceType::Pointer         SphereSourcePointer;
  typedef typename SphereSourceType::OutputMeshType  SphereType;
  typedef typename SphereType::Pointer               SpherePointer;
        
public:

  /** Method for creation through the object factory */
  itkNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro( SphereGridBasedImageFunction, itk::ImageFunction );
  
  itkSetMacro( Radius, double );
  itkGetConstMacro( Radius, double );

  itkSetMacro( Decimation, unsigned int );
  itkGetConstMacro( Decimation, unsigned int );
  
  /** Set/Get the radial resolution. If the radial resolution is 1, we only have points
    * on the boundary of the sphere. It does not include the center point. */
  itkSetMacro( RadialResolution, unsigned int );
  itkGetConstMacro( RadialResolution, unsigned int );
          
  /** Set the input image. */
  virtual void SetInputImage( const InputImageType * ptr );

  /** Initialize the flux-based function. Call this method before evaluating the function.
    * This method MUST be called after any changes to function parameters. */
  virtual void Initialize();
  
  /** Evalutate the  in the given dimension at specified point */
  virtual OutputType Evaluate( const PointType& point ) const;

  /** Evaluate the function at specified Index position */
  virtual OutputType EvaluateAtIndex( const IndexType & index ) const;

  /** Evaluate the function at specified ContinousIndex position */
  virtual OutputType EvaluateAtContinuousIndex( const ContinuousIndexType & index ) const;
  
protected:

  SphereGridBasedImageFunction();
  virtual ~SphereGridBasedImageFunction() {};
    
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:
  
  SphereGridBasedImageFunction( const Self& ); // purposely not implemented
  void operator=( const Self& ); // purposely not implemented

protected:

  /** Sphere mesh that will be used for sampling via ray-tracing from the sphere centre. */
  SpherePointer          m_Sphere;
  
  /** Radius of the sphere to be sampled. */
  double                 m_Radius;
  
  /** Decimation of the sphere to be sampled. Zero means a simple icosahedron whose vertices
    * have the sphere radius. Then, new vertices are generated by decimation of edges the
    * given number of times. */
  unsigned int           m_Decimation;
  
  /** Number of radial samples used for the sphere. This includes the center point. */
  unsigned int           m_RadialResolution;
};

} // namespace ivan

// Define instantiation macro for this template.
#define ITK_TEMPLATE_SphereGridBasedImageFunction(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT SphereGridBasedImageFunction< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef SphereGridBasedImageFunction< ITK_TEMPLATE_2 x > \
                                                  SphereGridBasedImageFunction##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanSphereGridBasedImageFunction+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanSphereGridBasedImageFunction.hxx"
#endif

#endif
