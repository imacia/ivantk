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
// File: ivanDiscreteHessianGaussianImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: Modified version of itkDiscreteHessianGaussianImageFunction, with support for gamma-normalized derivatives
// Date: 2010/05/19

#ifndef __itkDiscreteHessianGaussianImageFunction_hxx
#define __itkDiscreteHessianGaussianImageFunction_hxx

#include "ivanDiscreteHessianGaussianImageFunction.h"

#include "itkNeighborhoodOperatorImageFilter.h"

namespace ivan
{

/** Set the Input Image */
template< class TInputImage, class TOutput >
DiscreteHessianGaussianImageFunction< TInputImage, TOutput >
::DiscreteHessianGaussianImageFunction():
  m_MaximumError(0.005),
  m_MaximumKernelWidth(30),
  m_NormalizeAcrossScale(true),
  m_Gamma(1.0),
  m_UseImageSpacing(true),
  m_InterpolationMode(NearestNeighbourInterpolation)
{
  m_Variance.Fill(1.0);
  m_OperatorImageFunction = OperatorImageFunctionType::New();
}

/** Print self method */
template< class TInputImage, class TOutput >
void
DiscreteHessianGaussianImageFunction< TInputImage, TOutput >
::PrintSelf(std::ostream & os, itk::Indent indent) const
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "UseImageSpacing: " << m_UseImageSpacing << std::endl;
  os << indent << "NormalizeAcrossScale: " << m_NormalizeAcrossScale << std::endl;
  os << indent << "Gamma: " << m_Gamma << std::endl;
  os << indent << "Variance: " << m_Variance << std::endl;
  os << indent << "MaximumError: " << m_MaximumError << std::endl;
  os << indent << "MaximumKernelWidth: " << m_MaximumKernelWidth << std::endl;
  os << indent << "OperatorArray: " << m_OperatorArray << std::endl;
  os << indent << "KernelArray: " << m_KernelArray << std::endl;
  os << indent << "OperatorImageFunction: " << m_OperatorImageFunction << std::endl;
  os << indent << "InterpolationMode: " << m_InterpolationMode << std::endl;
}

/** Set the input image */
template< class TInputImage, class TOutput >
void
DiscreteHessianGaussianImageFunction< TInputImage, TOutput >
::SetInputImage(const InputImageType *ptr)
{
  Superclass::SetInputImage(ptr);
  m_OperatorImageFunction->SetInputImage(ptr);
}

/** Recompute the gaussian kernel used to evaluate indexes
 *  This should use a fastest Derivative Gaussian operator */
template< class TInputImage, class TOutput >
void
DiscreteHessianGaussianImageFunction< TInputImage, TOutput >
::RecomputeGaussianKernel()
{
  /* Create 3*N operators (N=ImageDimension) where the
   * first N are zero-order, the second N are first-order
   * and the third N are second order */
  unsigned int idx;
  unsigned int maxRadius = 0;

  for ( unsigned int direction = 0; direction <
        itkGetStaticConstMacro(ImageDimension); direction++ )
    {
    for ( unsigned int order = 0; order <= 2; ++order )
      {
      idx = itkGetStaticConstMacro(ImageDimension) * order + direction;
      m_OperatorArray[idx].SetDirection(direction);
      m_OperatorArray[idx].SetMaximumKernelWidth(m_MaximumKernelWidth);
      m_OperatorArray[idx].SetMaximumError(m_MaximumError);

      if ( ( m_UseImageSpacing == true ) && ( this->GetInputImage() ) )
        {
        if ( this->GetInputImage()->GetSpacing()[direction] == 0.0 )
          {
          itkExceptionMacro(<< "Pixel spacing cannot be zero");
          }
        else
          {
          m_OperatorArray[idx].SetSpacing(this->GetInputImage()->GetSpacing()[direction]);
          }
        }

      // NOTE: GaussianDerivativeOperator modifies the variance when
      // setting image spacing
      m_OperatorArray[idx].SetVariance(m_Variance[direction]);
      m_OperatorArray[idx].SetOrder(order);
      m_OperatorArray[idx].SetNormalizeAcrossScale(m_NormalizeAcrossScale);
      m_OperatorArray[idx].SetGamma(m_Gamma); // will only take effect when m_NormalizeAcrossScale is true
      m_OperatorArray[idx].CreateDirectional();

      // Check for maximum radius
      for ( unsigned int i = 0; i < itkGetStaticConstMacro(ImageDimension); ++i )
        {
        if ( m_OperatorArray[idx].GetRadius()[i] > maxRadius )
          {
          maxRadius = m_OperatorArray[idx].GetRadius()[i];
          }
        }
      }
    }

  // Now precompute the N-dimensional kernel. This fastest as we don't
  // have to perform N convolutions for each point we calculate but
  // only one.

  typedef itk::Image< TOutput, itkGetStaticConstMacro(ImageDimension) > KernelImageType;
  typename KernelImageType::Pointer kernelImage = KernelImageType::New();

  typedef typename KernelImageType::RegionType RegionType;
  RegionType region;

  typename RegionType::SizeType size;
  size.Fill(4 * maxRadius + 1);
  region.SetSize(size);

  kernelImage->SetRegions(region);
  kernelImage->Allocate();
  kernelImage->FillBuffer(itk::NumericTraits< TOutput >::Zero);

  // Initially the kernel image will be an impulse at the center
  typename KernelImageType::IndexType centerIndex;
  centerIndex.Fill(2 * maxRadius);   // include also boundaries

  // Create an image region to be used later that does not include boundaries
  RegionType kernelRegion;
  size.Fill(2 * maxRadius + 1);
  typename RegionType::IndexType origin;
  origin.Fill(maxRadius);
  kernelRegion.SetSize(size);
  kernelRegion.SetIndex(origin);

  // Now create an image filter to perform sucessive convolutions
  typedef itk::NeighborhoodOperatorImageFilter< KernelImageType, KernelImageType >
  NeighborhoodFilterType;
  typename NeighborhoodFilterType::Pointer convolutionFilter = NeighborhoodFilterType::New();

  // Array that stores the current order for each direction
  typedef itk::FixedArray< unsigned int, itkGetStaticConstMacro(ImageDimension) > OrderArrayType;
  OrderArrayType orderArray;

  // Precalculate compound derivative kernels (n-dimensional)
  // The order of calculation in the 3D case is: dxx, dxy, dxz, dyy,
  // dyz, dzz

  unsigned int opidx; // current operator index in m_OperatorArray
  unsigned int kernelidx = 0;

  for ( unsigned int i = 0; i < itkGetStaticConstMacro(ImageDimension); ++i )
    {
    for ( unsigned int j = i; j < itkGetStaticConstMacro(ImageDimension); ++j )
      {
      orderArray.Fill(0);
      ++orderArray[i];
      ++orderArray[j];

      // Reset kernel image
      kernelImage->FillBuffer(itk::NumericTraits< TOutput >::Zero);
      kernelImage->SetPixel(centerIndex, itk::NumericTraits< TOutput >::One);

      for ( unsigned int direction = 0; direction < itkGetStaticConstMacro(ImageDimension); ++direction )
        {
        opidx = itkGetStaticConstMacro(ImageDimension) * orderArray[direction] + direction;
        convolutionFilter->SetInput(kernelImage);
        convolutionFilter->SetOperator(m_OperatorArray[opidx]);
        convolutionFilter->Update();
        kernelImage = convolutionFilter->GetOutput();
        kernelImage->DisconnectPipeline();
        }

      // Set the size of the current kernel
      m_KernelArray[kernelidx].SetRadius(maxRadius);

      // Copy kernel image to neighborhood. Do not copy boundaries.
      itk::ImageRegionConstIterator< KernelImageType > it(kernelImage, kernelRegion);
      it.GoToBegin();
      idx = 0;

      while ( !it.IsAtEnd() )
        {
        m_KernelArray[kernelidx][idx] = it.Get();
        ++idx;
        ++it;
        }
      kernelidx++;
      }
    }
}

/** Evaluate the function at the specifed index */
template< class TInputImage, class TOutput >
typename DiscreteHessianGaussianImageFunction< TInputImage, TOutput >::OutputType
DiscreteHessianGaussianImageFunction< TInputImage, TOutput >
::EvaluateAtIndex(const IndexType & index) const
{
  OutputType hessian;

  for ( unsigned int i = 0; i < m_KernelArray.Size(); ++i )
    {
    m_OperatorImageFunction->SetOperator(m_KernelArray[i]);
    hessian[i] = m_OperatorImageFunction->EvaluateAtIndex(index);
    }
  return hessian;
}

/** Evaluate the function at the specifed point */
template< class TInputImage, class TOutput >
typename DiscreteHessianGaussianImageFunction< TInputImage, TOutput >::OutputType
DiscreteHessianGaussianImageFunction< TInputImage, TOutput >
::Evaluate(const PointType & point) const
{
  if ( m_InterpolationMode == NearestNeighbourInterpolation )
    {
    IndexType index;
    this->ConvertPointToNearestIndex(point, index);
    return this->EvaluateAtIndex (index);
    }
  else
    {
    ContinuousIndexType cindex;
    this->ConvertPointToContinuousIndex(point, cindex);
    return this->EvaluateAtContinuousIndex(cindex);
    }
}

/** Evaluate the function at specified ContinousIndex position.*/
template< class TInputImage, class TOutput >
typename DiscreteHessianGaussianImageFunction< TInputImage, TOutput >::OutputType
DiscreteHessianGaussianImageFunction< TInputImage, TOutput >
::EvaluateAtContinuousIndex(const ContinuousIndexType & cindex) const
{
  // TODO: we should think on resuing code latest code of itk::LinearInterpolateImageFunction for this

  typedef unsigned int NumberOfNeighborsType;

  unsigned int  dim; // index over dimension
  NumberOfNeighborsType neighbors = 1 << ImageDimension;

  // Compute base index = closet index below point
  // Compute distance from point to base index
  IndexType baseIndex;
  double    distance[ImageDimension];

  if ( m_InterpolationMode == NearestNeighbourInterpolation )
    {
    IndexType index;
    this->ConvertContinuousIndexToNearestIndex(cindex, index);
    return this->EvaluateAtIndex(index);
    }
  else
    {    
    for ( dim = 0; dim < ImageDimension; dim++ )
      {
      baseIndex[dim] = static_cast<IndexValueType>( vcl_floor( cindex[dim] ) );
      distance[dim] = cindex[dim] - static_cast< double >( baseIndex[dim] );
      }

    // Interpolated value is the weighted sum of each of the surrounding
    // neighbors. The weight for each neighbor is the fraction overlap
    // of the neighbor pixel with respect to a pixel centered on point.
    OutputType hessian, currentHessian;
    TOutput    totalOverlap = itk::NumericTraits< TOutput >::Zero;
    bool firstOverlap = true;

    for ( NumberOfNeighborsType counter = 0; counter < neighbors; counter++ )
      {
      double       overlap = 1.0;    // fraction overlap
      NumberOfNeighborsType upper = counter;  // each bit indicates upper/lower neighbour
      IndexType    neighIndex;

      // get neighbor index and overlap fraction
      for ( dim = 0; dim < ImageDimension; dim++ )
        {
        if ( upper & 1 )
          {
          neighIndex[dim] = baseIndex[dim] + 1;
          // Take care of the case where the pixel is just
          // in the outer upper boundary of the image grid.
          if ( neighIndex[dim] > this->m_EndIndex[dim] )
            {
            neighIndex[dim] = this->m_EndIndex[dim];
            }
          overlap *= distance[dim];
          }
        else
          {
          neighIndex[dim] = baseIndex[dim];
          // Take care of the case where the pixel is just
          // in the outer lower boundary of the image grid.
          if ( neighIndex[dim] < this->m_StartIndex[dim] )
            {
            neighIndex[dim] = this->m_StartIndex[dim];
            }
          overlap *= 1.0 - distance[dim];
          }
        upper >>= 1;
        }

      // get neighbor value only if overlap is not zero
      if ( overlap )
        {
        currentHessian = this->EvaluateAtIndex(neighIndex);
        for ( unsigned int i = 0; i < hessian.Size(); ++i )
          {
          hessian[i] += overlap * currentHessian[i];
          }
        totalOverlap += overlap;
        }

      if ( totalOverlap == 1.0 )
        {
        // finished
        break;
        }
      }

    return hessian;
    }
}

} // end namespace ivan

#endif
