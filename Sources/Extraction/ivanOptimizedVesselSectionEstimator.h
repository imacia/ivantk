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
// File: ivanOptimizedVesselSectionEstimator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2011/11/04


#ifndef __ivanOptimizedVesselSectionEstimator_h
#define __ivanOptimizedVesselSectionEstimator_h


#include "ivanVesselSectionEstimator.h"


namespace ivan
{

/** \class OptimizedVesselSectionEstimator
 *  \brief Estimates the vessel section using an optimization procedure given a (image-based) cost function.
 *
 * The cost function must be a subclass of VesselSectionFitCostFunction. Any optimizer may be used.
 *
 * \ingroup 
 */

template <class TCostFunction, class TOptimizer, class TCenterline, 
  class TMetricsCalculator = VesselSectionMetricsCalculator<TCenterline> >
class ITK_EXPORT OptimizedVesselSectionEstimator : 
  public VesselSectionEstimator<TCenterline,TMetricsCalculator>
{
public:

  /** Standard class typedefs. */
  typedef OptimizedVesselSectionEstimator
    <TCostFunction, TOptimizer, TCenterline, TMetricsCalculator>   Self;
  typedef VesselSectionEstimator<TCenterline,TMetricsCalculator>   Superclass;
  typedef itk::SmartPointer<Self>                                  Pointer;
  typedef itk::SmartPointer<const Self>                            ConstPointer;
  
  typedef TCenterline                             CenterlineType;
  typedef typename CenterlineType::Pointer        CenterlinePointer;
  
  typedef typename CenterlineType::SectionType    SectionType;
  typedef typename SectionType::Pointer           SectionPointer;
  
  typedef typename TCostFunction                  CostFunctionType;
  typedef typename CostFunctionType::Pointer      CostFunctionPointer;
  
  typedef typename TOptimizer                     OptimizerType;
  typedef typename OptimizerType::Pointer         OptimizerPointer;
    
  typedef typename OptimizerType::ParametersType  ParametersType;
       
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( OptimizedVesselSectionEstimator, VesselSectionEstimator );
  
  itkSetObjectMacro( CostFunction, CostFunctionType );
  itkGetObjectMacro( CostFunction, CostFunctionType );
  itkGetConstObjectMacro( CostFunction, CostFunctionType );
  
  itkSetObjectMacro( Optimizer, OptimizerType );
  itkGetObjectMacro( Optimizer, OptimizerType );
  itkGetConstObjectMacro( Optimizer, OptimizerType );
  
  itkSetObjectMacro( Centerline, CenterlineType );
  itkGetObjectMacro( Centerline, CenterlineType );
  itkGetConstObjectMacro( Centerline, CenterlineType );
  
  /** Set the initial position for the optimizer. */
  //virtual void SetInitialPosition( const ParametersType &params )
  //  { this->m_Optimizer->SetInitialPosition( params ); }
  
  virtual void Compute();
    
protected:
  
  OptimizedVesselSectionEstimator();
  ~OptimizedVesselSectionEstimator();
  
  void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  OptimizedVesselSectionEstimator(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

  CostFunctionPointer       m_CostFunction;
  OptimizerPointer          m_Optimizer;
  
  CenterlinePointer         m_Centerline;
};

} // end namespace ivan

#if ITK_TEMPLATE_TXX
# include "ivanOptimizedVesselSectionEstimator.hxx"
#endif

#endif
