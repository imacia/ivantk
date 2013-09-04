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
// File: ivanOptimizedVesselnessBasedSearchVesselTrackerFilter.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/05/27


#ifndef __ivanOptimizedVesselnessBasedSearchVesselTrackerFilter_h
#define __ivanOptimizedVesselnessBasedSearchVesselTrackerFilter_h

#include "ivanVesselnessBasedSearchVesselTrackerFilter.h"
#include "ivanImage3DPlaneFunctionToCostFunctionAdaptor.h"

#include "itkImage.h"
#include "itkSingleValuedNonLinearOptimizer.h"


namespace ivan
{

/** \class OptimizedVesselnessBasedSearchVesselTrackerFilter
 * \brief VesselTracker that optimizes a vesselness measure to guide the search stage.
 *
 * OptimizedVesselnessBasedSearchVesselTrackerFilter takes a vesselness measure to guide the search stage,
 * and optimizes its value so as to find the optimal values for the plane center and normal.
 * (VERIFY THIS)
 * 
 * The cost function is an adapted vesselness image function used to guide the
 * optimization procedure.
 *
 */

template <class TInputImage, class TOutputVessel, class TVesselnessFunction, class TCostFunction>
class ITK_EXPORT OptimizedVesselnessBasedSearchVesselTrackerFilter : 
  public VesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
{
public:
	
  /** Standard class typedefs. */
  typedef OptimizedVesselnessBasedSearchVesselTrackerFilter
    <TInputImage,TOutputVessel,TVesselnessFunction,TCostFunction>  Self;
  typedef VesselnessBasedSearchVesselTrackerFilter
    <TInputImage,TOutputVessel,TVesselnessFunction>                Superclass;
  typedef itk::SmartPointer<Self>                                  Pointer;
  typedef itk::SmartPointer<const Self>                            ConstPointer;
  
  /** Some convenient typedefs. */
  typedef TInputImage                               ImageType;
  typedef typename Superclass::ImagePointer         ImagePointer;
  typedef typename Superclass::ImageConstPointer    ImageConstPointer;
  typedef typename Superclass::ImageRegionType      ImageRegionType; 
  typedef typename Superclass::ImagePixelType       ImagePixelType;
  typedef typename Superclass::ImagePointType       ImagePointType;
  typedef typename Superclass::ImageIndexType       ImageIndexType; 
  	
  /** Some convenient typedefs. */
  typedef TOutputVessel                             OutputVesselType;
  typedef typename OutputVesselType::Pointer        OutputVesselPointer;
  
  typedef typename Superclass::CenterlineType       CenterlineType;
  typedef typename Superclass::SectionType          SectionType;

  typedef itk::SingleValuedNonLinearOptimizer       OptimizerType;
  
  typedef itk::Image<typename TVesselnessFunction::OutputType, 
    ImageType::ImageDimension>                                    VesselnessImageType;
      
  typedef typename TVesselnessFunction                   VesselnessFunctionType;
  typedef typename VesselnessFunctionType::Pointer       VesselnessFunctionPointer;
  typedef typename VesselnessFunctionType::OutputType    VesselnessValueType;
      
  typedef TCostFunction                             CostFunctionType;
  typedef typename CostFunctionType::Pointer        CostFunctionPointer;

public:
  
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( OptimizedVesselnessBasedSearchVesselTrackerFilter, VesselnessBasedSearchVesselTrackerFilter );
  
  /** Set the VesselnessImageFunction to use. This can be a vesselness calculated from a source
    * image as an image function, or, if the full vesselness image has been previously calculated,
    * can be an interpolator of that image, such as nearest neighbour or linear interpolator. 
    * The vesselness image function is passed to the cost function. */
  virtual void SetVesselnessFunction( VesselnessFunctionType *func )
    { 
      Superclass::SetVesselnessImageFunction( func );
      if( this->m_CostFunction.IsNotNull() )
        this->m_CostFunction->SetImageFunction( func );
      if( this->m_Optimizer.IsNotNull() )
        this->m_Optimizer->SetCostFunction( this->m_CostFunction );
    }

  /** Set/Get the cost function used by the optimizer. The provided vesselness function will be
    * assigned to this cost function. */
  virtual void SetCostFunction( CostFunctionType *costFunction )
   {
     this->m_CostFunction = costFunction;
     if( this->m_VesselnessFunction.IsNotNull() )
       this->m_CostFunction->SetImageFunction( this->m_VesselnessFunction );
     if( this->m_Optimizer.IsNotNull() )
       this->m_Optimizer->SetCostFunction( this->m_CostFunction );
   }
  
  CostFunctionType * GetCostFunction()
    { return this->m_CostFunction; }
  const CostFunctionType * GetCostFunction() const
    { return this->m_CostFunction; }
  
  /** Set the optimizer to use during different stages, such as the Search stage. The parameters
    * of the optimizer must be configured previously. */
  void SetOptimizer( OptimizerType * optimizer )
    { 
      this->m_Optimizer = optimizer;
      if( this->m_CostFunction.IsNotNull() )
        this->m_Optimizer->SetCostFunction( this->m_CostFunction );
      this->Modified();
    }
  itk::Optimizer * GetOptimizer()
    { return m_Optimizer; }
  const itk::Optimizer * GetOptimizer() const
    { return m_Optimizer; }
   
protected:
  
  OptimizedVesselnessBasedSearchVesselTrackerFilter();
  ~OptimizedVesselnessBasedSearchVesselTrackerFilter() {}
  
  /** Reimplemente the search for the optimum as a local optimization of the vesselness function. */
  virtual void Search();
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  OptimizedVesselnessBasedSearchVesselTrackerFilter(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
  
protected:

  CostFunctionPointer       m_CostFunction;
  
  /** Allow changing the optimization algorithm. */
  OptimizerType::Pointer    m_Optimizer;
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanOptimizedVesselnessBasedSearchVesselTrackerFilter.hxx"
#endif

#endif
