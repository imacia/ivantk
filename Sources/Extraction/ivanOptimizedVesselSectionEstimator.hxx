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
// File: ivanOptimizedVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: generic node of an acyclic graph structure
// Date: 2011/11/04


#ifndef __ivanOptimizedVesselSectionEstimator_hxx
#define __ivanOptimizedVesselSectionEstimator_hxx

#include "ivanOptimizedVesselSectionEstimator.h"

#ifdef _DEBUG
  #include <fstream>
#endif

namespace ivan
{

template <class TCostFunction, class TOptimizer, class TCenterline, class TMetricsCalculator>
OptimizedVesselSectionEstimator<TCostFunction,TOptimizer,TCenterline,TMetricsCalculator>
::OptimizedVesselSectionEstimator()
{
  this->m_CostFunction = CostFunctionType::New();
  this->m_Optimizer = OptimizerType::New();
}


template <class TCostFunction, class TOptimizer, class TCenterline, class TMetricsCalculator>
OptimizedVesselSectionEstimator<TCostFunction,TOptimizer,TCenterline,TMetricsCalculator>
::~OptimizedVesselSectionEstimator()
{

}


template <class TCostFunction, class TOptimizer, class TCenterline, class TMetricsCalculator>
void
OptimizedVesselSectionEstimator<TCostFunction,TOptimizer,TCenterline,TMetricsCalculator>
::Compute()
{
  assert( this->m_Optimizer.IsNotNull() && this->m_CostFunction.IsNotNull() );
  assert( this->GetCenterline() );
  assert( this->GetCenterline()->size() );
  
  this->m_Optimizer->SetCostFunction( this->m_CostFunction );
  
  OptimizerType::ParametersType initialPosition = OptimizerType::ParametersType
    ( this->m_CostFunction->GetNumberOfParameters() );
  OptimizerType::ParametersType  finalParameters;
  
  SectionPointer                          currentSection;
  typename SectionType::CenterPointType   currentCenter;
  typename SectionType::VectorType        currentNormal;  
  typename SectionType::VectorType        finalNormal;
  double                                  finalRadius;
  
  // If the section range is not specified take all the sections
  if( this->m_SectionRange[0] == 0 && this->m_SectionRange[1] == 0 )
    this->m_SectionRange[1] = this->GetCenterline()->size() - 1;
      
  for( unsigned int i = this->m_SectionRange[0]; i <= m_SectionRange[1]; ++i )
  {
    currentSection = this->GetCenterline()->at(i);
    
    this->m_CostFunction->SetSectionCenter( currentSection->GetCenter() );
    this->m_CostFunction->GetMedialnessFunction()->SetSigma( currentSection->GetScale() ); // this is the sigma for derivative calculations
    this->m_CostFunction->GetMedialnessFunction()->Initialize(); // recompute kernel since we changed sigma
    
    // Get the initial position from each section
    
    currentNormal = currentSection->GetNormal();

    initialPosition[0] = currentNormal[0];
    initialPosition[1] = currentNormal[1];
    //initialPosition[2] = currentNormal[2];
    //initialPosition[3] = currentSection->GetRadius();
    //initialPosition[4] = 1.0; // for the Lagrange multiplier
    initialPosition[2] = currentSection->GetRadius();
    
    this->m_Optimizer->SetInitialPosition( initialPosition );
    
    try 
    { 
      this->m_Optimizer->StartOptimization(); 
    } 
    catch (itk::ExceptionObject& excpt)
    {
      std::cerr << "EXCEPTION CAUGHT!" << excpt.GetDescription() << std::endl;
      return;
    }  
          
    finalParameters = this->m_Optimizer->GetCurrentPosition();
    
    finalNormal[0] = finalParameters[0];
    finalNormal[1] = finalParameters[1];
    //finalNormal[2] = finalParameters[2];
    finalNormal[2] = 1.0 - finalNormal[0] * finalNormal[0] - finalNormal[1] * finalNormal[1];
    
    if( finalNormal[2] < 0.0 ) // possibly due to rounding errors
      finalNormal[2] = 0.0;
    else
      finalNormal[2] = sqrt( finalNormal[2] );
    
    //finalNormal.Normalize();
    
    finalRadius = finalParameters[2];
    
    std::cout << "Optimization finished!!!" << std::endl;
    std::cout << "Reason: " << this->m_Optimizer->GetStopConditionDescription() << std::endl;
    std::cout << "Point " << currentSection->GetCenter() << std::endl;
    std::cout << " Normal     = " << finalNormal  << std::endl;
    std::cout << " Radius     = " << finalRadius  << std::endl;
    std::cout << " Iterations = " << this->m_Optimizer->GetCurrentIteration() << std::endl;
    std::cout << " Best value = " << this->m_Optimizer->GetValue() << std::endl;
    std::cout << " Frobenius norm = " << this->m_Optimizer->GetFrobeniusNorm() << std::endl;
    std::cout << std::endl;
    
    currentSection->SetNormal( finalNormal );
    currentSection->SetRadius( finalRadius );
    
    this->m_CurrentScale = finalRadius; // !!! WARNING: CHECK THIS
  }
}


template <class TCostFunction, class TOptimizer, class TCenterline, class TMetricsCalculator>
void 
OptimizedVesselSectionEstimator<TCostFunction,TOptimizer,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "CostFunction: " << this->m_CostFunction << std::endl;
  this->m_CostFunction->Print( os, indent.GetNextIndent() );
  
  os << indent << "Optimizer: " << this->m_Optimizer << std::endl;
  this->m_Optimizer->Print( os, indent.GetNextIndent() );
  
  os << indent << "Centerline: " << this->m_Centerline << std::endl;
  this->m_Centerline->Print( os, indent.GetNextIndent() );
}

} // end namespace ivan

#endif // __ivanOptimizedVesselSectionEstimator_hxx
