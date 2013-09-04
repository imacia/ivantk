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
// Date: 2011/11/07


#ifndef __ivanOptimizerIterationCommand_h
#define __ivanOptimizerIterationCommand_h


#include <itkCommand.h>
#include "itkWeakPointer.h"


namespace ivan
{

/**
 *  Implementation of the Command Pattern to be invoked every iteration
 */
template <class TOptimizer>
class ITK_EXPORT OptimizerIterationCommand : public itk::Command 
{
public:
  
  typedef OptimizerIterationCommand      Self;
  typedef itk::Command                   Superclass;

  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  /** Type defining the optimizer. */
  typedef TOptimizer                     OptimizerType;

  
public:
  
  /** Method for creation through the object factory. */
  itkNewMacro( Self );
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( OptimizerIterationCommand, itk::Command );

  /** Execute method will print data at each iteration. */
  void Execute( itk::Object *caller, const itk::EventObject & event )
    {
      this->Execute( (const itk::Object *)caller, event );
    }

  void Execute( const itk::Object *, const itk::EventObject & event )
  {
    if( typeid( event ) == typeid( itk::StartEvent ) )
      {
      std::cout << std::endl << "Position              Value";
      std::cout << std::endl << std::endl;
      }    
    else if( typeid( event ) == typeid( itk::IterationEvent ) )
      {
      std::cout << m_Optimizer->GetCurrentIteration() << " = ";
      std::cout << m_Optimizer->GetValue() << " : ";
      std::cout << m_Optimizer->GetCurrentPosition() << std::endl;
      }
    else if( typeid( event ) == typeid( itk::EndEvent ) )
      {
      std::cout << std::endl << std::endl;
      std::cout << "After " << m_Optimizer->GetCurrentIteration();
      std::cout << "  iterations " << std::endl;
      std::cout << "Solution is    = " << m_Optimizer->GetCurrentPosition();
      std::cout << std::endl;
      std::cout << "With value     = " << m_Optimizer->GetValue();
      std::cout << std::endl;
      std::cout << "Stop condition = " << m_Optimizer->GetStopCondition();
      std::cout << std::endl;
      }

  }



  /**
   * Set Optimizer
   */
  void SetOptimizer( OptimizerType * optimizer )
    { 
      m_Optimizer = optimizer;
      m_Optimizer->AddObserver( itk::IterationEvent(), this );
    }

protected:

  /**
   * Constructor
   */
  CommandIterationUpdate() {};

private:

  /** WeakPointer to the Optimizer. */
  WeakPointer<OptimizerType>   m_Optimizer;  
};

} // end namespace ivan

#endif

