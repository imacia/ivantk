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
// File: ivanVesselnessBasedVesselTrackerEndCondition.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/06/05


#ifndef __ivanVesselnessBasedVesselTrackerEndCondition_h
#define __ivanVesselnessBasedVesselTrackerEndCondition_h

#include "ivanVesselTrackerEndCondition.h"


namespace ivan
{

template <class TValue = double>
class ITK_EXPORT AbsoluteVesselnessMinValueFunctor
{
public:

  typedef TValue     ValueType;
  
public:
  
  AbsoluteVesselnessMinValueFunctor() : m_MinValue( 0.0 ) {}
  
  void SetMinValue( double minValue )
    { this->m_MinValue = minValue; }
  double GetMinValue() const
    { return this->m_MinValue; }

  bool operator () ( const TValue & value )
    {
      if( value <= this->m_MinValue )
        return true;
      else
        return false;
    }
    
private:
  
  ValueType   m_MinValue;
};


/** \class VesselnessBasedVesselTrackerEndCondition
 *  \brief End condition based on a value of vesselness.
 *
 * This class implements and end condition for vessel tracking based on a value of
 * vesselness. The value functor is used to compute the condition
 *
 */

template <class TVesselnessFunction, class TValueFunctor = 
  AbsoluteVesselnessMinValueFunctor<typename TVesselnessFunction::OutputType> >
class ITK_EXPORT VesselnessBasedVesselTrackerEndCondition : public VesselTrackerEndCondition
{
public:
  
  typedef VesselnessBasedVesselTrackerEndCondition
    <TVesselnessFunction,TValueFunctor>    Self;
  typedef VesselTrackerEndCondition        Superclass;
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;
  
  typedef TVesselnessFunction                          VesselnessFunctionType;
  typedef typename VesselnessFunctionType::Pointer     VesselnessFunctionPointer;
  typedef typename VesselnessFunctionType::OutputType  VesselnessValueType;
    
  typedef typename VesselnessFunctionType::PointType   PointType;
    
  typedef TValueFunctor                    ValueFunctorType; 
    
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselnessBasedVesselTrackerEndCondition, VesselTrackerEndCondition );
  
  virtual void SetVesselnessFunction( VesselnessFunctionType *vesselness )
    { 
      this->m_VesselnessFunction = vesselness;
      this->Modified();
    }
  VesselnessFunctionType * GetVesselnessFunction()
    { return this->m_VesselnessFunction.GetPointer(); }
  const VesselnessFunctionType * GetVesselnessFunction() const
    { return this->m_VesselnessFunction.GetPointer(); }
    
  void SetPosition( PointType *position )
    { this->m_Position = position; }
    
  /** Set/Get the value functor which calculates the condition based on the vesselness value. 
    * This is created by default, but may be useful to access it to set/get properties. 
    * We do not provide a Set method since the functor is static for simplicity. */
  ValueFunctorType * GetValueFunctor()
    { return &this->m_ValueFunctor; }
  const ValueFunctorType * GetValueFunctor() const
    { return &this->m_ValueFunctor; }
  
  virtual bool Finished();

protected:
  
  VesselnessBasedVesselTrackerEndCondition();
  virtual ~VesselnessBasedVesselTrackerEndCondition() {}
  virtual void PrintSelf(std::ostream& os, itk::Indent indent) const;
      
private:
  
  VesselnessBasedVesselTrackerEndCondition(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
  
protected:
  
  VesselnessFunctionPointer  m_VesselnessFunction;
  
  ValueFunctorType           m_ValueFunctor;
  
  PointType                  m_Position;
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanVesselnessBasedVesselTrackerEndCondition.hxx"
#endif

#endif
