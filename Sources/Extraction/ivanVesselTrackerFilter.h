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
// File: ivanVesselTrackerFilter.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/05/27


#ifndef __ivanVesselTrackerFilter_h
#define __ivanVesselTrackerFilter_h

#include "ivanImageToVesselDataObjectFilter.h"
#include "ivanVesselBranchNode.h"
#include "ivanVesselSectionEstimator.h"
#include "ivanVesselTrackerEndCondition.h"


namespace ivan
{
  
/** \class VesselTrackerFilter
 * \brief Base class for filters that take an image as input and produce a VesselDataObject as output.
 *
 * VesselTrackerFilter is the base class for all process objects that output
 * VesselDataObject data and require an image as input. Specifically, this class
 * defines the SetInput() method for defining the input to a filter.
 *
 * Currently VesselTrackerFilter only supports a VesselGraph as output. Another assumption is that
 * the center point of the section is an array-like object (such as Array or std::vector) with the 
 * same dimension as the image and the random access [] operator defined.
 *
 */

template <class TInputImage, class TOutputVessel>
class ITK_EXPORT VesselTrackerFilter : public ImageToVesselDataObjectFilter<TInputImage,TOutputVessel>
{
public:
	
  /** Standard class typedefs. */
  typedef VesselTrackerFilter
    <TInputImage, TOutputVessel>          Self;
  typedef ImageToVesselDataObjectFilter
    <TInputImage,TOutputVessel>           Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
  
  /** Image-related typedefs. */
  typedef TInputImage                             InputImageType;
  typedef typename InputImageType::Pointer        InputImagePointer;
  typedef typename InputImageType::ConstPointer   InputImageConstPointer;
  typedef typename InputImageType::RegionType     InputImageRegionType; 
  typedef typename InputImageType::PixelType      InputImagePixelType;
  typedef typename InputImageType::PointType      InputImagePointType;
  typedef typename InputImageType::IndexType      InputImageIndexType; 
  	
  /** Vessel-related typedefs. */
  typedef TOutputVessel                              OutputVesselType;
  typedef typename OutputVesselType::Pointer         OutputVesselPointer;
      
  typedef typename OutputVesselType::CenterlineType  CenterlineType;
  typedef typename CenterlineType::SectionType       SectionType;
    
  typedef VesselBranchNode<CenterlineType>           BranchNodeType;
  typedef typename BranchNodeType::Pointer           BranchNodePointer;
  
  typedef VesselSectionEstimator<CenterlineType>     SectionEstimatorType;
  typedef typename SectionEstimatorType::Pointer     SectionEstimatorPointer;
  
  typedef VesselTrackerEndCondition                  EndConditionType;
  typedef typename EndConditionType::Pointer         EndConditionPointer;
    
  /** ImageDimension constants */
  itkStaticConstMacro(InputImageDimension, unsigned int,
                      TInputImage::ImageDimension);
  
public:
  
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( VesselTrackerFilter, ImageToVesselObjectFilter );
  
  /** Flag for enabling search stage, if implemented in subclasses. */
  itkSetMacro( EnableSearchStage, bool );
  itkGetConstMacro( EnableSearchStage, bool );
  itkBooleanMacro( EnableSearchStage );
  
  /** Flag for enabling search stage, if implemented in subclasses. */
  itkSetMacro( InvertDirection, bool );
  itkGetConstMacro( InvertDirection, bool );
  itkBooleanMacro( InvertDirection );
  
  itkSetMacro( InitialStepSize, double );
  itkGetConstMacro( InitialStepSize, double );
  
  const BranchNodeType * GetCurrentBranch() const
    { return m_CurrentBranch.GetPointer(); }
  BranchNodeType * GetCurrentBranch()
    { return m_CurrentBranch.GetPointer(); }

  itkGetConstMacro( BranchPointIndex, unsigned int );

  virtual void SetStartingPoint( const InputImagePointType & point )
    { m_PreviousPoint = m_CurrentPoint = point; }
  const InputImagePointType & GetPreviousPoint() const
    { return m_PreviousPoint; }
  const InputImagePointType & GetCurrentPoint() const
    { return m_CurrentPoint; }
    
  /** Set/Get the SectionEstimator. Properties for this object must be set
    * externally before computing. */
  virtual void SetSectionEstimator( SectionEstimatorType * sectionEstimator );
  const SectionEstimatorType * GetSectionEstimator() const
    { return m_SectionEstimator.GetPointer(); }
  SectionEstimatorType * GetSectionEstimator()
    { return m_SectionEstimator.GetPointer(); }
    
  /** Set the end condition. */
  virtual void SetEndCondition( VesselTrackerEndCondition *endCondition )
    { m_EndCondition = endCondition; }
      
protected:
  
  VesselTrackerFilter();
  ~VesselTrackerFilter() {}
  
  virtual void GenerateData();
  
  /** Initializes the algorithm. The default version creates a vessel graph with a single branch.
    * The initialization can also be used, for example, to search the vessel center from the starting point. */
  virtual void Initialize();
  
  /** Search for the optimum vessel (center) location after we have adivanced the given step size. 
    * By default the search step is not implemented. Under the assumption of smoothness and with a small
    * step size, if the tracking method tracks the centerline (most common case) the next point can
    * be considered also to lie on the centerline. This method may be used to incorporate and optimization
    * stage where the centerline, and possibly the section normal, are optimized at the current location. */
  virtual void Search() {}
  
  /** Orient the local framework to estimate the vessel normal and possibly the section. */
  virtual void Turn();
  
  /** Estimate/Measure the desired properties. */
  virtual void Measure() {}
  
  /** Adivance in the estimated direction and possibly adapt the step size based on the estimated
    * measurements such as curvature. */
  virtual void Step();
  
  /** Check if we have finished. */
  virtual bool Finished();
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

private:

  VesselTrackerFilter(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
  
protected:

  bool                     m_EnableSearchStage;
  
  bool                     m_InvertDirection;

  double                   m_InitialStepSize;
  
  /** This represents the vessel point in the last iteration. */
  InputImagePointType      m_PreviousPoint;
  
  /** This represents the vessel point in the current iteration. It is updated in the Search step. */
  InputImagePointType      m_CurrentPoint;
  
  SectionEstimatorPointer  m_SectionEstimator;
    
  EndConditionPointer      m_EndCondition;
  
  /** Current branch. */
  BranchNodePointer        m_CurrentBranch;
  
  /** Current point index in branch. */
  unsigned int             m_BranchPointIndex;  
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanVesselTrackerFilter.hxx"
#endif

#endif
