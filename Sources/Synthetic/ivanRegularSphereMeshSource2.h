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
// File: ivanRegularSphereMeshSource2.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a regular sphere in the form of an itk::Mesh by decimating an icosahedron 
// Date: 2010/08/20

#ifndef __ivanRegularSphereMeshSource2_h
#define __ivanRegularSphereMeshSource2_h

#include "itkMesh.h"
#include "itkMeshSource.h"
#include "itkTriangleCell.h"
#include "itkDefaultStaticMeshTraits.h"


namespace ivan
{

/** \class RegularSphereMeshSource2
 * \brief 
 *
 */
template <class TPixel>
class ITK_EXPORT RegularSphereMeshSource2 : public itk::MeshSource<itk::Mesh<TPixel,3> >
{
public:
  
  /** Standard typedefs. */
  typedef RegularSphereMeshSource2   Self;
  typedef itk::MeshSource
    <itk::Mesh<TPixel,3> >          Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef itk::Mesh<TPixel,3>                   OutputMeshType;
  typedef typename OutputMeshType::MeshTraits   OutputMeshTraits;
  typedef typename OutputMeshType::PointType    OutputPointType;
  typedef typename OutputMeshTraits::PixelType  OutputPixelType;  

  /** Some convenient typedefs. */
  typedef typename OutputMeshType::Pointer             OutputMeshPointer;
  typedef typename OutputMeshType::PointsContainer     PointsContainer;

  typedef typename OutputMeshType::CellType            CellInterfaceType;
  typedef typename CellInterfaceType::CellAutoPointer  CellAutoPointer;
    
  typedef itk::TriangleCell<CellInterfaceType>         TriangleCellType;
    
  typedef typename OutputMeshType::PointIdentifier     PointIdentifier;
   
  struct EdgeStruct
  {
    unsigned long    edgeId;
    PointIdentifier  first;
    PointIdentifier  second;
    
    EdgeStruct() : edgeId(0), first(0), second(0) {}
    EdgeStruct( unsigned long id, PointIdentifier fst, PointIdentifier snd ) :
      edgeId( id ), first( fst ), second( snd ) {}
      
    bool operator == ( const EdgeStruct & another )
    {
      // compare only by id
      return ( first == another.first && second == another.second ); 
    }
    
    bool operator < ( const EdgeStruct & another )
    {
      if( first < another.first )
        return true;
      else if( first > another.first )
        return false;
        
      if( second < another.second )
        return true;
      else if( second > another.second )
        return false;  
      
      // Should not reach this point
      if( edgeId < another.edgeId )
        return true;
      else
        return false; 
    }
  };    
    
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );  

  /** Run-time type information (and related methods). */
  itkTypeMacro( RegularSphereMeshSource2, MeshSource );

  itkSetMacro( Center, OutputPointType );
  itkGetConstMacro( Center, OutputPointType );
  
  itkSetMacro( Radius, double );
  itkGetConstMacro( Radius, double );

  itkSetMacro( Decimation, unsigned int );
  itkGetConstMacro( Decimation, unsigned int );

protected:
  
  RegularSphereMeshSource2();
  ~RegularSphereMeshSource2() {}
  
  void GenerateData();
  
  virtual void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:
  
  RegularSphereMeshSource2(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:
  
  /** Model center */
  OutputPointType  m_Center; 

  /** Radius or scale. Default radius is 1.0 */
  double           m_Radius;
  
  /** Decimation level. */
  unsigned int     m_Decimation;
};

} // end namespace ivan
#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanRegularSphereMeshSource2.hxx"
#endif
#endif
