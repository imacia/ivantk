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
// File: ivanPlatonicSolidMeshSource.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a platonic solid in the form of an itk::Mesh
// Date: 2010/08/18

#ifndef __ivanPlatonicSolidMeshSource_h
#define __ivanPlatonicSolidMeshSource_h

#include "itkMesh.h"
#include "itkMeshSource.h"
#include "itkDefaultStaticMeshTraits.h"

namespace ivan
{

/** \class PlatonicSolidMeshSource
 * \brief 
 *
 */
template <class TPixel>
class ITK_EXPORT PlatonicSolidMeshSource : public itk::MeshSource<itk::Mesh<TPixel,3> >
{
public:
  
  /** Standard typedefs. */
  typedef PlatonicSolidMeshSource        Self;
  typedef itk::MeshSource
    <itk::Mesh<TPixel,3> >               Superclass;
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
    
public:

  /** Method for creation through the object factory. */
  //itkNewMacro( Self );  

  /** Run-time type information (and related methods). */
  itkTypeMacro( PlatonicSolidMeshSource, MeshSource );

  itkSetMacro( Center, OutputPointType );
  itkGetConstMacro( Center, OutputPointType );
  
  itkSetMacro( Radius, double );
  itkGetConstMacro( Radius, double );
  
  /** Convenience methods for easy access to some properties of the solids. */
  virtual unsigned int GetNumberOfVertices() const = 0;
  virtual unsigned int GetCellSize() const = 0;
  virtual unsigned int GetNumberOfCells() const = 0;

protected:
  
  PlatonicSolidMeshSource();
  ~PlatonicSolidMeshSource() {}
  
  /** Create mesh for several solid provided specific vertex and connectivity data. The actual
    * type of cell will depend on the solid being created. */
  template <class TCell>
  void GenerateMesh( double solidScale, const double *pointData, const unsigned long *vertexData );
  
  virtual void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:
  
  PlatonicSolidMeshSource(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:
  
  /** Model center */
  OutputPointType  m_Center; 

  /** Radius or scale. Default radius is 1.0 */
  double           m_Radius;
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanPlatonicSolidMeshSource.hxx"
#endif
#endif
