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
// File: ivanDodecahedronMeshSource.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates an Dodecahedron in the form of an itk::Mesh
// Date: 2010/08/18

#ifndef __ivanDodecahedronMeshSource_h
#define __ivanDodecahedronMeshSource_h

#include "itkPlatonicSolidMeshSource.h"
#include "itkPolygonCell.h"


namespace ivan
{

/** \class DodecahedronMeshSource
 * \brief 
 *
 */
template <class TPixel>
class ITK_EXPORT DodecahedronMeshSource : public PlatonicSolidMeshSource<TPixel>
{
public:
  
  /** Standard typedefs. */
  typedef DodecahedronMeshSource            Self;
  typedef PlatonicSolidMeshSource
    <TPixel>                        Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  /** Hold on to the type information specified by the template parameters. */
  typedef typename Superclass::OutputMeshType   OutputMeshType;
  typedef typename Superclass::OutputMeshTraits OutputMeshTraits;
  typedef typename Superclass::OutputPointType  OutputPointType;
  typedef typename Superclass::OutputPixelType  OutputPixelType;  

  /** Some convenient typedefs. */
  typedef typename Superclass::OutputMeshPointer      OutputMeshPointer;
  typedef typename Superclass::PointsContainer        PointsContainer;
  
  typedef typename Superclass::CellInterfaceType      CellInterfaceType;
  typedef typename Superclass::CellAutoPointer        CellAutoPointer;
    
  /** This is the type of cell used by this solid. */
  typedef itk::PolygonCell<CellInterfaceType>         PolygonCellType;
    
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );  

  /** Run-time type information (and related methods). */
  itkTypeMacro( DodecahedronMeshSource, MeshSource );
  
  virtual unsigned int GetNumberOfVertices() const
    { return 20; }
  virtual unsigned int GetCellSize() const
    { return 5; }
  virtual unsigned int GetNumberOfCells() const
    { return 12; }

protected:
  
  DodecahedronMeshSource();
  ~DodecahedronMeshSource() {}
  
  virtual void GenerateData();
  
  virtual void PrintSelf(std::ostream& os, itk::Indent indent) const;

private:
  
  DodecahedronMeshSource(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:
  

};

} // end namespace ivan
#ifndef ITK_MANUAL_INSTANTIATION
#include "itkDodecahedronMeshSource.hxx"
#endif
#endif
