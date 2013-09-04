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
// File: CenterlineGenerator.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: base class for centerline generators
// Date: 2010/10/02


#ifndef __CenterlineGenerator_h_
#define __CenterlineGenerator_h_

#include "itkPoint.h"
#include "itkVectorContainer.h"


template <class TCoordRep=double, unsigned int VDimension=3>
class CenterlineGenerator
{
public:
  
  typedef TCoordRep                          CoordRepType;
  typedef itk::Point<TCoordRep, VDimension>  PointType;

  typedef itk::VectorContainer<unsigned int,PointType>  CenterlineType;
  typedef typename CenterlineType::Pointer              CenterlinePointer;
  
public:

  CenterlineGenerator() {};
  ~CenterlineGenerator() {};
  
  virtual CenterlinePointer Create() = 0;

protected: 
  

};

#endif // __CenterlineGenerator_h_

