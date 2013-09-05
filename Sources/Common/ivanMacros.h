/*=========================================================================

Image-based Vascular Analysis Toolkit (IVAN)

Copyright (c) 2012-2013, Ivan Macia Oliver
Vicomtech Foundation, San Sebastian - Donostia (Spain)
University of the Basque Country, San Sebastian - Donostia (Spain)

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
// File: ivanMacros.h
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: global macros used in the library
// Date: 2013/09/05

#ifndef __ivanMacros_h_
#define __ivanMacros_h_

#include "itkConfigure.h"

#if ITK_VERSION_MAJOR < 4
  #define ITKImageDimensionMacro( TImage ) \
    ::itk::GetImageDimension< TImage >::ImageDimension
#else
  #define ITKImageDimensionMacro( TImage ) \
    TImage::ImageDimension
#endif

#endif // __ivanMacros_h_

