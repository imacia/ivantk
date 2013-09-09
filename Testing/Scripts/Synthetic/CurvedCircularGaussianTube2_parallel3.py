import subprocess
import math

test_exe_path_str = 'C:/SDKs/ITKVesselBin/Tests/Release/'
objects_exe_path_str = 'C:/SDKs/ITKVesselBin/TestObjects/Release/'
output_path_str = 'D:/VolumeData/VesselExperiments/'

gaussian_tube_exe_str = objects_exe_path_str + 'CurvedCircularGaussianTube2.exe'
gaussian_tube_output_str_root = output_path_str + 'TestObjects/' + 'CurvedGaussianTube_'

tubeSpacing = 0.3
tubeHeightStr = '100'
tubeScales = [1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
tubeXOffsets = [25.0, 37.5, 50.0]

for i in range( 0, len( tubeScales ) ):
  tubeScaleModf = math.modf( tubeScales[i] )
  tubeImageSizeY = int( 5.0 * tubeScales[i] / tubeSpacing )
  if tubeImageSizeY < 21:
    tubeImageSizeY = 21
  elif tubeImageSizeY % 2 == 0:
    tubeImageSizeY += 1  
  for j in range( 0, len( tubeXOffsets ) ):
    tubeOffsetModf = math.modf( tubeXOffsets[j] )
    tubeImageSizeX = int( 2.5 * ( tubeXOffsets[j] + 2.0 * tubeScales[i] ) / tubeSpacing )
    if tubeImageSizeX < 21:
      tubeImageSizeX = 21
    elif tubeImageSizeX % 2 == 0:
      tubeImageSizeX += 1  
    print 'Scale', tubeScales[i], 'XOffset', tubeXOffsets[j], 'ImageSize', tubeImageSizeX
    gaussian_tube_output_str = gaussian_tube_output_str_root + str( int( tubeScaleModf[1] ) ) + '_' + str( int( tubeScaleModf[0] * 100.0 ) ) + '_xoff_' + str( int( tubeOffsetModf[1] ) ) + '_' + str( int( tubeOffsetModf[0] * 100.0 ) ) + '.mhd'
    subprocess.call( [ gaussian_tube_exe_str, gaussian_tube_output_str, str( tubeScales[i] ), tubeHeightStr, str( tubeXOffsets[j] ), str( tubeImageSizeX ), str( tubeImageSizeY ), str( tubeSpacing )] )
