import subprocess
import math

exe_path_str = 'C:/SDKs/IVANBinaries/bin/Release/'
output_path_str = 'D:/VolumeData/VesselExperiments/'

gaussian_tube_exe_str = exe_path_str + 'TestCircularGaussianCircularTube.exe'
gaussian_tube_output_str_root = output_path_str + 'TestObjects/' + 'GaussianCircularTube_'

tubeScales = [1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0, 7.0, 7.5, 8.0, 9.0, 10.0]
#tubeSpacing = 0.3

normalize = 0
maxValue = 255.0
sectionImageSize = 0
writeCenterline = 1

for i in range( 0, len( tubeScales ) ):

  tubeSpacing = 0.2 * tubeScales[i]
  tubeSpacingModf = math.modf( tubeSpacing )
  
  tubeScaleModf = math.modf( tubeScales[i] )
  
  if tubeScales[i] <= 3.0:
    tubeHeightStr = '41'
  elif tubeScales[i] > 3.0 and tubeScales[i] <= 5.0:
    tubeHeightStr = '81'
  elif tubeScales[i] > 3.0 and tubeScales[i] <= 8.0:
     tubeHeightStr = '121'
  else:
     tubeHeightStr = '161'
  
  gaussian_tube_output_str = gaussian_tube_output_str_root + \
    str( int( tubeScaleModf[1] ) ) + '_' + str( int( tubeScaleModf[0] * 100.0 ) ) + \
    '_sp' + str( int( tubeSpacingModf[1] ) ) + '_' + str( int( tubeSpacingModf[0] * 100.0 ) ) + '.mhd'
  
  print gaussian_tube_exe_str, gaussian_tube_output_str, str( tubeScales[i] ), tubeHeightStr, \
    str( normalize ), str( maxValue ), str( tubeSpacing ), str( sectionImageSize ), str( writeCenterline )
  
  subprocess.call( [ gaussian_tube_exe_str, gaussian_tube_output_str, str( tubeScales[i] ), tubeHeightStr, \
    str( normalize ), str( maxValue ), str( tubeSpacing ), str( sectionImageSize ), str( writeCenterline ) ] )
