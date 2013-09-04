import subprocess
import math

exe_path_str = 'C:/SDKs/IVANBinaries/bin/Release/'
output_path_str = 'D:/VolumeData/VesselExperiments/'
#output_path_str = 'F:/VolumeData/VesselExperiments/'

gaussian_tube_exe_str = exe_path_str + 'TestCircularGaussianTubeSinusoidalRadius.exe'
noise_exe_str = exe_path_str + 'UtilityGaussianNoiseAdder.exe'
gaussian_tube_output_str_root = output_path_str + 'TestObjects/' + 'CircularGaussianTubeSin2Lobes_'

# We generate scales as sigma_i = sigma_0 * factor^i where factor is a number > 1.0
numScales = 4
scaleFactor = 1.5
zeroScale = 1.0
currentScale = zeroScale

minScales = [ 1.0, 2.25, 1.0 ]
maxScales = [ 3.375, 5.0625, 5.0 ]

#minScales = [ 1.0 ]
#maxScales = [ 5.0 ]

numLobes = 2
tubeSpacing = 1.0
maxValue = 255.0

noise_levels = [ 10, 25, 50, 75 ]

for i in range( len( minScales ) ):

  tubeSpacingModf  = math.modf( tubeSpacing )
  tubeScaleMinModf = math.modf( minScales[i] )
  tubeScaleMaxModf = math.modf( maxScales[i] )
  
  tubeHeightStr = '81'
  
  gaussian_tube_output_str = gaussian_tube_output_str_root + \
    str( int( tubeScaleMinModf[1] ) ) + '_' + str( int( tubeScaleMinModf[0] * 100.0 ) ) + \
    '-' + str( int( tubeScaleMaxModf[1] ) ) + '_' + str( int( tubeScaleMaxModf[0] * 100.0 ) ) + '.mhd'
    
  print gaussian_tube_exe_str, gaussian_tube_output_str, str( minScales[i] ), str( maxScales[i] ), tubeHeightStr, str( numLobes ), str( maxValue ), str( tubeSpacing )
  
  subprocess.call( [ gaussian_tube_exe_str, gaussian_tube_output_str, str( minScales[i] ), str( maxScales[i] ), tubeHeightStr, str( numLobes ), str( maxValue ), str( tubeSpacing ) ] )

  # Now create noisy versions
    
  for k in range ( 0, len( noise_levels ) ):
      
    gaussian_tube_output_noise_str = gaussian_tube_output_str_root + \
    str( int( tubeScaleMinModf[1] ) ) + '_' + str( int( tubeScaleMinModf[0] * 100.0 ) ) + \
    '-' + str( int( tubeScaleMaxModf[1] ) ) + '_' + str( int( tubeScaleMaxModf[0] * 100.0 ) ) + \
    '_noise' + str( noise_levels[k] ) + '.mhd'
    
    print noise_exe_str, gaussian_tube_output_str, gaussian_tube_output_noise_str, str( noise_levels[k] ), str( maxValue )
    subprocess.call( [ noise_exe_str, gaussian_tube_output_str, gaussian_tube_output_noise_str, str( noise_levels[k] ), str( maxValue ) ] )