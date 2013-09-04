import subprocess
import math

exe_path_str = 'C:/SDKs/IVANBinaries/bin/Release/'
#output_path_str = 'D:/VolumeData/VesselExperiments/'
output_path_str = 'F:/VolumeData/VesselExperiments/'

gaussian_tube_exe_str = exe_path_str + 'TestCircularFlatConvolvedTube.exe'
noise_exe_str = exe_path_str + 'UtilityGaussianNoiseAdder.exe'
gaussian_tube_output_str_root = output_path_str + 'TestObjects/New/' + 'CircularFlatConvolvedTube_'

# We generate scales as sigma_i = sigma_0 * factor^i where factor is a number > 1.0

#numScales = 6
#scaleFactor = 1.5
#zeroScale = 1.0

numScales = 12
scaleFactor = 1.225
zeroScale = 0.75

currentScale = zeroScale
sigma = 0.7 # for the convolution

tubeSpacing = 1.0
maxValue = 255.0

noise_levels = [ 10, 25, 50, 75 ]

for i in range( 0, numScales ):

  tubeSpacingModf = math.modf( tubeSpacing )
  tubeScaleModf = math.modf( currentScale )
  sigmaModf = math.modf( sigma )
  
  #if currentScale <= 3.0:
  #  tubeHeightStr = '41'
  #elif currentScale > 3.0 and currentScale <= 5.0:
  #  tubeHeightStr = '81'
  #elif currentScale > 3.0 and currentScale <= 8.0:
  #   tubeHeightStr = '121'
  #else:
  #   tubeHeightStr = '161'
  
  tubeHeightStr = '81'
  
  gaussian_tube_output_str = gaussian_tube_output_str_root + \
    str( int( tubeScaleModf[1] ) ) + '_' + str( int( tubeScaleModf[0] * 100.0 ) ) + '_Gs' + \
    str( int( sigmaModf[1] ) ) + '_' + str( int( sigmaModf[0] * 100.0 ) ) + '.mhd'
    
  print gaussian_tube_exe_str, gaussian_tube_output_str, str( currentScale ), tubeHeightStr, str( sigma ), str( maxValue ), str( tubeSpacing )
  
  subprocess.call( [ gaussian_tube_exe_str, gaussian_tube_output_str, str( currentScale ), tubeHeightStr, str( sigma ), str( maxValue ), str( tubeSpacing ) ] )

  currentScale = currentScale * scaleFactor
  
  # Now create noisy versions
    
  for k in range ( 0, len( noise_levels ) ):
      
    gaussian_tube_output_noise_str = gaussian_tube_output_str_root + str( int( tubeScaleModf[1] ) ) + '_' + \
      str( int( tubeScaleModf[0] * 100.0 ) ) + '_noise' + str( noise_levels[k] ) + '.mhd'
    print noise_exe_str, gaussian_tube_output_str, gaussian_tube_output_noise_str, str( noise_levels[k] ), str( maxValue )
    subprocess.call( [ noise_exe_str, gaussian_tube_output_str, gaussian_tube_output_noise_str, str( noise_levels[k] ), str( maxValue ) ] )