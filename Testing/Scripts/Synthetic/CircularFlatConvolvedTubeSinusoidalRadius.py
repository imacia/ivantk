import subprocess
import math

exe_path_str = 'C:/SDKs/IVANBinaries/bin/Release/'
output_path_str = 'D:/VolumeData/VesselExperiments/'
#output_path_str = 'F:/VolumeData/VesselExperiments/'

flat_tube_exe_str = exe_path_str + 'TestCircularBarConvolvedTubeSinusoidalRadius.exe'
noise_exe_str = exe_path_str + 'UtilityGaussianNoiseAdder.exe'
flat_tube_output_str_root = output_path_str + 'TestObjects/' + 'CircularGaussianTubeSin2Lobes_'

# We generate scales as sigma_i = sigma_0 * factor^i where factor is a number > 1.0
numScales = 6
scaleFactor = 1.5
zeroScale = 1.0
currentScale = zeroScale

minRadii = [ 1.0, 2.25 ]
maxRadii = [ 3.375, 5.0625 ]

numLobes = 2
convSigma = 0.7
tubeSpacing = 1.0
maxValue = 255.0
tubeHeight = 61

noise_levels = [ 10, 25, 50, 75 ]

for i in range( len( minRadii ) ):

  tubeSpacingModf  = math.modf( tubeSpacing )
  tubeRadiusMinModf = math.modf( minRadii[i] )
  tubeRadiusMaxModf = math.modf( maxRadii[i] )
  
  flat_tube_output_str = flat_tube_output_str_root + \
    str( int( tubeRadiusMinModf[1] ) ) + '_' + str( int( tubeRadiusMinModf[0] * 100.0 ) ) + \
    '-' + str( int( tubeRadiusMaxModf[1] ) ) + '_' + str( int( tubeRadiusMaxModf[0] * 100.0 ) ) + '.mhd'
    
  print flat_tube_exe_str, flat_tube_output_str, str( minRadii[i] ), str( maxRadii[i] ), str( tubeHeight ), str( numLobes ), str( convSigma ), \
  str( maxValue ), str( tubeSpacing )
  
  subprocess.call( [ flat_tube_exe_str, flat_tube_output_str, str( minRadii[i] ), str( maxRadii[i] ), str( tubeHeight ), str( numLobes ), \
  str( convSigma ), str( maxValue ), str( tubeSpacing ) ] )

  # Now create noisy versions
    
  for k in range ( 0, len( noise_levels ) ):
      
    flat_tube_output_noise_str = flat_tube_output_str_root + \
    str( int( tubeRadiusMinModf[1] ) ) + '_' + str( int( tubeRadiusMinModf[0] * 100.0 ) ) + \
    '-' + str( int( tubeRadiusMaxModf[1] ) ) + '_' + str( int( tubeRadiusMaxModf[0] * 100.0 ) ) + \
    '_noise' + str( noise_levels[k] ) + '.mhd'
    
    print noise_exe_str, flat_tube_output_str, flat_tube_output_noise_str, str( noise_levels[k] ), str( maxValue )
    subprocess.call( [ noise_exe_str, flat_tube_output_str, flat_tube_output_noise_str, str( noise_levels[k] ), str( maxValue ) ] )