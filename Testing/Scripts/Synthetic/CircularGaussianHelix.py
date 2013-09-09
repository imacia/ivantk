import subprocess
import math

objects_exe_path_str = 'C:/SDKs/ITKVesselBin/TestObjects/Release/'
output_path_str = 'D:/VolumeData/VesselExperiments/'

gaussian_helix_exe_str = objects_exe_path_str + 'CircularGaussianHelix.exe'
gaussian_helix_output_str_root = output_path_str + 'TestObjects/' + 'CircularGaussianHelix'

#helixScales = [0.75, 1.0, 1.25, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0]
#helixRadii  = [1.0, 2.5, 5.0, 10.0, 25.0, 50.0, 100.0, 250.0]

helixScales = [1.0, 5.0, 10.0]
helixRadii  = [2.5, 10.0, 25.0, 100.0]
# For this experiment use same pitch as radius
helixPitch  = [2.5, 10.0, 25.0, 100.0]

maxValue = 255.0
startAngle = -45.0
endAngle = 45.0
#numberOfPts = 200

for i in range( len( helixScales ) ):

  helixSpacing = 0.2 * helixScales[i]
  
  if helixSpacing < 0.25 : # otherwise too much memory involved
    helixSpacing = 0.25
  
  helixSpacingModf = math.modf( helixSpacing )
  helixScaleModf = math.modf( helixScales[i] )
  
  if helixScales[i] >= 5.0:
    fullCircle = 1
  else:
    fullCircle = 0
  
  for j in range( len( helixRadii ) ):
  
    helixRadiusModf = math.modf( helixRadii[j] )
    helixPitchModf  = math.modf( helixPitch[j] )
    
    gaussian_helix_output_str = gaussian_helix_output_str_root + '_sigma' + \
    str( int( helixScaleModf[1] ) ) + '_' + str( int( helixScaleModf[0] * 100.0 ) ) + '_R' + \
    str( int( helixRadiusModf[1] ) ) + '_' + str( int( helixRadiusModf[0] * 100.0 ) ) + '_H' + \
    str( int( helixPitchModf[1] ) ) + '_' + str( int( helixPitchModf[0] * 100.0 ) ) + '_sp' + \
    str( int( helixSpacingModf[1] ) ) + '_' + str( int( helixSpacingModf[0] * 100.0 ) ) + '.mhd'
    
    print gaussian_helix_exe_str, gaussian_helix_output_str, str( helixScales[i] ), str( helixRadii[j] ), str( helixPitch[j] ), '2', str( helixSpacing ), str( startAngle ), str( endAngle ), '0', str( maxValue ), fullCircle
    
    subprocess.call( [ gaussian_helix_exe_str, gaussian_helix_output_str, str( helixScales[i] ), str( helixRadii[j] ), str( helixPitch[j] ), '2', str( helixSpacing ), str( startAngle ), str( endAngle ), '0', str( maxValue ), str( fullCircle ) ] )
