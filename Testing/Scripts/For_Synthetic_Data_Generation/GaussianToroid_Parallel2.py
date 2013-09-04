import subprocess
import math

objects_exe_path_str = 'C:/SDKs/ITKVesselBin/TestObjects/Release/'
output_path_str = 'D:/VolumeData/VesselExperiments/'

gaussian_toroid_exe_str = objects_exe_path_str + 'GaussianToroid.exe'
gaussian_toroid_output_str_root = output_path_str + 'TestObjects/' + 'GaussianToroid'

toroidScales = [1.25, 1.5, 2.0]
toroidRadii  = [1.0, 2.5, 5.0, 10.0, 25.0, 50.0, 100.0, 250.0]

maxValue = 255.0
startAngle = -45.0
endAngle = 45.0
#numberOfPts = 200

for i in range( len( toroidScales ) ):

  toroidSpacing = 0.2 * toroidScales[i]
  
  if toroidSpacing < 0.25 : # otherwise too much memory involved
    toroidSpacing = 0.25
  
  toroidSpacingModf = math.modf( toroidSpacing )
  toroidScaleModf = math.modf( toroidScales[i] )
  
  for j in range( len( toroidRadii ) ):
  
    toroidRadiusModf = math.modf( toroidRadii[j] )
    gaussian_toroid_output_str = gaussian_toroid_output_str_root + '_sigma' + \
    str( int( toroidScaleModf[1] ) ) + '_' + str( int( toroidScaleModf[0] * 100.0 ) ) + '_R' + \
    str( int( toroidRadiusModf[1] ) ) + '_' + str( int( toroidRadiusModf[0] * 100.0 ) ) + '_sp' + \
    str( int( toroidSpacingModf[1] ) ) + '_' + str( int( toroidSpacingModf[0] * 100.0 ) ) + '.mhd'
    print gaussian_toroid_exe_str, gaussian_toroid_output_str, str( toroidScales[i] ), str( toroidRadii[j] ), '2', str( toroidSpacing ), str( startAngle ), str( endAngle ), str( maxValue )
    subprocess.call( [ gaussian_toroid_exe_str, gaussian_toroid_output_str, str( toroidScales[i] ), str( toroidRadii[j] ), '2', str( toroidSpacing ), str( startAngle ), str( endAngle ), str( maxValue ) ] )
