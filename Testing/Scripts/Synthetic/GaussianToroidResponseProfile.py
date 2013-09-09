import subprocess
import math

exe_path_str = 'C:/SDKs/ITKVesselBin/Experiments/TestObjects/Release/'
exe_name = exe_path_str + 'GaussianToroidResponseProfile.exe'
output_path_root = 'D:/VolumeData/VesselExperiments/'

#sigmas = [0.75, 1.0, 1.25, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0]
#torusRadii = [1.0, 2.5, 5.0, 10.0, 25.0, 50.0, 100.0, 250.0]
sigmas = [1.0, 5.0, 10.0]
torusRadii = [2.5, 10.0, 50.0, 250.0]

for i in range( 0, len( sigmas ) ):

  toroidSigmaModf = math.modf( sigmas[i] )
  
  for j in range( 0, len( torusRadii ) ):
  
    toroidRadiusModf = math.modf( torusRadii[j] )
    
    toroidSpacing = 0.2 * sigmas[i]
  
    if toroidSpacing < 0.25 : # otherwise too much memory involved
      toroidSpacing = 0.25
  
    toroidSpacingModf = math.modf( toroidSpacing )
    
    if sigmas[i] >= 5.0:
      fullCircle = 1
    else:
      fullCircle = 0
    
    input_image_file = output_path_root + 'TestObjects/' + 'GaussianToroid' + '_sigma' + \
    str( int( toroidSigmaModf[1] ) ) + '_' + str( int( toroidSigmaModf[0] * 100.0 ) ) + '_R' + \
    str( int( toroidRadiusModf[1] ) ) + '_' + str( int( toroidRadiusModf[0] * 100.0 ) ) + '_sp' + \
    str( int( toroidSpacingModf[1] ) ) + '_' + str( int( toroidSpacingModf[0] * 100.0 ) ) + '.mhd'
    
    output_file = output_path_root + 'ResponseProfiles/' + 'GaussianToroidResponseProfile_' + str( int( toroidSigmaModf[1] ) ) + '_' + \
    str( int( toroidSigmaModf[0] * 100.0 ) ) + '_R' + \
    str( int( toroidRadiusModf[1] ) ) + '_' + str( int( toroidRadiusModf[0] * 100.0 ) ) + '_sp' + \
    str( int( toroidSpacingModf[1] ) ) + '_' + str( int( toroidSpacingModf[0] * 100.0 ) ) + '.txt'
    
    print 'Processing...', input_image_file, output_file, sigmas[i], torusRadii[j], 2, fullCircle
    subprocess.call( [ exe_name, input_image_file, output_file, str( sigmas[i] ), str( torusRadii[j] ), '2', str( fullCircle ) ] )
