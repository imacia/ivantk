import subprocess
import math

test_exe_path_str = 'C:/SDKs/ITKVesselBin/Tests/Release/'
objects_exe_path_str = 'C:/SDKs/ITKVesselBin/TestObjects/Release/'
output_path_str = 'D:/VolumeData/VesselExperiments/TestObjects/'

noise_adder_exe_str = objects_exe_path_str + 'GaussianNoiseAdder.exe'

inputFileNames = [
  'CurvedGaussianTube_1_0_xoff_2_50',
  'CurvedGaussianTube_1_0_xoff_5_0'
]

noiseLevels = [2.5, 3.75, 5.0, 7.5, 10.0, 15.0, 20.0]

for i in range( 0, len( inputFileNames ) ):
  for j in range( 0, len( noiseLevels ) ):
    noiseLevelModf = math.modf( noiseLevels[j] ) 
    inputFileName = output_path_str + inputFileNames[i] + '.mhd'
    outputFileName = output_path_str + inputFileNames[i] + '_NL_' + str( int( noiseLevelModf[1] ) ) + '_' + str( int( noiseLevelModf[0] * 100.0 ) ) + '.mhd'
    print 'Adding Noise...', inputFileName, outputFileName, 'Noise level =', noiseLevels[j] 
    subprocess.call( [ noise_adder_exe_str, inputFileName, outputFileName, str( noiseLevels[j] )] )
