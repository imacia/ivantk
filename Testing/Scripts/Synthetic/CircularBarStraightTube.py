import subprocess
import math

exe_path_str = 'C:/Binaries/Libraries/ivan-x64/trunk/bin/Release/'
output_path_str = 'C:/Data/DISTANCE/Synthetic/CircularBarStraight/'

circular_bar_straight_exe_str = exe_path_str + 'TestCircularBarStraightTube.exe'

print( circular_bar_straight_exe_str )

tubeRadius = 10
tubeHeight = 50
maxValue = 1
imageSpacing = 1

output_filename_path = output_path_str + 'neew_TubeR_' + str( tubeRadius ) + '_tubeH_' + str( tubeHeight ) + '_maxValue_' + str( maxValue ) + '_spacing_' + str( imageSpacing ) + '.mhd'

subprocess.call( [ circular_bar_straight_exe_str, output_filename_path, str( tubeRadius ), str( tubeHeight ), str( maxValue ), str( imageSpacing) ] )

input()