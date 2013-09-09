import subprocess
import math

exe_path_str = 'C:/Binaries/Libraries/ivan-x64/trunk/bin/Release/'
output_path_str = 'C:/Data/DISTANCE/Synthetic/CircularBarToroid/'

circular_bar_toroid_exe_str = exe_path_str + 'TestCircularBarToroid.exe'

print( 'circular_bar_toroid_exe_str' )

print( circular_bar_toroid_exe_str )

tubeRadius = 5
toroidRadius = 50
direction = 0
imageSpacing = 1
startAngle = 0
endAngle = 180
maxValue = 1

output_filename_path = output_path_str + 'Modified_MorePoints_TubeR_' + str( tubeRadius ) + 'toroidR_' + str( toroidRadius )+ '_maxValue_' + str( maxValue ) + '_spacing_' + str( imageSpacing ) + '_d_' + str( direction ) + '_a1_' + str( startAngle ) + '_a2_' + str( endAngle )+ '.mhd'

subprocess.call( [ circular_bar_toroid_exe_str, output_filename_path, str( tubeRadius ), str( toroidRadius ), str( direction ), str( imageSpacing ), str( startAngle ), str( endAngle ), str( maxValue ) ] )

input()