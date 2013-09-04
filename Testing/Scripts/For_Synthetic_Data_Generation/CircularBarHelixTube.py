import subprocess
import math

exe_path_str = 'C:/Binaries/Libraries/ivan-x64/trunk/bin/Release/'
output_path_str = 'C:/Data/DISTANCE/Synthetic/CircularBarHelix/'

circular_bar_helix_exe_str = exe_path_str + 'TestCircularBarHelix.exe'

print( circular_bar_helix_exe_str )

tubeRadius = 5
helixRadius = 50
unitPitch = 50
startAngle = 0
endAngle = 360
maxValue = 1
imageSpacing = 1
direction = 1

output_filename_path = output_path_str + 'TubeR_' + str( tubeRadius ) + 'helixR_' + str( helixRadius )+ 'unitP_' + str( unitPitch ) + '_a1_' + str( startAngle ) + '_a2_' + str( endAngle ) + '_maxValue_' + str( maxValue ) + '_spacing_' + str( imageSpacing ) + '_d_' + str( direction ) + '.mhd'

subprocess.call( [ circular_bar_helix_exe_str, output_filename_path, str( tubeRadius ), str( helixRadius ), str( unitPitch ), str( startAngle ), str( endAngle ), str( maxValue ), str( imageSpacing), str( direction ) ] )

input()