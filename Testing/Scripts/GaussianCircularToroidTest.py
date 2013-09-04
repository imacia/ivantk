from datetime import date
import subprocess

path_name = 'C:\\Binaries\\Libraries\\ivan\\\\LumenSegmentation\\LumenRegionGrowing\\Release\\'

# Argument 0: Execution file
exe_name = 'LumenAdaptive2DNeighborhoodConnected.exe'
full_exe_name = path_name + exe_name

# Argumen 1: Input image directory ( DICOM folder )
patient = 'Littlebury^Agamemnon'
data_repository_path = 'C:\\Data\\SERENA\\Anonymous\\' + patient + '\\PostOperatory\\CT_ContrastEnhanced\\'
input_filename = 'ROI_' + patient + '_SE2'
format = '.mhd'
input_filename_path = data_repository_path + input_filename + format

print( input_filename_path )

# Argument 2: Output Directory
today = date.today()
output_repository_path = 'C:\\OutputData\\SERENA\\Metadata\\LumenAdaptive2DNeighborhoodConnected\\' + patient + '\\PostOperatory\\CT_ContrastEnhanced\\new\\'
 
# Argument 4: SeedX
seedX = 329

# Argument 5: SeedY
seedY = 328

# Argument 6: SeedZ
seedZ = 1

# Argument 7: Lower Threshold
lowerThreshold = 200

# Argument 8: [ Upper Threshold ]
# The default value is the maximum posible value
#upperThreshold = 450

# Argument 9:
radiusX = 2

# Argument 10:
radiusY = 2


# Argument 3: Output Filename

parameter_list = '_l_' + str ( lowerThreshold ) + '_u_' + 'max' +'_seedX_' + str( seedX ) + '_seedY_' + str( seedY ) + '_seedZ_' + str( seedZ ) + '_r_' + str( radiusX ) + '_' + str( radiusY ) 
output_filename = today.isoformat() + input_filename + parameter_list + format

subprocess.call( [ full_exe_name , input_filename_path ,  output_repository_path , output_filename , str( seedX ) , str( seedY ) , str( seedZ ) ,  str( radiusX ) , str( radiusY ) , str( lowerThreshold ) ] )        
      
input()
