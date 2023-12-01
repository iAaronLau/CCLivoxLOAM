# CloudCompare example for standard plugins

# REPLACE ALL 'SubSampleTools2' OCCURENCES BY YOUR PLUGIN NAME
# AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_CCLivoxLOAM "Install CCLivoxLOAM" ON )

# set(DEMO_NAME CCLivoxLOAM )

if ( PLUGIN_CCLivoxLOAM )
	project( CCLivoxLOAM )
	 
	AddPlugin( NAME ${PROJECT_NAME} )
		
	add_subdirectory( include )
	add_subdirectory( src )
	#add_subdirectory( Eigen )

    add_subdirectory(sdk_core)
    # add_subdirectory(lib) 

	# link_libraries("lib/serial.lib"　"lib/livox_sdk_static.lib")
	# link_libraries("lib")
	# target_link_libraries(${PROJECT_NAME}  serial.lib )
	# target_link_libraries(${PROJECT_NAME}  livox_sdk_static.lib )

endif()
