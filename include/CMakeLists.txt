
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/CCLivoxLOAM.h
		${CMAKE_CURRENT_LIST_DIR}/CCLivoxLOAMDlg.h
		${CMAKE_CURRENT_LIST_DIR}/LdsLidar.h
		${CMAKE_CURRENT_LIST_DIR}/CCRegistrationTools.h
		${CMAKE_CURRENT_LIST_DIR}/SensorReader.h
		${CMAKE_CURRENT_LIST_DIR}/Serial.h
		${CMAKE_CURRENT_LIST_DIR}/MiniCloudProcesser.h

)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
