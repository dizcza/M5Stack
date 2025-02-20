file(GLOB_RECURSE SOURCES "src/*.*")
idf_component_register(
    SRCS ${SOURCES} 
    INCLUDE_DIRS src "$ENV{IDF_PATH}/components/bt/host/bluedroid/api/include/api"
    REQUIRES arduino
)

list(APPEND DEFINITIONS "ARDUINO_TTGO_T1")

target_compile_definitions(${COMPONENT_TARGET} PUBLIC ${DEFINITIONS})
