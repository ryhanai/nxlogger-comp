add_subdirectory(nxlogger_comp)

MAP_ADD_STR(hdrs "${PROJECT_NAME}/" headers)
set(headers ${headers} PARENT_SCOPE)
