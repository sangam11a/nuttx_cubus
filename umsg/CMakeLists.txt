nuttx_add_kernel_library(umsg umsg.c)
nuttx_add_subdirectory()
target_include_directories(umsg PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})