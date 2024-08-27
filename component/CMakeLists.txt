cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS "pl_adxl355_types.cpp" "pl_adxl355.cpp" INCLUDE_DIRS "include" REQUIRES "pl_spi")