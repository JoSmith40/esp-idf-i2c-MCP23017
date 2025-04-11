cmake_minimum_required(VERSION 3.16)

idf_component_register(SRCS "mcp23017.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver)
