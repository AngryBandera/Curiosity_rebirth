set(srcs "my_platform.cpp")

set(requires "bluepad32" "btstack" "motors")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
