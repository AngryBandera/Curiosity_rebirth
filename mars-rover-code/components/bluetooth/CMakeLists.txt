set(srcs "my_platform.cpp")

set(requires "bluepad32" "btstack" "motors" "stepper")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
