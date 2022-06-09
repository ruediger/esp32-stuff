find_package (Python COMPONENTS Interpreter)

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
  add_custom_target(
    font5x5 ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/font5x5.hpp)

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/font5x5.hpp
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generate_glyphs.py
    COMMAND
    "${Python_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/generate_glyphs.py" --multiplier 1 > ${CMAKE_CURRENT_BINARY_DIR}/font5x5.hpp
    VERBATIM)

  add_custom_target(
    font10x10 ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/font10x10.hpp)

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/font10x10.hpp
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generate_glyphs.py
    COMMAND
    "${Python_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/generate_glyphs.py" --multiplier 2 > ${CMAKE_CURRENT_BINARY_DIR}/font10x10.hpp
    VERBATIM)

  add_custom_target(
    font30x30 ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/font30x30.hpp)

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/font30x30.hpp
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generate_glyphs.py
    COMMAND
    "${Python_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/generate_glyphs.py" --multiplier 6 > ${CMAKE_CURRENT_BINARY_DIR}/font30x30.hpp
    VERBATIM)

  set_property(DIRECTORY "${COMPONENT_DIR}" APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES font5x5.hpp)
endif()

idf_component_register(SRCS "ssd1306_main.cpp"
                    INCLUDE_DIRS "." "${CMAKE_CURRENT_BINARY_DIR}")
