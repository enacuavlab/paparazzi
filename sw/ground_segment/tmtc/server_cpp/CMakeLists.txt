cmake_minimum_required(VERSION 3.20)
project(paparazzi_server_cpp LANGUAGES CXX)

find_package(pprzlink++ CONFIG REQUIRED COMPONENTS core ivy)
find_package(tinyxml2 REQUIRED)
find_package(CLI11 CONFIG REQUIRED)
find_package(PROJ CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost CONFIG REQUIRED)
if(PROJ_VERSION VERSION_LESS 8.0)
  message(FATAL_ERROR "PROJ 8 or newer is required for topocentric conversions")
endif()

add_library(server_core
  src/server.cpp
  src/configuration.cpp
  src/server_configuration.cpp
  src/telemetry.cpp
  src/commands.cpp
  src/publishing.cpp
  src/monitoring.cpp
  src/logging.cpp
  src/telemetry_json.cpp
  src/legacy_text.cpp
  src/geodesy.cpp)
target_include_directories(server_core PUBLIC src)
target_link_libraries(server_core PUBLIC pprzlink::core tinyxml2::tinyxml2)
target_link_libraries(server_core PRIVATE PROJ::proj)
target_compile_features(server_core PUBLIC cxx_std_23)
target_compile_options(server_core PRIVATE -Wall -Wextra -Wpedantic)
add_executable(server_cpp src/main.cpp src/application.cpp src/pprz_connection_ivy.cpp)
target_link_libraries(server_cpp PRIVATE server_core pprzlink::ivy CLI11::CLI11 Boost::headers Threads::Threads)
target_compile_options(server_cpp PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS server_cpp RUNTIME DESTINATION bin)

include(CTest)
if(BUILD_TESTING)
  # Compile configuration on its own: it must not depend on the server or PprzLink.
  add_executable(configuration_test tests/configuration_test.cpp src/configuration.cpp)
  target_include_directories(configuration_test PRIVATE src)
  target_compile_features(configuration_test PRIVATE cxx_std_23)
  target_link_libraries(configuration_test PRIVATE tinyxml2::tinyxml2)
  target_compile_options(configuration_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME server_configuration COMMAND configuration_test)
  add_executable(geodesy_test tests/geodesy_test.cpp)
  target_link_libraries(geodesy_test PRIVATE server_core)
  add_test(NAME server_geodesy COMMAND geodesy_test)
  add_executable(server_test tests/server_test.cpp)
  target_link_libraries(server_test PRIVATE server_core)
  add_test(NAME server_core COMMAND server_test ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)
  option(SERVER_CPP_IVY_TEST "Run the local Ivy integration test (requires python3-ivy and networking)" OFF)
  if(SERVER_CPP_IVY_TEST)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME server_ivy COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/ivy_smoke.py $<TARGET_FILE:server_cpp>
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)
    add_test(NAME server_ivy_server_first COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/ivy_smoke.py $<TARGET_FILE:server_cpp>
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../.. --server-first)
    set_tests_properties(server_ivy server_ivy_server_first PROPERTIES TIMEOUT 45)
  endif()
  option(SERVER_CPP_REPLAY_TEST "Run replay session restart integration tests (requires python3-ivy and networking)" OFF)
  if(SERVER_CPP_REPLAY_TEST)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME server_replay_restart COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/replay_restart.py $<TARGET_FILE:server_cpp>
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)
    add_test(NAME server_replay_restart_server_first COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/replay_restart.py $<TARGET_FILE:server_cpp>
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../.. --server-first)
    set_tests_properties(server_replay_restart server_replay_restart_server_first PROPERTIES TIMEOUT 60)
  endif()
endif()
