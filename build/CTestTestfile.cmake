# CMake generated Testfile for 
# Source directory: /home/utonto/ros2_ws/src/robot_slam
# Build directory: /home/utonto/ros2_ws/src/robot_slam/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cppcheck "/usr/bin/python3" "-u" "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/cppcheck.xunit.xml" "--package-name" "robot_slam" "--output-file" "/home/utonto/ros2_ws/src/robot_slam/build/ament_cppcheck/cppcheck.txt" "--command" "/home/utonto/ros2_jazzy/install/ament_cppcheck/bin/ament_cppcheck" "--xunit-file" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/cppcheck.xunit.xml")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/utonto/ros2_ws/src/robot_slam" _BACKTRACE_TRIPLES "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/utonto/ros2_jazzy/install/ament_cmake_cppcheck/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;66;ament_add_test;/home/utonto/ros2_jazzy/install/ament_cmake_cppcheck/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;87;ament_cppcheck;/home/utonto/ros2_jazzy/install/ament_cmake_cppcheck/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;46;ament_package;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;0;")
add_test(flake8 "/usr/bin/python3" "-u" "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/flake8.xunit.xml" "--package-name" "robot_slam" "--output-file" "/home/utonto/ros2_ws/src/robot_slam/build/ament_flake8/flake8.txt" "--command" "/home/utonto/ros2_jazzy/install/ament_flake8/bin/ament_flake8" "--xunit-file" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/flake8.xunit.xml")
set_tests_properties(flake8 PROPERTIES  LABELS "flake8;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/utonto/ros2_ws/src/robot_slam" _BACKTRACE_TRIPLES "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/utonto/ros2_jazzy/install/ament_cmake_flake8/share/ament_cmake_flake8/cmake/ament_flake8.cmake;69;ament_add_test;/home/utonto/ros2_jazzy/install/ament_cmake_flake8/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;19;ament_flake8;/home/utonto/ros2_jazzy/install/ament_cmake_flake8/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;46;ament_package;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/lint_cmake.xunit.xml" "--package-name" "robot_slam" "--output-file" "/home/utonto/ros2_ws/src/robot_slam/build/ament_lint_cmake/lint_cmake.txt" "--command" "/home/utonto/ros2_jazzy/install/ament_lint_cmake/bin/ament_lint_cmake" "--xunit-file" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/utonto/ros2_ws/src/robot_slam" _BACKTRACE_TRIPLES "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/utonto/ros2_jazzy/install/ament_cmake_lint_cmake/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/home/utonto/ros2_jazzy/install/ament_cmake_lint_cmake/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/home/utonto/ros2_jazzy/install/ament_cmake_lint_cmake/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;46;ament_package;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;0;")
add_test(pep257 "/usr/bin/python3" "-u" "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/pep257.xunit.xml" "--package-name" "robot_slam" "--output-file" "/home/utonto/ros2_ws/src/robot_slam/build/ament_pep257/pep257.txt" "--command" "/home/utonto/ros2_jazzy/install/ament_pep257/bin/ament_pep257" "--xunit-file" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/pep257.xunit.xml")
set_tests_properties(pep257 PROPERTIES  LABELS "pep257;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/utonto/ros2_ws/src/robot_slam" _BACKTRACE_TRIPLES "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/utonto/ros2_jazzy/install/ament_cmake_pep257/share/ament_cmake_pep257/cmake/ament_pep257.cmake;41;ament_add_test;/home/utonto/ros2_jazzy/install/ament_cmake_pep257/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;18;ament_pep257;/home/utonto/ros2_jazzy/install/ament_cmake_pep257/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;46;ament_package;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3" "-u" "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/uncrustify.xunit.xml" "--package-name" "robot_slam" "--output-file" "/home/utonto/ros2_ws/src/robot_slam/build/ament_uncrustify/uncrustify.txt" "--command" "/home/utonto/ros2_jazzy/install/ament_uncrustify/bin/ament_uncrustify" "--xunit-file" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/utonto/ros2_ws/src/robot_slam" _BACKTRACE_TRIPLES "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/utonto/ros2_jazzy/install/ament_cmake_uncrustify/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;85;ament_add_test;/home/utonto/ros2_jazzy/install/ament_cmake_uncrustify/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;43;ament_uncrustify;/home/utonto/ros2_jazzy/install/ament_cmake_uncrustify/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;46;ament_package;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/xmllint.xunit.xml" "--package-name" "robot_slam" "--output-file" "/home/utonto/ros2_ws/src/robot_slam/build/ament_xmllint/xmllint.txt" "--command" "/home/utonto/ros2_jazzy/install/ament_xmllint/bin/ament_xmllint" "--xunit-file" "/home/utonto/ros2_ws/src/robot_slam/build/test_results/robot_slam/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/utonto/ros2_ws/src/robot_slam" _BACKTRACE_TRIPLES "/home/utonto/ros2_jazzy/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/utonto/ros2_jazzy/install/ament_cmake_xmllint/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/home/utonto/ros2_jazzy/install/ament_cmake_xmllint/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/home/utonto/ros2_jazzy/install/ament_cmake_xmllint/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/utonto/ros2_jazzy/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/utonto/ros2_jazzy/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;46;ament_package;/home/utonto/ros2_ws/src/robot_slam/CMakeLists.txt;0;")
