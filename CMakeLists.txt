cmake_minimum_required(VERSION 3.20)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex_m3.cmake")
message("[ROOT] Build Type: ${CMAKE_BUILD_TYPE}")
message("[ROOT] Toolchain File: ${CMAKE_TOOLCHAIN_FILE}")

project(DEMO LANGUAGES C CXX ASM)

add_executable(${PROJECT_NAME})

# 静态分析配置
option(ENABLE_STATIC_ANALYSIS "Enable static code analysis" ON)

if(ENABLE_STATIC_ANALYSIS)
    # 查找静态分析工具
    find_program(CPPCHECK cppcheck)
    find_program(CLANG_TIDY clang-tidy)
    
    # 创建静态分析目标
    if(CPPCHECK OR CLANG_TIDY)
        add_custom_target(static-analysis
            COMMENT "Running static code analysis"
            VERBATIM
        )
        
        # 添加cppcheck分析
        if(CPPCHECK)
            add_custom_command(TARGET static-analysis
                COMMAND ${CPPCHECK}
                    --enable=warning,performance,portability
                    --suppress=missingIncludeSystem
                    --inline-suppr
                    --quiet
                    ${CMAKE_SOURCE_DIR}/src
                COMMENT "Running cppcheck"
            )
        endif()
        
        # 添加clang-tidy分析
        if(CLANG_TIDY AND EXISTS ${CMAKE_BINARY_DIR}/compile_commands.json)
            add_custom_command(TARGET static-analysis
                COMMAND ${CLANG_TIDY} -p ${CMAKE_BINARY_DIR} ${CMAKE_SOURCE_DIR}/src/*.c
                COMMENT "Running clang-tidy"
            )
        endif()
    else()
        message(WARNING "No static analysis tools found, static-analysis target will be empty")
    endif()
else()
    message(STATUS "Static analysis disabled")
endif()


set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

# cmsis-core
target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_SOURCE_DIR}/third_party/arm/cmsis-core/Include"
)

# cmsis-compiler
target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_SOURCE_DIR}/third_party/arm/cmsis-compiler/include"
    "${CMAKE_SOURCE_DIR}/third_party/arm/cmsis-compiler/include/gcc"
)

target_sources(${PROJECT_NAME} PRIVATE
    "${CMAKE_SOURCE_DIR}/third_party/arm/cmsis-compiler/source/gcc/retarget_syscalls.c"
)

#app
add_subdirectory(src/app)

# driver
add_subdirectory(src/driver)

# STM32F4xx_HAL_Driver
add_subdirectory(third_party/st/STM32F1xx_HAL_Driver)

target_link_options(${PROJECT_NAME} PRIVATE
    -T${CMAKE_SOURCE_DIR}/stm32f103zet6.ld
    -Wl,-Map=${PROJECT_NAME}.map
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
)
