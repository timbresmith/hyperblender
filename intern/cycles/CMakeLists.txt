
set(CYCLES_INSTALL_PATH "scripts/addons/cycles")
set(WITH_CYCLES_BLENDER ON)

# External Libraries

include(cmake/external_libs.cmake)

# Build Flags

if(WITH_RAYOPTIMIZATION AND SUPPORT_SSE_BUILD)
	set(WITH_CYCLES_OPTIMIZED_KERNEL ON)
endif()

if(WIN32 AND MSVC)
	# there is no /arch:SSE3, but intrinsics are available anyway
	set(CYCLES_SSE2_KERNEL_FLAGS "/arch:SSE2 /fp:fast -D_CRT_SECURE_NO_WARNINGS /Gs-")
	set(CYCLES_SSE3_KERNEL_FLAGS "/arch:SSE2 /fp:fast -D_CRT_SECURE_NO_WARNINGS /Gs-")

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast -D_CRT_SECURE_NO_WARNINGS /Gs-")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Ox")
	set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /Ox")
elseif(CMAKE_COMPILER_IS_GNUCC)
	set(CYCLES_SSE2_KERNEL_FLAGS "-ffast-math -msse -msse2 -mfpmath=sse")
	set(CYCLES_SSE3_KERNEL_FLAGS "-ffast-math -msse -msse2 -msse3 -mssse3 -mfpmath=sse")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	set(CYCLES_SSE2_KERNEL_FLAGS "-ffast-math -msse -msse2")
	set(CYCLES_SSE3_KERNEL_FLAGS "-ffast-math -msse -msse2 -msse3 -mssse3")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")
endif()

# for OSL
if(WIN32 AND MSVC)
	set(RTTI_DISABLE_FLAGS "/GR- -DBOOST_NO_RTTI -DBOOST_NO_TYPEID")
elseif(CMAKE_COMPILER_IS_GNUCC OR (CMAKE_C_COMPILER_ID MATCHES "Clang"))
	set(RTTI_DISABLE_FLAGS "-fno-rtti -DBOOST_NO_RTTI -DBOOST_NO_TYPEID")
endif()

# Definitions and Includes

add_definitions(${BOOST_DEFINITIONS} ${OPENIMAGEIO_DEFINITIONS})

add_definitions(-DCCL_NAMESPACE_BEGIN=namespace\ ccl\ {)
add_definitions(-DCCL_NAMESPACE_END=})

if(WITH_CYCLES_OPTIMIZED_KERNEL)
	add_definitions(-DWITH_OPTIMIZED_KERNEL)
endif()

if(WITH_CYCLES_NETWORK)
	add_definitions(-DWITH_NETWORK)
endif()

if(WITH_CYCLES_OSL)
	add_definitions(-DWITH_OSL)
	add_definitions(-DOSL_STATIC_LIBRARY)
	include_directories(${OSL_INCLUDES})
endif()

add_definitions(-DWITH_OPENCL)
add_definitions(-DWITH_CUDA)
add_definitions(-DWITH_MULTI)

include_directories(
	SYSTEM
	${BOOST_INCLUDE_DIR}
	${OPENIMAGEIO_INCLUDE_DIRS}
	${OPENIMAGEIO_INCLUDE_DIRS}/OpenImageIO
	${OPENEXR_INCLUDE_DIR})

# Subdirectories

if(WITH_CYCLES_BLENDER)
	add_subdirectory(blender)
endif()

if(WITH_CYCLES_TEST OR WITH_CYCLES_NETWORK)
	add_subdirectory(app)
endif()

add_subdirectory(bvh)
add_subdirectory(device)
add_subdirectory(doc)
add_subdirectory(kernel)
add_subdirectory(render)
add_subdirectory(subd)
add_subdirectory(util)

