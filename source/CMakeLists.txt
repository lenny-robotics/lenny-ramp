if (${LENNY_PROJECT}_BUILD_APPS)
    add_subdirectory(apps)
endif ()
if (NOT TARGET lenny::mamp)
    add_subdirectory(mamp)
endif ()
if (NOT TARGET lenny::samp)
    add_subdirectory(samp)
endif ()
