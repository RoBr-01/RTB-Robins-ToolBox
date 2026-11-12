# If there are submodules in this project, make sure they are initialised.
#
# Submodules that are uninitialized or already match the commit recorded by
# this repo are safely updated. A submodule checked out to a different
# commit than recorded (e.g. someone manually checked out another branch to
# test against) is left untouched and reported with a warning instead of
# being silently reset — "git submodule update" would otherwise discard
# that checkout with no confirmation.
function(CHECK_GIT_SUBMODULES)

    find_package(Git QUIET)
    if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git" AND EXISTS "${PROJECT_SOURCE_DIR}/.gitmodules")

        option(GIT_SUBMODULES_CHECK "Check submodules during build" ON)

        if(GIT_SUBMODULES_CHECK)

            execute_process(COMMAND ${GIT_EXECUTABLE} submodule status
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                OUTPUT_VARIABLE GIT_SUBMODULE_STATUS
                RESULT_VARIABLE GIT_SUBMOD_STATUS_RESULT
            )

            if(NOT GIT_SUBMOD_STATUS_RESULT EQUAL "0")
                message(FATAL_ERROR "git submodule status failed with ${GIT_SUBMOD_STATUS_RESULT}")
            endif()

            string(REPLACE "\n" ";" GIT_SUBMODULE_STATUS_LINES "${GIT_SUBMODULE_STATUS}")

            set(SAFE_SUBMODULE_PATHS "")
            foreach(STATUS_LINE ${GIT_SUBMODULE_STATUS_LINES})
                if(STATUS_LINE STREQUAL "")
                    continue()
                endif()

                string(SUBSTRING "${STATUS_LINE}" 0 1 STATUS_PREFIX)
                # "<sha1> <path> (<describe>)", prefix stripped above
                string(REGEX MATCH "[^ ]+ ([^ ]+)" STATUS_MATCH "${STATUS_LINE}")
                set(SUBMODULE_PATH "${CMAKE_MATCH_1}")

                if(STATUS_PREFIX STREQUAL "+")
                    message(WARNING
                        "Submodule '${SUBMODULE_PATH}' is checked out to a "
                        "different commit than this repo expects; leaving "
                        "it untouched instead of resetting it. Run "
                        "'git submodule update --init -- ${SUBMODULE_PATH}' "
                        "yourself if you want it reset.")
                else()
                    list(APPEND SAFE_SUBMODULE_PATHS "${SUBMODULE_PATH}")
                endif()
            endforeach()

            if(SAFE_SUBMODULE_PATHS)
                message("-- Git submodules found, updating")

                execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive -- ${SAFE_SUBMODULE_PATHS}
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    RESULT_VARIABLE GIT_SUBMOD_RESULT
                )

                if(NOT GIT_SUBMOD_RESULT EQUAL "0")
                    message(FATAL_ERROR "Git submodule update failed with ${GIT_SUBMOD_RESULT}")
                endif()
            endif()
        endif()
    else()
        message("-- No git submodules, not updating")
    endif()

endfunction()
