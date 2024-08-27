project(Lib)

#FIXME(dualsensectl-library) we cannot currently add the dualsensectl project directly as a library since it's created
#   as a single main cli application, until an actual usable library is found we have a simple wrapper for the cli commands instead
#add_custom_target(dualsensectl COMMAND make WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dualsensectl)
#FIXME(dualsensectl-cli): integrate steps for installing the project
