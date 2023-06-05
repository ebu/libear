set(current_dir ${CMAKE_CURRENT_LIST_DIR})

function(generate_points_file infile outfile)
  file(READ "${infile}" contents)
  string(REGEX REPLACE "([^ \n]+) +([^ \n]+)\n" "{\\1, \\2},\n" POINTS
                       "${contents}")

  configure_file("${current_dir}/points_file.cpp.in" "${outfile}" @ONLY)

  set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS "${infile}")
endfunction()
