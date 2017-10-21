add_executable( pose_txt_generate pose_txt_generate.cpp )
target_link_libraries( pose_txt_generate localization ${DBoW3_LIBS} )

add_executable( image_retrieve_test image_retrieve_test.cpp )
target_link_libraries( image_retrieve_test localization ${OpenCV_LIBS} ${DBoW3_LIBS} )

add_executable( map_generate_test map_generate_test.cpp )
target_link_libraries( map_generate_test localization  ${DBoW3_LIBS})

add_executable( pose_estimation_test pose_estimation_test.cpp )
target_link_libraries( pose_estimation_test localization  ${DBoW3_LIBS})


add_executable( pose_estimation_3d2d pose_estimation_3d2d.cpp )
target_link_libraries( pose_estimation_3d2d 
   ${OpenCV_LIBS}
   ${CSPARSE_LIBRARY}
   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)



