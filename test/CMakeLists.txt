add_executable( image_retrieve_test image_retrieve_test.cpp )
target_link_libraries( image_retrieve_test localization ${OpenCV_LIBS} ${DBoW3_LIBS} )

