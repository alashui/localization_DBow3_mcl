add_library( localization SHARED
    config.cpp
)

target_link_libraries( localization
    ${THIRD_PARTY_LIBS} 
)


add_executable( vocabulary_training vocabulary_training.cpp )
target_link_libraries( vocabulary_training localization ${OpenCV_LIBS} ${DBoW3_LIBS} )

add_executable( creat_database creat_database.cpp )
target_link_libraries( creat_database localization ${OpenCV_LIBS} ${DBoW3_LIBS} )

add_executable( image_retrieve image_retrieve.cpp )
target_link_libraries( image_retrieve localization ${OpenCV_LIBS} ${DBoW3_LIBS} )
