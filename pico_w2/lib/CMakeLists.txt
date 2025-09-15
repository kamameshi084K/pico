# "my_test_lib" という名前のライブラリを作成
# 元になるソースファイルは "test/test.cpp"
add_library(my_test_lib
    test/test.cpp
)

# このライブラリがPicoの標準関数を使うことを指定
target_link_libraries(my_test_lib
    pico_stdlib
)

# ヘッダーファイルが格納されている "test" フォルダを
# インクルードパスとして公開(PUBLIC)する
target_include_directories(my_test_lib PUBLIC test)