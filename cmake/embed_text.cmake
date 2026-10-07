# cmake -DIN=<text file> -DOUT=<header> -DNAME=<identifier> -P embed_text.cmake
# Writes `inline constexpr const char* NAME = R"NAPL(<contents>)NAPL";` (used for the flatc JSON Schema).
file(READ "${IN}" content)
file(WRITE "${OUT}" "inline constexpr const char* ${NAME} = R\"NAPL(${content})NAPL\";\n")
