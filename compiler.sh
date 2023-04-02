parametros=""
for var in "$@" 
do
    parametros+="$var;"
done

cmake .. -DCMAKE_ARGS="$parametros"
cmake --build .
