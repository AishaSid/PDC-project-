#!/bin/bash
# Compile the C++ file
g++ -o main main.cpp Sosp_Update.cpp Mosp_Update.cpp
# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "Compilation successful. Running program..."
    ./main
else
    echo "Compilation failed."
fi