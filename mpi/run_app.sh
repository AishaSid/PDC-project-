#!/bin/bash
# Compile the C++ file with profiling support
g++ -pg -o main main.cpp Sosp_Update.cpp Mosp_Update.cpp
# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "Compilation successful. Running program with profiling..."
    ./main
    echo "Generating profiling report with gprof..."
    gprof main gmon.out > profile_report.txt
    echo "Profiling completed. Results saved to profile_report.txt"
else
    echo "Compilation failed."
fi