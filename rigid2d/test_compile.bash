#!/bin/bash

# compile the program
g++ -Wall -Wextra -g -std=c++17 -o rigid2d_test main.cpp rigid2d.cpp

# clear contents of code output file
> output.txt

# run code with test inputs adn store results in output.txt
./rigid2d_test < test1_input.txt >> output.txt

# test if results match 
cmp -s output.txt test1_answer.txt
status=$?
if [[ $status = 0 ]]; then
    echo "Success!"
else
    echo "Failure!"
fi
