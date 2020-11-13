cmake_minimum_required(VERSION 3.18.4)

project(MY680 VERSION 1.0)

add_executable(${PROJECT_NAME} bme680.c main.c)
