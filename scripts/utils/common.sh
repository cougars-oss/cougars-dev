#!/bin/bash
# Created by Nelson Durrant, Jan 2026

function print_info {
    echo -e "\033[0m\033[36m[ 🌊 ] $1\033[0m"
}

function print_warning {
    echo -e "\033[0m\033[33m[ 🐡 ] $1\033[0m"
}

function print_error {
    echo -e "\033[0m\033[31m[ 🦀 ] $1\033[0m"
}
