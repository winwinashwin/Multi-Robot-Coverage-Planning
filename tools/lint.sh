#!/bin/bash

function banner() {
  local YELLOW='\033[1;33m'
  local NC='\033[0m'

  echo -e "${YELLOW}$*${NC}"
}

function list_cxx_source_files() {
  local path='./src'

  find $path \
  -path "./**/cmake-build-*" -prune -false -o \
  \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cc" -o -iname "*.cpp" \)
}

function list_python_source_files() {
  local path='./src'

  find $path \
  -path "./**/cmake-build-*" -prune -false -o \
  \( -iname "*.py" -o -iname "*.pyi" \)
}

function run_linter_clang_format() {
  local clang_format_executable="clang-format-11"

  banner "[clang-format]"

  while read -r file; do
    $clang_format_executable -style=file -i --verbose "$file"
  done < <(list_cxx_source_files)
}

function run_linter_black() {
  local path='./src'

  banner "[black]"

  while read -r file; do
    black --verbose --config ./pyproject.toml "$file"
  done < <(list_python_source_files)
}

function catkin_lint_check() {
  local path='.'

  banner "[catkin_lint]"

  catkin_lint $path
}

run_linter_clang_format
run_linter_black
catkin_lint_check
