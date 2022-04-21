#!/bin/bash

# sphinx-apidoc -o ./source/ ../python_driver/
make html
firefox build/html/index.html
