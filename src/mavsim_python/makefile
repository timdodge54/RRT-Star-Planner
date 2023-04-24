.PHONY:
    install
    clean
    lint
    help

install:            ## Install the project
    pip install -f requirements.txt -e .

clean:              ## Remove the project and its dependencies from your python environment. It also removes any downloaded cached files
    pip uninstall -y mav_sim
    pip cache purge

lint:               ## Run pylint over the project
    mypy --install-types --config-file mypy.ini --package mav_sim
    mypy --install-types --config-file mypy.ini scripts/
    pydocstyle -e --count mav_sim scripts
    pylint --jobs 0 --rcfile .pylintrc mav_sim scripts/

format:             ## Auto format the current code
    black mav_sim scripts
    isort .

help:               ## Print the help statements
    @findstr /x "^[a-z]*[a-z_]*:.*##.*$" makefile
