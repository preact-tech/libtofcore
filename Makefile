help:
	@fgrep -h "##" $(MAKEFILE_LIST) | fgrep -v fgrep | sed -e 's/\\$$//' | sed -e 's/##//'

PHONY: build
build:   ## build libtofcore
	cmake -B build
	cmake --build build

PHONY: pytofcore
pytofcore: ## install pytofcore (need to run make build first)
	python3 -m pip uninstall -y pytofcore
	cd tofcore/wrappers/python && python3 setup.py install --user
	ln -sf ./build/tofcore/wrappers/python/pytofcore.cpython*.so  

PHONY: pytofcrust
pytofcrust: ## install pytofcrust (need to run make build first)
	python3 -m pip uninstall -y pytofcrust
	cp build/tofcrust/wrappers/python/*.so tofcrust/wrappers/python/pytofcrust
	cd tofcrust/wrappers/python && python3 setup.py install --user
	ln -sf ./build/tofcrust/wrappers/python/pytofcrust.cpython*.so  

PHONY: functional_tests
functional_tests: ## runs python functional tests (needs pytofcore and pytofcrust installed and oasis device connected to PC)
	python3 -m pytest -m "functional" -v .

PHONY: unit_tests
unit_tests: ## runs unit tests (make sure no oasis is connected to PC)
	python3 -m pytest -m "not functional and not sdram_selftest" -v .


PHONY : cpp_unit_test
cpp_unit_test:
	ctest --test-dir ./build/tofcore 


PHONY: clean
clean: ## cleans environment of build artifacts
	python3 -m pip uninstall -y pytofcrust pytofcore
	rm -rf build
	rm -f tofcore/wrappers/python/pytofcore/*.so
	rm -f tofcrust/wrappers/python/pytofcrust/*.so
	rm -rf tofcore/wrappers/python/*.egg-info
	rm -rf tofcore/wrappers/python/build
	rm -rf tofcore/wrappers/python/dist
	rm -rf tofcrust/wrappers/python/*.egg-info
	rm -rf tofcrust/wrappers/python/build
	rm -rf tofcrust/wrappers/python/dist
