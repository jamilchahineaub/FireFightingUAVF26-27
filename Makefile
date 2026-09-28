.PHONY: params sdf test test-octave hooks clean

# Convert params.yaml -> params.json (MATLAB reads the JSON)
params:
	python3 params/yaml2json.py

# Generate the Gazebo model SDF from params.yaml
sdf: params
	python3 sim/gen_sdf.py

# Run the MATLAB test suite (MATLAB)
test: params
	matlab -batch "cd tests; run_all_tests('all')"

# Same suite under GNU Octave (CI / machines without MATLAB)
test-octave: params
	octave --no-gui --quiet --eval "cd tests; run_all_tests('all')"

# Python-side tests (SDF generator parse-back)
test-sim: params
	python3 -m pytest -q sim/

# Install the pre-commit hook that regenerates params.json
hooks:
	printf '#!/bin/sh\npython3 params/yaml2json.py && git add params/params.json\n' > .git/hooks/pre-commit
	chmod +x .git/hooks/pre-commit
	@echo "pre-commit hook installed"

clean:
	rm -f params/params.json sim/generated/*.sdf results/*.mat
