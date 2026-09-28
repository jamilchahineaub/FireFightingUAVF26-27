#!/usr/bin/env python3
"""Convert params/params.yaml -> params/params.json and record its SHA-256.

MATLAB/Octave read the JSON with the built-in jsondecode(); there is no native
YAML parser in MATLAB. Run `make params` (or this script) after every edit of
params.yaml. A pre-commit hook does it automatically (see Makefile: hooks).
"""
import hashlib
import json
import pathlib
import sys

import yaml

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE / "params.yaml"
DST = HERE / "params.json"


def main() -> int:
    raw = SRC.read_bytes()
    sha = hashlib.sha256(raw).hexdigest()
    data = yaml.safe_load(raw)
    data.setdefault("meta", {})["sha256"] = sha
    data["meta"]["source_file"] = SRC.name
    DST.write_text(json.dumps(data, indent=1))
    print(f"{DST.name} written  sha256={sha[:12]}...  schema_version={data['meta'].get('schema_version')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
