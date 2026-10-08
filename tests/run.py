#!/usr/bin/env python3
"""luce-cad's gate: the Base contracts (layout, trim predicates) and the Luce
CAD regressions, native and through the C backend."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
EXE = ".exe" if os.name == "nt" else ""
# Base contract entry points; each one runs the contract modules it imports.
CONTRACTS = ["layout_contract", "trim_predicates_contract", "exact_contract", "direct_fillet_contract"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=Path(os.environ.get("LUCE_BASE", ROOT.parent / f"luce-base/build/luce-base{EXE}")))
    parser.add_argument("--luce", type=Path, default=Path(os.environ.get("LUCE", ROOT.parent / f"luce/build/luce{EXE}")))
    parser.add_argument("--opt", choices=["0", "1", "2", "3"], default="0")
    parser.add_argument("--backend", choices=["native", "c", "both"], default="both")
    args = parser.parse_args()
    backends = ["native", "c"] if args.backend == "both" else [args.backend]
    base, luce = args.base.resolve(), args.luce.resolve()
    with tempfile.TemporaryDirectory(prefix="luce-cad-tests-") as temporary:
        env = dict(os.environ, LUCE_BASE=str(base), LUCE_CACHE=str(Path(temporary) / "cache"))
        for backend in backends:
            flags = ["--native", "--opt", args.opt] if backend == "native" else ["--backend=c"] + (["--release"] if int(args.opt) >= 2 else [])
            for contract in CONTRACTS:
                print(f"TEST {contract} {backend}", flush=True)
                binary = Path(temporary) / contract
                subprocess.run([str(base), "build", str(ROOT / f"src/tests/{contract}.lucb"), *flags, "-o", str(binary)],
                               check=True, env=env, timeout=600)
                subprocess.run([str(binary)], check=True, timeout=120)
            print(f"TEST regressions {backend}", flush=True)
            binary = Path(temporary) / "regressions"
            subprocess.run([str(luce), "build", str(ROOT / "tests/main.luc"), *flags, "-o", str(binary)],
                           check=True, env=env, timeout=600)
            subprocess.run([str(binary)], check=True, timeout=300, cwd=ROOT / "tests")
    print("PASS luce-cad", flush=True)


if __name__ == "__main__":
    main()
