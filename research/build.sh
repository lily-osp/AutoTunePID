#!/usr/bin/env bash
# build.sh -- build helper for the AutoTunePID comparative evaluation paper.
#
# Usage:
#   ./build.sh          compile paper.pdf (pdflatex + bibtex, runs from any cwd)
#   ./build.sh --sim    recompile the benchmark harness, regenerate all results
#                       and figures first, then compile the PDF
#   ./build.sh --clean  remove LaTeX build artifacts and generated data
set -euo pipefail

cd "$(dirname "$0")"

# Library source: ../src when this project lives inside the AutoTunePID
# repository (research/), otherwise the standalone sketchbook path.
DEFAULT_LIB="$(cd .. && pwd)/src"
[[ -f "$DEFAULT_LIB/AutoTunePID.h" ]] || \
    DEFAULT_LIB="/home/azzar/project/microcontrollers/Arduino/libraries/AutoTunePID/src"
LIB_SRC="${ATP_LIB_SRC:-$DEFAULT_LIB}"
MAIN=paper

clean() {
    rm -f "$MAIN".{aux,log,out,bbl,blg,toc,fls,fdb_latexmk,synctex.gz} "$MAIN".pdf
    rm -rf sim/harness
    echo "[build] cleaned"
}

run_sim() {
    if [[ ! -d "$LIB_SRC" ]]; then
        echo "[build] ERROR: library source not found at $LIB_SRC (override with ATP_LIB_SRC)" >&2
        exit 1
    fi
    echo "[build] compiling harness (g++ -O2, C++14) against unmodified library source..."
    g++ -O2 -std=c++14 -Wall -I sim -I "$LIB_SRC" \
        sim/harness.cpp "$LIB_SRC/AutoTunePID.cpp" -o sim/harness
    mkdir -p results figures
    echo "[build] running benchmark matrix..."
    ./sim/harness
    echo "[build] results/ and figures/ regenerated"
}

case "${1:-}" in
    --clean) clean; exit 0 ;;
    --sim)   run_sim ;;
    "")      : ;;
    *)       echo "usage: $0 [--sim|--clean]" >&2; exit 2 ;;
esac

for f in results/table_est.tex results/table_main.tex results/table_lambda.tex results/table_phaseb.tex; do
    if [[ ! -f "$f" ]]; then
        echo "[build] missing $f; run './build.sh --sim' first" >&2
        exit 1
    fi
done

echo "[build] pdflatex pass 1..."
pdflatex -interaction=nonstopmode -halt-on-error "$MAIN.tex" >/dev/null
echo "[build] bibtex..."
bibtex "$MAIN" >/dev/null
echo "[build] pdflatex pass 2/3..."
pdflatex -interaction=nonstopmode -halt-on-error "$MAIN.tex" >/dev/null
pdflatex -interaction=nonstopmode -halt-on-error "$MAIN.tex" >/dev/null

pages=$(pdfinfo "$MAIN.pdf" | awk '/^Pages:/ {print $2}')
echo "[build] done: $MAIN.pdf ($pages pages)"
