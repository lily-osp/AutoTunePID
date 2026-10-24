.PHONY: all sim clean

all: paper.pdf

paper.pdf: paper.tex references.bib results/table_est.tex results/table_main.tex results/table_lambda.tex results/table_phaseb.tex figures/step_P1_ZN.csv
	./build.sh

sim:
	./build.sh --sim

clean:
	./build.sh --clean
