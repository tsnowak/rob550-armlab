OUTDIRS=/tmp/texouts
FILENAME=$(shell ls -rt1 *.tex | xargs basename -s .tex)

PDFLATEX=pdflatex
BIBTEX=bibtex

plnx:	${FILENAME}.tex
		mkdir -p ${OUTDIRS}
		${PDFLATEX} -halt-on-error -output-directory ${OUTDIRS} ${FILENAME}.tex
		cp ${OUTDIRS}/${FILENAME}.pdf .

clearoutdir: 
		rm -rf ${OUTDIRS}
