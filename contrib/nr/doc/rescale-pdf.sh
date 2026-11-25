#!/usr/bin/env bash
# Copyright (c) 2011 ns-3 project
#
# SPDX-License-Identifier: GPL-2.0-only
#
# Rescale a PDF figure to a given width (e.g. "10cm"), keeping the paper size
# fitted to the figure. Usage: rescale-pdf.sh <width> <file.pdf>
# Imported from the ns-3 utils/rescale-pdf.sh so that the nr documentation
# also builds from a standalone checkout (outside an ns-3 source tree).

TMPDIR=${TMPDIR:-/tmp}

TMPFILE=`mktemp -t $(basename ${2}).XXXXXX`

ME=$(basename $0)
echo "$ME $(basename ${2}) to ${1}"

echo "
\documentclass{book}
  \usepackage{pdfpages}
  \begin{document}
    \includepdf[width=${1},fitpaper]{${2}}
  \end{document}
" \
>${TMPFILE}.tex

pdflatex -output-directory ${TMPDIR} ${TMPFILE}.tex >/dev/null 2>/dev/null
cp ${TMPFILE}.pdf ${2}
rm -f ${TMPFILE}{,.{tex,aux,log,pdf}}
