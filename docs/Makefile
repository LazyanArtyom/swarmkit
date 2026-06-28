# Minimal makefile for Sphinx documentation.

SPHINXBUILD ?= sphinx-build
DOXYGEN ?= doxygen
SOURCEDIR = source
BUILDDIR = build

.PHONY: help doxygen html clean

help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" -c .

doxygen:
	@mkdir -p "$(BUILDDIR)/doxygen"
	@$(DOXYGEN) Doxyfile

html: doxygen
	@$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" -c . $(SPHINXOPTS) $(O)

clean:
	rm -rf "$(BUILDDIR)"
