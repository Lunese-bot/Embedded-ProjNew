# BARR-C:2018 checks of app_program/ (see docs/BARR_C.md).
#   make -C tools/barrc              both checks below
#   make -C tools/barrc check        fail on any finding not in allowlist.tsv
#   make -C tools/barrc format-check fail if a file differs from
#                                    app_program/.clang-format (clang-format 18)
#   make -C tools/barrc report       print every finding, allowed or not
PYTHON       ?= python3
CLANG_FORMAT ?= clang-format
APP          := ../../app_program
SRCS          = $(shell find $(APP) -name '*.[ch]')

all: check format-check

check:
	$(PYTHON) barrc_check.py $(APP) --allow allowlist.tsv

format-check:
	@$(CLANG_FORMAT) --style=file:$(APP)/.clang-format --dry-run --Werror $(SRCS)

report:
	$(PYTHON) barrc_check.py $(APP)

.PHONY: all check format-check report
