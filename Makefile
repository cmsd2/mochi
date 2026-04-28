# Render the example notebooks (executed) to markdown so they can be
# read straight from the GitHub repo.  Modelled on maxima-demos's
# Makefile but renders to markdown instead of HTML.
#
#   make            — execute every notebook and render to markdown
#   make render     — render only (skip the execute step; assumes
#                     the .macnb files already have outputs baked in)
#   make execute    — execute only (writes results back into the .macnb)
#   make clean      — remove the rendered docs
#
# Override AXIMAR_MCP / FORMAT / OUTPUT_DIR on the command line if
# needed:
#
#   make AXIMAR_MCP=/path/to/aximar-mcp
#   make FORMAT=markdown OUTPUT_DIR=docs/notebooks

AXIMAR_MCP ?= aximar-mcp
FORMAT      ?= markdown
OUTPUT_DIR  ?= docs/notebooks

NOTEBOOKS  := $(wildcard examples/notebooks/*.macnb)
# Map examples/notebooks/<stem>.macnb → docs/notebooks/<stem>.md
MD_FILES   := $(addprefix $(OUTPUT_DIR)/,$(addsuffix .md,$(basename $(notdir $(NOTEBOOKS)))))

.PHONY: all clean execute render

all: $(MD_FILES)

# Per-notebook rule: execute the notebook in place via aximar-mcp, then
# convert to markdown via jupyter nbconvert.
define nb_rule
$(OUTPUT_DIR)/$(basename $(notdir $(1))).md: $(1) | $(OUTPUT_DIR)
	$$(AXIMAR_MCP) run --allow-dangerous $$<
	uv run jupyter nbconvert --to $$(FORMAT) \
		--output-dir $$(OUTPUT_DIR) \
		--output $$(basename $$(notdir $$<) .macnb) \
		$$<
endef

$(foreach nb,$(NOTEBOOKS),$(eval $(call nb_rule,$(nb))))

$(OUTPUT_DIR):
	mkdir -p $@

# Execute every notebook in place (writes results into the .macnb files).
execute:
	@for nb in $(NOTEBOOKS); do \
		echo "  exec  $$nb"; \
		$(AXIMAR_MCP) run --allow-dangerous "$$nb"; \
	done

# Render every (already-executed) notebook to markdown.
render: | $(OUTPUT_DIR)
	@for nb in $(NOTEBOOKS); do \
		name=$$(basename "$$nb" .macnb); \
		echo "  render $$nb -> $(OUTPUT_DIR)/$$name.md"; \
		uv run jupyter nbconvert --to $(FORMAT) \
			--output-dir $(OUTPUT_DIR) --output "$$name" "$$nb"; \
	done

clean:
	rm -rf $(OUTPUT_DIR)
