
BOARD_OPENSOURCE_DIR ?= vendor/qcom/opensource
.PHONY:vendor_prop_context_restriction_enforcement

vendor_prop_context_restriction_enforcement:
	python $(BOARD_OPENSOURCE_DIR)/core-utils/build/vendor_prop_context_restriction.py --m error

ifneq ($(TARGET_BOARD_PLATFORM), qssi)
droidcore:vendor_prop_context_restriction_enforcement
droidcore-unbundled:vendor_prop_context_restriction_enforcement
endif
