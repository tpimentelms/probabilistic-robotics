SUBDIRS = shared master controller

CLEANDIRS = $(SUBDIRS:%=clean-%)

.PHONY: subdirs $(SUBDIRS)
subdirs: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

.PHONY: print
print:
	@echo Subdirs: $(SUBDIRS)
	@echo Cleandirs: $(CLEANDIRS)

.PHONY: subdirs $(CLEANDIRS)
clean: $(CLEANDIRS)
$(CLEANDIRS): 
	$(MAKE) -C $(@:clean-%=%) clean
