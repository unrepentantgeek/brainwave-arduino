VERSION		= 1.0.0

DISTFILES	= Brainwave.inf license.txt
distdir		= brainwave-avr-$(VERSION)
TXZFILE		= brainwave-avr-$(VERSION).tar.xz

dist:
	rm -rf $(distdir)
	mkdir $(distdir)
	cp -R brainwave $(distdir)/brainwave
	$(foreach var,$(DISTFILES),cp $(var) $(distdir)/brainwave/$(var);)
	mkdir $(distdir)/brainwave/cores
	cd $(distdir) && git clone https://github.com/PaulStoffregen/cores.git && cp -R cores/teensy brainwave/cores/
	-chmod -R a+r $(distdir)
	cd $(distdir) && tar Jcvf ../$(TXZFILE) brainwave
	-rm -rf $(distdir)
	sed -e "s/@VERSION@/$(VERSION)/; s/@ARCHIVE@/$(TXZFILE)/; s/@SHA256SUM@/$(shell sha256sum $(TXZFILE) | cut -f1 -d' ')/; s/@SIZE@/$(shell stat -c %s $(TXZFILE))/" < package_brainwave-avr_index.json.in > package_brainwave-avr_index.json
