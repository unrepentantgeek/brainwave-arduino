VERSION		= 1.0.0

DISTFILES	= Brainwave.inf license.txt
distdir		= brainwave-avr-$(VERSION)
BZ2FILE		= brainwave-avr-$(VERSION).tar.bz2

dist:
	rm -rf $(distdir)
	mkdir $(distdir)
	cp -R brainwave $(distdir)/brainwave
	$(foreach var,$(DISTFILES),cp $(var) $(distdir)/brainwave/$(var);)
	mkdir $(distdir)/brainwave/cores
	cd $(distdir) && git clone https://github.com/PaulStoffregen/cores.git && cp -R cores/teensy cores/usb_* brainwave/cores/
	cp /usr/share/arduino/hardware/arduino/avr/platform.txt $(distdir)/brainwave/
	-chmod -R a+r $(distdir)
	cd $(distdir) && tar jcvf ../$(BZ2FILE) brainwave
	-rm -rf $(distdir)
	sed -e "s/@VERSION@/$(VERSION)/; s/@ARCHIVE@/$(BZ2FILE)/; s/@SHA256SUM@/$(shell sha256sum $(BZ2FILE) | cut -f1 -d' ')/; s/@SIZE@/$(shell stat -c %s $(BZ2FILE))/" < package_brainwave-avr_index.json.in > package_brainwave-avr_index.json
