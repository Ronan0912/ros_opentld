all: installed

INSTALL_DIR = opentld 

SVN_DIR = build/git-opentld/
SVN_URL = https://github.com/gnebehay/OpenTLD/trunk
SVN_REVISION = -r HEAD
SVN_PATCH = patches/diff.patch patches/diff_2.patch
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) 
	cd build; cmake git-opentld; make -j

	touch installed

clean:
	rm -rf $(INSTALL_DIR) installed
	rm -rf build

.PHONY : clean download
