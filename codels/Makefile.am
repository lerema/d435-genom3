#
# Copyright (c) 1996-2015 CNRS/LAAS
#
# Permission to use, copy, modify, and distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
lib_LTLIBRARIES = libd435_codels.la

libd435_codels_la_SOURCES=	d435_c_types.h
libd435_codels_la_SOURCES+=	d435_codels.cc
libd435_codels_la_SOURCES+=	d435_hw_comm_codels.cc
libd435_codels_la_SOURCES+=	d435_rgb_publish_codels.cc
libd435_codels_la_SOURCES+=	d435_depth_publish_codels.cc

libd435_codels_la_CPPFLAGS=	$(requires_CFLAGS)
libd435_codels_la_LDFLAGS=	$(requires_LIBS)
libd435_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libd435_codels_la_LDFLAGS+=	$(codels_requires_LIBS)
libd435_codels_la_LDFLAGS+=	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	d435_c_types.h
CLEANFILES=	${BUILT_SOURCES}
d435_c_types.h: ${top_srcdir}/d435.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/d435_c_types.h.d
