#
# Copyright (c) 2019 LAAS/CNRS
# All rights reserved.
#
# Redistribution  and  use  in  source  and binary  forms,  with  or  without
# modification, are permitted provided that the following conditions are met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
# THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
# WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
# MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
# ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
# WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
# IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#                                              Martin Jacquet - November 2019
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
