##############################################################################
#
#    file                 : Makefile
#    created              : Tue Dec 18 23:40:54 CET 2007
#    copyright            : (C) 2002 Daniele Loiacono
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

dbc_funcs.h dbc_funcs.c: dbc_funcs.dbc
	dbcc -o . dbc_funcs.dbc

ROBOT       = can_server
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp sensors.cpp ObstacleSensors.cpp dbc_funcs.c CanServerBot.cpp InputManager.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml 
SHIPSUBDIRS = 0 1 2 3 4 5 6 7 8 9

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-can_server_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-can_server_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
