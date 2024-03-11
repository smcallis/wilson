#!/bin/sh

# This script can build a copy of OpenSSL using Emscripten.
EMSCRIPTEN=/usr/share/emscripten
VERSION=1.1.1w

wget https://www.openssl.org/source/openssl-${VERSION}.tar.gz
tar xf openssl-${VERSION}.tar.gz
cd openssl-${VERSION}

emconfigure ./Configure linux-generic64 --prefix=$EMSCRIPTEN/system

sed -i 's|^CROSS_COMPILE.*$|CROSS_COMPILE=|g' Makefile

emmake make -j 12 build_generated libssl.a libcrypto.a
rm -rf $EMSCRIPTEN/system/include/openssl
cp -R include/openssl $EMSCRIPTEN/system/include
cp libcrypto.a libssl.a $EMSCRIPTEN/system/lib
cd ..
rm -rf openssl-${VERSION}*
