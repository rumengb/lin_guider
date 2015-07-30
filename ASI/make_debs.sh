#!/bin/bash

if [ "$USER" != "root" ]; then
	echo "Please run as root!"
	exit 1
fi

PACKAGE="libasicamera"
VERSION="0.1.0723"

# arch i386
ARCH="i386"
NAME="$PACKAGE-$VERSION-$ARCH"

install -d $NAME/DEBIAN
install -m 644 control $NAME/DEBIAN
sed "s/VERSION/$VERSION/g" -i $NAME/DEBIAN/control
sed "s/ARCH/$ARCH/g" -i $NAME/DEBIAN/control
install -d $NAME/etc/udev/rules.d
install -m 644 99-asi.rules $NAME/etc/udev/rules.d
install -d $NAME/usr/lib
cp -a lib/x86/* $NAME/usr/lib
install -d $NAME/usr/include
install -m 644 include/* $NAME/usr/include

dpkg -b $NAME $NAME.deb
rm -r $NAME

# arch amd64
ARCH="amd64"
NAME="$PACKAGE-$VERSION-$ARCH"

install -d $NAME/DEBIAN
install -m 644 control $NAME/DEBIAN
sed "s/VERSION/$VERSION/g" -i $NAME/DEBIAN/control
sed "s/ARCH/$ARCH/g" -i $NAME/DEBIAN/control
install -d $NAME/etc/udev/rules.d
install -m 644 99-asi.rules $NAME/etc/udev/rules.d
install -d $NAME/usr/lib
cp -a lib/x64/* $NAME/usr/lib
install -d $NAME/usr/include
install -m 644 include/* $NAME/usr/include

dpkg -b $NAME $NAME.deb
rm -r $NAME

# arch armhf
ARCH="armhf"
NAME="$PACKAGE-$VERSION-$ARCH"

install -d $NAME/DEBIAN
install -m 644 control $NAME/DEBIAN
sed "s/VERSION/$VERSION/g" -i $NAME/DEBIAN/control
sed "s/ARCH/$ARCH/g" -i $NAME/DEBIAN/control
install -d $NAME/etc/udev/rules.d
install -m 644 99-asi.rules $NAME/etc/udev/rules.d
install -d $NAME/usr/lib
cp -a lib/armv6/* $NAME/usr/lib
install -d $NAME/usr/include
install -m 644 include/* $NAME/usr/include

dpkg -b $NAME $NAME.deb
rm -r $NAME




