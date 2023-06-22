#!/usr/bin/env sh
set -ue
cd "$(dirname "$0")"
rm -vrf /usr/local/share/pololu-jrk-g2
mkdir -v /usr/local/share/pololu-jrk-g2
cp jrk2cmd jrk2gui LICENSE.html /usr/local/share/pololu-jrk-g2
ln -vsf /usr/local/share/pololu-jrk-g2/jrk2cmd /usr/local/bin/
ln -vsf /usr/local/share/pololu-jrk-g2/jrk2gui /usr/local/bin/
cp -v 99-pololu.rules /etc/udev/rules.d/
