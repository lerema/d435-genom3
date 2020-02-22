#!/bin/sh
autoreconf -vi
#!/bin/sh
autoreconf -vi
if [ -z ${GENOM_DEVEL+x} ]
then echo "GENOM_DEVEL is unset, not copying the instrinsic calibration."
else echo "copying calibration to \$GENOM_DEVEL/etc/d435.json"
mkdir -p ${GENOM_DEVEL}/etc/
cp ./config/settings_intel.json ${GENOM_DEVEL}/etc/d435.json
fi
