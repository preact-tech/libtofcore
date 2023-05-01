#!/bin/sh
#
# Copyright 2023 PreAct Technologies
#
# Use tof-rw to read metadata and data for the various storage areas and verify
# that the CRCs match
#
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
BINDIR="${DIR}/../../../build/test/functional-tests/tof-rw/"

checkStorage ()
{
	id=$1
	cmdargs=$2
	# Get the expected CRC from the metatdata
	metadata=`${BINDIR}/tof-rw $cmdargs -C`
	crcExpected=`echo $metadata |  gawk '{ print $5 }'`
	# Read the actual data, calculate its CRC and compare
	`${BINDIR}/tof-rw $cmdargs > "$id.bin"`
	crc=$(crc32 "$id.bin")
	if [ "0x$crc" != "$crcExpected" ]; then
		echo "! BAD !: $id CRC = 0x$crc, not $crcExpected"
	else
		echo "CORRECT: $id CRC32 = 0x$crc"
	fi
}

checkStorage "cal-extracted" "-c"
checkStorage "cal-raw" "-c -r"
checkStorage "lens-extracted" "-l"
checkStorage "lens-raw" "-l -r"
checkStorage "log-extracted" "-L"
checkStorage "log-raw" "-L -r"
checkStorage "manufacturing-extracted" "-m"
checkStorage "manufacturing-raw" "-m -r"
checkStorage "settings-extracted" "-S"
checkStorage "settings-raw" "-S -r"
