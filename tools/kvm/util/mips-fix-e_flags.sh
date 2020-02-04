#/bin/sh

_blob=guest/guest_init.o

if od -t x1 guest/init | head -1 | grep -q '7f 45 4c 46 02'
then
    # ELF Class 64
    _offset=48
    _elfclass=64
else
    # ELF Class 32
    _offset=36
    _elfclass=32
fi
echo "Setting ELF$_elfclass header flags in binary $_blob to what's used in guest/init"

dd if=guest/init bs=1 skip=$_offset count=4 | dd of=$_blob conv=nocreat,notrunc bs=1 seek=$_offset count=4

