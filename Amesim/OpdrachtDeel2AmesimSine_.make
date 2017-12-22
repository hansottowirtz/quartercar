#
# LMS Amesim system make file
#

# This variable can be used in -L statements
# or otherwise to separate machine dependent code
# The legal values are : sun, ibm, hp, sgi, lnx, win32

# This makefile has been created using the following cathegory path list
#	$AME
#	$AME/libsig
#	$AME/libmec
#	$AME/libhydr
#	$AME/libhcd
#	$AME/libhr
#	$AME/libpn
#	$AME/libpcd
#	$AME/libth
#	$AME/libthh
#	$AME/libthcd
#	$AME/libtr
#	$AME/libfi
#	$AME/libcs
#	$AME/libem
#	$AME/libtpf
#	$AME/libac
#	$AME/libemd
#	$AME/libdrv
#	$AME/libeng
#	$AME/libexh
#	$AME/libplm
#	$AME/libheat
#	$AME/libdv
#	$AME/libicar
#	$AME/libma
#	$AME/libgm
#	$AME/libcosim
#	$AME/libesc
#	$AME/libesg
#	$AME/libec
#	$AME/libae
#	$AME/libcf
#	$AME/libcfd1d
#	$AME/libaero
#	$AME/libace
#	$AME/libacf
#	$AME/libess
#	$AME/libeb
#	$AME/libm6dof
#	$AME/liblp
#	$AME/libmotion
# End category path list

MACHINETYPE = win32-gcc

# Then the object files
OBJECTS = \
	c:/program\ files\ (x86)/lms/lms\ imagine.lab/v1520/amesim/libmec/submodels/win32-gcc/SD0000A.o \
	c:/program\ files\ (x86)/lms/lms\ imagine.lab/v1520/amesim/libmec/submodels/win32-gcc/MAS002.o \
	c:/program\ files\ (x86)/lms/lms\ imagine.lab/v1520/amesim/libmec/submodels/win32-gcc/XVLC01.o \
	c:/program\ files\ (x86)/lms/lms\ imagine.lab/v1520/amesim/libmec/submodels/win32-gcc/MAS001.o \
	c:/program\ files\ (x86)/lms/lms\ imagine.lab/v1520/amesim/libsig/submodels/win32-gcc/SIN0.o

OBJECTS2 = \
	"c:/program files (x86)/lms/lms imagine.lab/v1520/amesim/libmec/submodels/win32-gcc/SD0000A.o" \
	"c:/program files (x86)/lms/lms imagine.lab/v1520/amesim/libmec/submodels/win32-gcc/MAS002.o" \
	"c:/program files (x86)/lms/lms imagine.lab/v1520/amesim/libmec/submodels/win32-gcc/XVLC01.o" \
	"c:/program files (x86)/lms/lms imagine.lab/v1520/amesim/libmec/submodels/win32-gcc/MAS001.o" \
	"c:/program files (x86)/lms/lms imagine.lab/v1520/amesim/libsig/submodels/win32-gcc/SIN0.o"

OpdrachtDeel2AmesimSine_.dll: $(OBJECTS) OpdrachtDeel2AmesimSine_.o
	@echo OpdrachtDeel2AmesimSine_.make.link_args =
	@type OpdrachtDeel2AmesimSine_.make.link_args
	"$(AME)\interfaces\standalonesimulator\win32\stdsim_dynlink"  $(CC) $(LDFLAGS) -o OpdrachtDeel2AmesimSine_.dll OpdrachtDeel2AmesimSine_.o @"OpdrachtDeel2AmesimSine_.make.link_args" $(AMELIBS)

OpdrachtDeel2AmesimSine_.o: OpdrachtDeel2AmesimSine_.c
	"$(AME)\interfaces\user_cosim\win32\ame_user_cosim_dyncompile" $(CC) -c -DAMEUSERCOSIM -DSTANDALONESIMULATOR -I"$(AME)\interfaces\user_cosim" -I"$(AME)\interfaces\standalonesimulator" -I"$(AME)\interfaces" $(CFLAGS) -o OpdrachtDeel2AmesimSine_.o OpdrachtDeel2AmesimSine_.c

.c.o:
	@echo
	@echo "Warning: \"$<\" is newer than the object."
	@echo ""

.f.o:
	@echo
	@echo "Warning: \"$<\" is newer than the object."
	@echo ""

# End of file

