current_dir="$(pwd)"

if [ "$1" = "all" ]; then
	mkdir /tmp/simdepends && cd /tmp/simdepends
	mkdir libserial_build && cd libserial_build
	git clone https://github.com/crayzeewulf/libserial.git
	cd libserial && mkdir build && cd build
	cmake -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_DOCUMENTATION=OFF ..
	make -j$(nproc)
	sudo make install
	cd /tmp
	git clone https://github.com/brechtsanders/xlsxio.git && cd xlsx*
	rm -rf build
	mkdir build && cd build && cmake -G"Unix Makefiles" ..
	make
	sudo make install
	sudo ldconfig
	cd  "$current_dir"
fi

make clean && make
make install
echo "Done, Now run from the terminal by running 'FlightSimulator --help' to get started"
