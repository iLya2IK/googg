/* Googg
An example of using an OGG decoder

Copyright (c) 2024 by Ilya Medvedkov

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

package main

import (
	"io"
	"os"

	OGG "libs.com/googg"
)

func check(e error) {
	if e != nil {
		panic(e)
	}
}

const READ_CHUNK = 1024
const BUF_SIZE = 4096
const OGG_CHUNK = 256

func main() {
	// open file main.go to encode it in ogg format
	f, err := os.Open("main.go")
	check(err)

	// create new file main.go.ogg to save encoded data
	sf, err := os.Create("main.go.ogg")
	check(err)

	// generate ogg encoder (writing encoded data in fragments of data no larger than OGG_CHUNK)
	enc, err := OGG.NewEncoder(OGG_CHUNK, sf)
	check(err)

	// write header to the output file
	enc.SetWriteHeaderFunc(func(id int) ([]byte, error) {
		seq := [...]byte{0xff, 0x00, 0xff, 0xaa}
		return seq[0:], nil
	})
	check(enc.WriteHeader(0))

	// read data from file, encode and save to the output file
	buf := make([]byte, 0, READ_CHUNK)
	for true {
		n, err := f.Read(buf[0:READ_CHUNK])
		if err == io.EOF {
			break
		}
		check(err)

		_, err = enc.Write(buf[0:n])
		check(err)
	}
	enc.Close()
	sf.Close()
	f.Close()

	// open file main.go.ogg to read encoded data from
	f, err = os.Open("main.go.ogg")
	check(err)
	defer f.Close()

	// create new file main.out to save decoded data to
	sf, err = os.Create("main.out")
	check(err)
	defer sf.Close()

	// generate ogg decoder
	dec, err := OGG.NewDecoder(BUF_SIZE, f)
	check(err)
	defer dec.Close()

	// read header from the encoded file
	check(dec.ReadHeaders(1))

	// read data from file, decode and save to the output file
	for true {
		n, err := dec.Read(buf[0:READ_CHUNK])
		if err == io.EOF {
			break
		}
		check(err)

		_, err = sf.Write(buf[0:n])
		check(err)
	}
}
