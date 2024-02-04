/* Googg
An example of using an OGG encoder

Copyright (c) 2024 by Ilya Medvedkov

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"os"

	OGG "github.com/ilya2ik/googg"
)

func check(e error) {
	if e != nil {
		panic(e)
	}
}

const READ_CHUNK = 1024
const OGG_CHUNK = 256

func main() {
	/* open the input file */
	f, err := os.Open("ogg.h")
	check(err)
	defer f.Close()

	/* create the output file */
	sf, err := os.Create("ogg.h.ogg")
	check(err)
	defer sf.Close()

	/* initialize the new encoder */
	enc, err := OGG.NewEncoder(OGG_CHUNK, sf)
	check(err)
	defer enc.Close()

	filesize := int32(0)

	/* write file headers */
	enc.SetWriteHeaderFunc(func(id int) ([]byte, error) {
		switch id {
		case 0:
			{
				buf := bytes.NewBuffer(make([]byte, 0, 4))
				err := binary.Write(buf, binary.LittleEndian, filesize)
				return buf.Bytes(), err
			}
		case 1:
			{
				return []byte("Sample OGG header\000"), nil
			}
		}
		return nil, fmt.Errorf("Wrong header id: %d", id)
	})
	/* write the first header */
	check(enc.WriteHeader(0))
	/* write the second header */
	check(enc.WriteHeader(1))

	/* encode the whole file */
	buf := make([]byte, 0, READ_CHUNK)
	for true {
		n, err := f.Read(buf[0:READ_CHUNK])
		if err == io.EOF {
			break
		}
		check(err)

		filesize += int32(n)
		_, err = enc.Write(buf[0:n])
		check(err)
	}
	/* this is usually the end of all encoding operations (+ enc.Close()) */
	/* but we need to rewrite the first header */
	/* ...manually write EOS packet */
	check(enc.WriteEOS())
	/* ...write the encoded but unwritten data to the output file */
	check(enc.Flush())
	/* ...resetting the encoder state */
	check(enc.Reset())
	/* ...go to the start of the output file */
	sf.Seek(0, io.SeekStart)
	/* ...write the first updated header again */
	check(enc.WriteHeader(0))
	/* ...write the encoded but unwritten data to the output file */
	check(enc.Flush())
	/* ...clear the encoder state. */
	/*    this prevents the EOS packet from being written on enc.Close() */
	enc.Done()
}
