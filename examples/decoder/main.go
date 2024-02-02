/* Googg
An example of using an OGG decoder

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

	OGG "libs.com/googg"
)

func check(e error) {
	if e != nil {
		panic(e)
	}
}

const READ_CHUNK = 1024
const BUF_SIZE = 4096

func main() {
	/* open encoded file */
	f, err := os.Open("ogg.h.ogg")
	check(err)
	defer f.Close()

	/* open file to save decoded data */
	sf, err := os.Create("ogg.h")
	check(err)
	defer sf.Close()

	/* initialize OGG decoder */
	dec, err := OGG.NewDecoder(BUF_SIZE, f)
	defer dec.Close()

	/* read headers */
	dec.SetOnReadHeader(func(id int, data []byte) (bool, error) {
		switch id {
		case 0:
			{
				buf := bytes.NewBuffer(data)
				var res int32
				check(binary.Read(buf, binary.LittleEndian, &res))
				fmt.Printf("header 0 - %d\n", res)
				return true, nil
			}
		case 1:
			{
				fmt.Printf("header 1 - %s\n", (string)(data))
				return false, nil
			}
		}
		return false, fmt.Errorf("Wrong header id:%d", id)
	})
	check(dec.ReadHeaders(2))

	/* read and decode the rest data body */
	buf := make([]byte, 0, READ_CHUNK)
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
