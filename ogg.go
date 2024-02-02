/* Googg
Wrapper for OGG library

Copyright (c) 2024 by Ilya Medvedkov

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

package googg

/*
#cgo CFLAGS: -I/usr/include
#cgo LDFLAGS: -logg
#include "ogg/ogg.h"
#include <stdlib.h>
#include <string.h>

int size_of_struct_ogg_iovec_t() {
    return sizeof(ogg_iovec_t);
}
int size_of_struct_ogg_stream_state() {
    return sizeof(ogg_stream_state);
}
int size_of_struct_ogg_sync_state() {
    return sizeof(ogg_sync_state);
}
int size_of_struct_ogg_page() {
    return sizeof(ogg_page);
}
int size_of_struct_ogg_packet() {
    return sizeof(ogg_packet);
}
int size_of_struct_oggpack_buffer() {
	return sizeof(oggpack_buffer);
}
*/
import "C"
import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"math/rand"
	"reflect"
	"runtime"
	"time"
	"unsafe"
)

func Ptr(data interface{}) unsafe.Pointer {
	if data == nil {
		return unsafe.Pointer(nil)
	}
	var addr unsafe.Pointer
	v := reflect.ValueOf(data)
	switch v.Type().Kind() {
	case reflect.Ptr:
		e := v.Elem()
		switch e.Kind() {
		case
			reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64,
			reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64,
			reflect.Float32, reflect.Float64:
			addr = unsafe.Pointer(e.UnsafeAddr())
		default:
			panic(fmt.Errorf("unsupported pointer to type %s; must be a slice or pointer to a singular scalar value or the first element of an array or slice", e.Kind()))
		}
	case reflect.Uintptr:
		addr = unsafe.Pointer(data.(uintptr))
	case reflect.Slice:
		addr = unsafe.Pointer(v.Index(0).UnsafeAddr())
	default:
		panic(fmt.Errorf("unsupported type %s; must be a slice or pointer to a singular scalar value or the first element of an array or slice", v.Type()))
	}
	return addr
}

/* Errors */

type errOGGException struct{}

func (v errOGGException) Error() string {
	return "libOGG. Internal error"
}

var EOGGException = errOGGException{}

type errOGGOutOfMemory struct{ err error }

func (v errOGGOutOfMemory) Error() string {
	if v.err != nil {
		return fmt.Sprintf("Fatal. Out of memory. %s", v.err.Error())
	} else {
		return "Fatal. Out of memory"
	}
}

var EOGGOutOfMemory = errOGGOutOfMemory{nil}

/* Interfaces */

type IOGGIOVec interface {
	Ref() *C.ogg_iovec_t

	AddBuffer(iov_base unsafe.Pointer, iov_len uint64)
	GetBufferAt(pos int) *C.ogg_iovec_t
	Count() int
}

type IOGGPage interface {
	Ref() *C.ogg_page

	ChecksumSet()
	Version() int
	Continued() bool
	BoS() bool
	EoS() bool
	GranulePos() int64
	SerialNo() int32
	PageNo() int64
	Packets() int
}

type IOGGPacket interface {
	Ref() *C.ogg_packet

	GetBOS() bool
	GetData() unsafe.Pointer
	GetEOS() bool
	GetGranulePos() int64
	GetPacketNum() int64
	GetSize() int64

	SetBOS(AValue bool)
	SetData(AValue unsafe.Pointer)
	SetEOS(AValue bool)
	SetGranulePos(AValue int64)
	SetPacketNum(AValue int64)
	SetSize(AValue int64)

	Clear()
}

type IOGGStreamState interface {
	Ref() *C.ogg_stream_state

	Init(serialno int32) error
	Done()

	Clear()
	Reset() error
	ResetSerialNo(serialno int32) int
	Check() bool
	EoS() bool

	PacketIn(op IOGGPacket) error
	IOVecIn(iov IOGGIOVec, e_o_s bool, granulepos int64) error
	PageOutNew() (IOGGPage, error)
	PageFlushNew() (IOGGPage, error)
	PageOut(og IOGGPage) bool
	PageOutFill(og IOGGPage, nfill int) bool
	PageOutToStream(aStr io.Writer) error
	PageFlushToStream(aStr io.Writer) error
	PagesOutToStream(aStr io.Writer) error
	PagesFlushToStream(aStr io.Writer) error
	SavePacketToStream(aStr io.Writer, op IOGGPacket) error
	Flush(og IOGGPage) bool
	FlushFill(og IOGGPage, nfill int) bool

	PageIn(og IOGGPage) error
	PageInIgnoreErrors(og IOGGPage) int
	PacketOut(op IOGGPacket) (bool, error)
	PacketOutIgnoreErrors(op IOGGPacket) int
	PacketPeek(op IOGGPacket) (bool, error)
}

type IOGGSyncState interface {
	Ref() *C.ogg_sync_state

	Init() int
	Done() int

	Clear() int
	Reset() int
	Check() int
	Buffer(size int64) unsafe.Pointer
	Wrote(bytes int64) int
	PageSeek(og IOGGPage) int64
	PageOut(og IOGGPage) int
}

type IOGGPackBuffer interface {
	Ref() *C.oggpack_buffer

	SetEndianMode(e binary.ByteOrder)
	GetEndianMode() binary.ByteOrder

	WriteInit()
	WriteCheck() int
	WriteTrunc(bits int64)
	WriteAlign()
	WriteCopy(source unsafe.Pointer, bits int64)
	Reset()
	WriteClear()
	ReadInit(buf *byte, bytes int)
	Write(value uint32, bits int)
	Look(bits int) int64
	Look1bit() int64
	Adv(bits int)
	Adv1bit()
	Read(bits int) int64
	Read1() int64
	Bytes() int64
	Bits() int64
	GetBuffer() *byte
}

/* Implementation */

type OGGRefPackBuffer struct {
	fPRef *C.oggpack_buffer
	fEnd  binary.ByteOrder
}

func (v *OGGRefPackBuffer) Ref() *C.oggpack_buffer {
	return v.fPRef
}

func NewOGGRefPackBuffer(aRef *C.oggpack_buffer, aEndian binary.ByteOrder) *OGGRefPackBuffer {
	res := OGGRefPackBuffer{
		fPRef: aRef,
		fEnd:  aEndian,
	}

	return &res
}

func (v *OGGRefPackBuffer) SetEndianMode(e binary.ByteOrder) {
	v.fEnd = e
}

func (v *OGGRefPackBuffer) GetEndianMode() binary.ByteOrder {
	return v.fEnd
}

func (v *OGGRefPackBuffer) WriteInit() {
	switch v.fEnd {
	case binary.LittleEndian:
		C.oggpack_writeinit(v.Ref())
	case binary.BigEndian:
		C.oggpackB_writeinit(v.Ref())
	}
}

func (v *OGGRefPackBuffer) WriteCheck() int {
	switch v.fEnd {
	case binary.LittleEndian:
		return int(C.oggpack_writecheck(v.Ref()))
	case binary.BigEndian:
		return int(C.oggpackB_writecheck(v.Ref()))
	}
	return -1
}

func (v *OGGRefPackBuffer) WriteTrunc(bits int64) {
	switch v.fEnd {
	case binary.LittleEndian:
		C.oggpack_writetrunc(v.Ref(), C.long(bits))
	case binary.BigEndian:
		C.oggpackB_writetrunc(v.Ref(), C.long(bits))
	}
}

func (v *OGGRefPackBuffer) WriteAlign() {
	switch v.fEnd {
	case binary.LittleEndian:
		C.oggpack_writealign(v.Ref())
	case binary.BigEndian:
		C.oggpackB_writealign(v.Ref())
	}
}

func (v *OGGRefPackBuffer) WriteCopy(source unsafe.Pointer, bits int64) {
	switch v.fEnd {
	case binary.LittleEndian:
		C.oggpack_writecopy(v.Ref(), source, C.long(bits))
	case binary.BigEndian:
		C.oggpackB_writecopy(v.Ref(), source, C.long(bits))
	}
}

func (v *OGGRefPackBuffer) Reset() {
	switch v.fEnd {
	case binary.LittleEndian:
		C.oggpack_reset(v.Ref())
	case binary.BigEndian:
		C.oggpackB_reset(v.Ref())
	}
}

func (v *OGGRefPackBuffer) WriteClear() {
	switch v.fEnd {
	case binary.LittleEndian:
		C.oggpack_writeclear(v.Ref())
	case binary.BigEndian:
		C.oggpackB_writeclear(v.Ref())
	}
}

func (v *OGGRefPackBuffer) ReadInit(buf *byte, bytes int) {
	switch v.fEnd {
	case binary.LittleEndian:
		C.oggpack_readinit(v.Ref(), (*C.uchar)(buf), C.int(bytes))
	case binary.BigEndian:
		C.oggpackB_readinit(v.Ref(), (*C.uchar)(buf), C.int(bytes))
	}
}

func (v *OGGRefPackBuffer) Write(value uint32, bits int) {
	switch v.fEnd {
	case binary.LittleEndian:
		C.oggpack_write(v.Ref(), (C.ulong)(value), C.int(bits))
	case binary.BigEndian:
		C.oggpackB_write(v.Ref(), (C.ulong)(value), C.int(bits))
	}
}

func (v *OGGRefPackBuffer) Look(bits int) int64 {
	switch v.fEnd {
	case binary.LittleEndian:
		return int64(C.oggpack_look(v.Ref(), C.int(bits)))
	case binary.BigEndian:
		return int64(C.oggpackB_look(v.Ref(), C.int(bits)))
	}
	return -1
}

func (v *OGGRefPackBuffer) Look1bit() int64 {
	switch v.fEnd {
	case binary.LittleEndian:
		return int64(C.oggpack_look1(v.Ref()))
	case binary.BigEndian:
		return int64(C.oggpackB_look1(v.Ref()))
	}
	return -1
}

func (v *OGGRefPackBuffer) Adv(bits int) {
	switch v.fEnd {
	case binary.LittleEndian:
		C.oggpack_adv(v.Ref(), C.int(bits))
	case binary.BigEndian:
		C.oggpackB_adv(v.Ref(), C.int(bits))
	}
}

func (v *OGGRefPackBuffer) Adv1bit() {
	switch v.fEnd {
	case binary.LittleEndian:
		C.oggpack_adv1(v.Ref())
	case binary.BigEndian:
		C.oggpackB_adv1(v.Ref())
	}
}

func (v *OGGRefPackBuffer) Read(bits int) int64 {
	switch v.fEnd {
	case binary.LittleEndian:
		return int64(C.oggpack_read(v.Ref(), C.int(bits)))
	case binary.BigEndian:
		return int64(C.oggpackB_read(v.Ref(), C.int(bits)))
	}
	return -1
}

func (v *OGGRefPackBuffer) Read1() int64 {
	switch v.fEnd {
	case binary.LittleEndian:
		return int64(C.oggpack_read1(v.Ref()))
	case binary.BigEndian:
		return int64(C.oggpackB_read1(v.Ref()))
	}
	return -1
}

func (v *OGGRefPackBuffer) Bytes() int64 {
	switch v.fEnd {
	case binary.LittleEndian:
		return int64(C.oggpack_bytes(v.Ref()))
	case binary.BigEndian:
		return int64(C.oggpackB_bytes(v.Ref()))
	}
	return -1
}

func (v *OGGRefPackBuffer) Bits() int64 {
	switch v.fEnd {
	case binary.LittleEndian:
		return int64(C.oggpack_bits(v.Ref()))
	case binary.BigEndian:
		return int64(C.oggpackB_bits(v.Ref()))
	}
	return -1
}

func (v *OGGRefPackBuffer) GetBuffer() *byte {
	switch v.fEnd {
	case binary.LittleEndian:
		return (*byte)(C.oggpack_get_buffer(v.Ref()))
	case binary.BigEndian:
		return (*byte)(C.oggpackB_get_buffer(v.Ref()))
	}
	return nil
}

type OGGUniqPackBuffer struct {
	fRef   *OGGRefPackBuffer
	fValue *C.oggpack_buffer
}

func NewOGGUniqPackBuffer(bv binary.ByteOrder) (*OGGUniqPackBuffer, error) {
	value := new(OGGUniqPackBuffer)
	mem, err := C.calloc(1, (C.size_t)(C.size_of_struct_oggpack_buffer()))
	if mem == nil {
		return nil, errOGGOutOfMemory{err}
	}
	value.fValue = (*C.oggpack_buffer)(mem)
	runtime.SetFinalizer(value, func(a *OGGUniqPackBuffer) {
		if a.fValue != nil {
			C.free(unsafe.Pointer(a.fValue))
		}
	})
	value.fRef = NewOGGRefPackBuffer(value.fValue, bv)

	return value, nil
}

func (v *OGGUniqPackBuffer) Ref() *C.oggpack_buffer {
	return v.fRef.Ref()
}

func (v *OGGUniqPackBuffer) SetEndianMode(e binary.ByteOrder) {
	v.fRef.SetEndianMode(e)
}

func (v *OGGUniqPackBuffer) GetEndianMode() binary.ByteOrder {
	return v.fRef.GetEndianMode()
}

func (v *OGGUniqPackBuffer) WriteInit() {
	v.fRef.WriteInit()
}

func (v *OGGUniqPackBuffer) WriteCheck() int {
	return v.fRef.WriteCheck()
}

func (v *OGGUniqPackBuffer) WriteTrunc(bits int64) {
	v.fRef.WriteTrunc(bits)
}

func (v *OGGUniqPackBuffer) WriteAlign() {
	v.fRef.WriteAlign()
}

func (v *OGGUniqPackBuffer) WriteCopy(source unsafe.Pointer, bits int64) {
	v.fRef.WriteCopy(source, bits)
}

func (v *OGGUniqPackBuffer) Reset() {
	v.fRef.Reset()
}

func (v *OGGUniqPackBuffer) WriteClear() {
	v.fRef.WriteClear()
}

func (v *OGGUniqPackBuffer) ReadInit(buf *byte, bytes int) {
	v.fRef.ReadInit(buf, bytes)
}

func (v *OGGUniqPackBuffer) Write(value uint32, bits int) {
	v.fRef.Write(value, bits)
}

func (v *OGGUniqPackBuffer) Look(bits int) int64 {
	return v.fRef.Look(bits)
}

func (v *OGGUniqPackBuffer) Look1bit() int64 {
	return v.fRef.Look1bit()
}

func (v *OGGUniqPackBuffer) Adv(bits int) {
	v.fRef.Adv(bits)
}

func (v *OGGUniqPackBuffer) Adv1bit() {
	v.fRef.Adv1bit()
}

func (v *OGGUniqPackBuffer) Read(bits int) int64 {
	return v.fRef.Read(bits)
}

func (v *OGGUniqPackBuffer) Read1() int64 {
	return v.fRef.Read1()
}

func (v *OGGUniqPackBuffer) Bytes() int64 {
	return v.fRef.Bytes()
}

func (v *OGGUniqPackBuffer) Bits() int64 {
	return v.fRef.Bits()
}

func (v *OGGUniqPackBuffer) GetBuffer() *byte {
	return v.fRef.GetBuffer()
}

type OGGRefSyncState struct {
	fPRef *C.ogg_sync_state
}

func NewOGGRefSyncState(ref *C.ogg_sync_state) *OGGRefSyncState {
	res := OGGRefSyncState{fPRef: ref}
	return &res
}

func (v *OGGRefSyncState) Init() int {
	return int(C.ogg_sync_init(v.Ref()))
}

func (v *OGGRefSyncState) Done() int {
	return int(C.ogg_sync_clear(v.Ref()))
}

func (v *OGGRefSyncState) Ref() *C.ogg_sync_state {
	return v.fPRef
}

func (v *OGGRefSyncState) Clear() int {
	return int(C.ogg_sync_clear(v.Ref()))
}

func (v *OGGRefSyncState) Reset() int {
	return int(C.ogg_sync_reset(v.Ref()))
}

func (v *OGGRefSyncState) Check() int {
	return int(C.ogg_sync_check(v.Ref()))
}

func (v *OGGRefSyncState) Buffer(size int64) unsafe.Pointer {
	return unsafe.Pointer(C.ogg_sync_buffer(v.Ref(), C.long(size)))
}

func (v *OGGRefSyncState) Wrote(bytes int64) int {
	return int(C.ogg_sync_wrote(v.Ref(), C.long(bytes)))
}

func (v *OGGRefSyncState) PageSeek(og IOGGPage) int64 {
	return int64(C.ogg_sync_pageseek(v.Ref(), og.Ref()))
}

func (v *OGGRefSyncState) PageOut(og IOGGPage) int {
	return int(C.ogg_sync_pageout(v.Ref(), og.Ref()))
}

type OGGUniqSyncState struct {
	fRef   *OGGRefSyncState
	fValue *C.ogg_sync_state
}

func NewOGGUniqSyncState() (*OGGUniqSyncState, error) {
	value := new(OGGUniqSyncState)
	mem, err := C.calloc(1, (C.size_t)(C.size_of_struct_ogg_sync_state()))
	if mem == nil {
		return nil, errOGGOutOfMemory{err}
	}
	value.fValue = (*C.ogg_sync_state)(mem)
	runtime.SetFinalizer(value, func(a *OGGUniqSyncState) {
		a.Done()
	})
	value.fRef = NewOGGRefSyncState(value.fValue)

	return value, nil
}

func (v *OGGUniqSyncState) Ref() *C.ogg_sync_state {
	return v.fRef.Ref()
}

func (v *OGGUniqSyncState) Init() int {
	return v.fRef.Init()
}

func (v *OGGUniqSyncState) Done() int {
	if v.fValue != nil {
		res := v.fRef.Done()
		C.free(unsafe.Pointer(v.fValue))
		v.fValue = nil
		return res
	}
	return -1
}

func (v *OGGUniqSyncState) Clear() int {
	return v.fRef.Clear()
}

func (v *OGGUniqSyncState) Reset() int {
	return v.fRef.Reset()
}

func (v *OGGUniqSyncState) Check() int {
	return v.fRef.Check()
}

func (v *OGGUniqSyncState) Buffer(size int64) unsafe.Pointer {
	return v.fRef.Buffer(size)
}

func (v *OGGUniqSyncState) Wrote(bytes int64) int {
	return v.fRef.Wrote(bytes)
}

func (v *OGGUniqSyncState) PageSeek(og IOGGPage) int64 {
	return v.fRef.PageSeek(og)
}

func (v *OGGUniqSyncState) PageOut(og IOGGPage) int {
	return v.fRef.PageOut(og)
}

type OGGRefStreamState struct {
	fPRef *C.ogg_stream_state
}

func NewOGGRefStreamState(ref *C.ogg_stream_state) *OGGRefStreamState {
	res := new(OGGRefStreamState)
	res.fPRef = ref
	res.Init(0)
	return res
}

func (v *OGGRefStreamState) Init(serialno int32) error {
	R := C.ogg_stream_init(v.Ref(), C.int(serialno))
	if R != 0 {
		return EOGGException
	}
	return nil
}

func (v *OGGRefStreamState) Done() {
	C.ogg_stream_clear(v.Ref())
}

func (v *OGGRefStreamState) Ref() *C.ogg_stream_state {
	return v.fPRef
}

func (v *OGGRefStreamState) Clear() {
	C.ogg_stream_clear(v.Ref())
}

func (v *OGGRefStreamState) Reset() error {
	R := C.ogg_stream_reset(v.Ref())
	if R != 0 {
		return EOGGException
	}
	return nil
}

func (v *OGGRefStreamState) ResetSerialNo(serialno int32) int {
	return int(C.ogg_stream_reset_serialno(v.Ref(), C.int(serialno)))
}

func (v *OGGRefStreamState) Check() bool {
	return C.ogg_stream_check(v.Ref()) == 0
}

func (v *OGGRefStreamState) EoS() bool {
	return C.ogg_stream_eos(v.Ref()) != 0
}

func (v *OGGRefStreamState) PacketIn(op IOGGPacket) error {
	R := C.ogg_stream_packetin(v.Ref(), op.Ref())
	if R < 0 {
		return EOGGException
	}
	return nil
}

func (v *OGGRefStreamState) IOVecIn(iov IOGGIOVec, e_o_s bool, granulepos int64) error {
	var e_os_b C.long
	if e_o_s {
		e_os_b = 1
	} else {
		e_os_b = 0
	}

	R := C.ogg_stream_iovecin(v.Ref(), iov.Ref(), C.int(iov.Count()), e_os_b, C.long(granulepos))
	if R < 0 {
		return EOGGException
	}
	return nil
}

func (v *OGGRefStreamState) PageOutNew() (IOGGPage, error) {
	Result, err := NewOGGUniqPage()
	if err != nil {
		return nil, err
	}
	if !v.PageOut(Result) {
		return nil, EOGGException
	}
	return Result, nil
}

func (v *OGGRefStreamState) PageFlushNew() (IOGGPage, error) {
	Result, err := NewOGGUniqPage()
	if err != nil {
		return nil, err
	}
	if !v.Flush(Result) {
		return nil, EOGGException
	}
	return Result, nil
}

func (v *OGGRefStreamState) PageOut(og IOGGPage) bool {
	return C.ogg_stream_pageout(v.Ref(), og.Ref()) != 0
}

func (v *OGGRefStreamState) PageOutFill(og IOGGPage, nfill int) bool {
	return C.ogg_stream_pageout_fill(v.Ref(), og.Ref(), C.int(nfill)) != 0
}

func writePage(aStr io.Writer, og IOGGPage) error {
	h := unsafe.Slice((*byte)(og.Ref().header), og.Ref().header_len)
	_, err := aStr.Write(h)
	if err != nil {
		return err
	}
	b := unsafe.Slice((*byte)(og.Ref().body), og.Ref().body_len)
	_, err = aStr.Write(b)
	if err != nil {
		return err
	}
	return nil
}

func (v *OGGRefStreamState) PageOutToStream(aStr io.Writer) error {
	og, err := v.PageOutNew()
	if err != nil {
		return err
	}
	return writePage(aStr, og)
}

func (v *OGGRefStreamState) PageFlushToStream(aStr io.Writer) error {
	og, err := v.PageOutNew()
	if err != nil {
		return err
	}
	return writePage(aStr, og)
}

func (v *OGGRefStreamState) PagesOutToStream(aStr io.Writer) error {
	og, err := NewOGGUniqPage()
	if err != nil {
		return err
	}
	for v.PageOut(og) {
		err := writePage(aStr, og)
		if err != nil {
			return err
		}
	}
	return nil
}

func (v *OGGRefStreamState) PagesFlushToStream(aStr io.Writer) error {
	og, err := NewOGGUniqPage()
	if err != nil {
		return err
	}
	for v.Flush(og) {
		err := writePage(aStr, og)
		if err != nil {
			return err
		}
	}
	return nil
}

func (v *OGGRefStreamState) SavePacketToStream(aStr io.Writer, op IOGGPacket) error {
	err := v.PacketIn(op)
	if err != nil {
		return err
	}
	return v.PagesOutToStream(aStr)
}

func (v *OGGRefStreamState) Flush(og IOGGPage) bool {
	return C.ogg_stream_flush(v.Ref(), og.Ref()) != 0
}

func (v *OGGRefStreamState) FlushFill(og IOGGPage, nfill int) bool {
	return C.ogg_stream_flush_fill(v.Ref(), og.Ref(), C.int(nfill)) != 0
}

func (v *OGGRefStreamState) PageIn(og IOGGPage) error {
	R := C.ogg_stream_pagein(v.Ref(), og.Ref())
	if R < 0 {
		return EOGGException
	}
	return nil
}

func (v *OGGRefStreamState) PageInIgnoreErrors(og IOGGPage) int {
	return int(C.ogg_stream_pagein(v.Ref(), og.Ref()))
}

func (v *OGGRefStreamState) PacketOut(op IOGGPacket) (bool, error) {
	R := C.ogg_stream_packetout(v.Ref(), op.Ref())
	if R == 1 {
		return true, nil
	} else if R < 0 {
		return false, EOGGException
	} else {
		return false, nil
	}
}

func (v *OGGRefStreamState) PacketOutIgnoreErrors(op IOGGPacket) int {
	return int(C.ogg_stream_packetout(v.Ref(), op.Ref()))
}

func (v *OGGRefStreamState) PacketPeek(op IOGGPacket) (bool, error) {
	R := C.ogg_stream_packetpeek(v.Ref(), op.Ref())
	if R == 1 {
		return true, nil
	} else if R < 0 {
		return false, EOGGException
	} else {
		return true, nil
	}
}

type OGGUniqStreamState struct {
	fRef   *OGGRefStreamState
	fValue *C.ogg_stream_state
}

func NewOGGUniqStreamState(serialno int32) (*OGGUniqStreamState, error) {
	value := new(OGGUniqStreamState)

	mem, err := C.calloc(1, (C.size_t)(C.size_of_struct_ogg_stream_state()))
	if mem == nil {
		return nil, errOGGOutOfMemory{err}
	}
	value.fValue = (*C.ogg_stream_state)(mem)
	runtime.SetFinalizer(value, func(a *OGGUniqStreamState) {
		if a.fValue != nil {
			a.Done()
		}
	})

	value.fRef = NewOGGRefStreamState(value.fValue)
	res := value.fRef.ResetSerialNo(serialno)
	if res != 0 {
		return nil, EOGGException
	}

	return value, nil
}

func (v *OGGUniqStreamState) Ref() *C.ogg_stream_state {
	return v.fRef.Ref()
}

func (v *OGGUniqStreamState) Init(serialno int32) error {
	return v.fRef.Init(serialno)
}

func (v *OGGUniqStreamState) Done() {
	if v.fValue != nil {
		v.fRef.Done()
		C.free(unsafe.Pointer(v.fValue))
		v.fValue = nil
	}
}

func (v *OGGUniqStreamState) Clear() {
	v.fRef.Clear()
}

func (v *OGGUniqStreamState) Reset() error {
	return v.fRef.Reset()
}

func (v *OGGUniqStreamState) ResetSerialNo(serialno int32) int {
	return v.fRef.ResetSerialNo(serialno)
}

func (v *OGGUniqStreamState) Check() bool {
	return v.fRef.Check()
}

func (v *OGGUniqStreamState) EoS() bool {
	return v.fRef.EoS()
}

func (v *OGGUniqStreamState) PacketIn(op IOGGPacket) error {
	return v.fRef.PacketIn(op)
}

func (v *OGGUniqStreamState) IOVecIn(iov IOGGIOVec, e_o_s bool, granulepos int64) error {
	return v.fRef.IOVecIn(iov, e_o_s, granulepos)
}

func (v *OGGUniqStreamState) PageOutNew() (IOGGPage, error) {
	return v.fRef.PageOutNew()
}

func (v *OGGUniqStreamState) PageFlushNew() (IOGGPage, error) {
	return v.fRef.PageFlushNew()
}

func (v *OGGUniqStreamState) PageOut(og IOGGPage) bool {
	return v.fRef.PageOut(og)
}

func (v *OGGUniqStreamState) PageOutFill(og IOGGPage, nfill int) bool {
	return v.fRef.PageOutFill(og, nfill)
}

func (v *OGGUniqStreamState) PageOutToStream(aStr io.Writer) error {
	return v.fRef.PageOutToStream(aStr)
}

func (v *OGGUniqStreamState) PageFlushToStream(aStr io.Writer) error {
	return v.fRef.PageFlushToStream(aStr)
}

func (v *OGGUniqStreamState) PagesOutToStream(aStr io.Writer) error {
	return v.fRef.PagesOutToStream(aStr)
}

func (v *OGGUniqStreamState) PagesFlushToStream(aStr io.Writer) error {
	return v.fRef.PagesFlushToStream(aStr)
}

func (v *OGGUniqStreamState) SavePacketToStream(aStr io.Writer, op IOGGPacket) error {
	return v.fRef.SavePacketToStream(aStr, op)
}

func (v *OGGUniqStreamState) Flush(og IOGGPage) bool {
	return v.fRef.Flush(og)
}

func (v *OGGUniqStreamState) FlushFill(og IOGGPage, nfill int) bool {
	return v.fRef.FlushFill(og, nfill)
}

func (v *OGGUniqStreamState) PageIn(og IOGGPage) error {
	return v.fRef.PageIn(og)
}

func (v *OGGUniqStreamState) PageInIgnoreErrors(og IOGGPage) int {
	return v.fRef.PageInIgnoreErrors(og)
}

func (v *OGGUniqStreamState) PacketOut(op IOGGPacket) (bool, error) {
	return v.fRef.PacketOut(op)
}

func (v *OGGUniqStreamState) PacketOutIgnoreErrors(op IOGGPacket) int {
	return v.fRef.PacketOutIgnoreErrors(op)
}

func (v *OGGUniqStreamState) PacketPeek(op IOGGPacket) (bool, error) {
	return v.fRef.PacketPeek(op)
}

type OGGRefPacket struct {
	fPRef *C.ogg_packet
}

func NewOGGRefPacket(ref *C.ogg_packet) *OGGRefPacket {
	res := OGGRefPacket{fPRef: ref}
	return &res
}

func (v *OGGRefPacket) GetBOS() bool {
	return bool(v.fPRef.b_o_s > 0)
}

func (v *OGGRefPacket) GetData() unsafe.Pointer {
	return unsafe.Pointer(v.fPRef.packet)
}

func (v *OGGRefPacket) GetEOS() bool {
	return bool(v.fPRef.e_o_s > 0)
}

func (v *OGGRefPacket) GetGranulePos() int64 {
	return int64(v.fPRef.granulepos)
}

func (v *OGGRefPacket) GetPacketNum() int64 {
	return int64(v.fPRef.packetno)
}

func (v *OGGRefPacket) GetSize() int64 {
	return int64(v.fPRef.bytes)
}

func (v *OGGRefPacket) SetBOS(AValue bool) {
	if AValue {
		v.fPRef.b_o_s = C.long(1)
	} else {
		v.fPRef.b_o_s = C.long(0)
	}
}

func (v *OGGRefPacket) SetData(AValue unsafe.Pointer) {
	v.fPRef.packet = (*C.uchar)(AValue)
}

func (v *OGGRefPacket) SetEOS(AValue bool) {
	if AValue {
		v.fPRef.e_o_s = C.long(1)
	} else {
		v.fPRef.e_o_s = C.long(0)
	}
}

func (v *OGGRefPacket) SetGranulePos(AValue int64) {
	v.fPRef.granulepos = C.long(AValue)
}

func (v *OGGRefPacket) SetPacketNum(AValue int64) {
	v.fPRef.packetno = C.long(AValue)
}

func (v *OGGRefPacket) SetSize(AValue int64) {
	v.Ref().bytes = C.long(AValue)
}

func (v *OGGRefPacket) Ref() *C.ogg_packet {
	return v.fPRef
}

func (v *OGGRefPacket) Clear() {
	C.ogg_packet_clear(v.fPRef)
}

type OGGUniqPacket struct {
	fRef   *OGGRefPacket
	fValue *C.ogg_packet
}

func NewOGGUniqPacket() (*OGGUniqPacket, error) {
	value := new(OGGUniqPacket)
	mem, err := C.calloc(1, (C.size_t)(C.size_of_struct_ogg_packet()))
	if mem == nil {
		return nil, errOGGOutOfMemory{err}
	}
	value.fValue = (*C.ogg_packet)(mem)
	runtime.SetFinalizer(value, func(a *OGGUniqPacket) {
		if a.fValue != nil {
			C.free(unsafe.Pointer(a.fValue))
		}
	})
	value.fRef = NewOGGRefPacket(value.fValue)

	return value, nil
}

func (v *OGGUniqPacket) Ref() *C.ogg_packet {
	return v.fRef.Ref()
}

func (v *OGGUniqPacket) GetBOS() bool {
	return v.fRef.GetBOS()
}

func (v *OGGUniqPacket) GetData() unsafe.Pointer {
	return v.fRef.GetData()
}

func (v *OGGUniqPacket) GetEOS() bool {
	return v.fRef.GetEOS()
}

func (v *OGGUniqPacket) GetGranulePos() int64 {
	return v.fRef.GetGranulePos()
}

func (v *OGGUniqPacket) GetPacketNum() int64 {
	return v.fRef.GetPacketNum()
}

func (v *OGGUniqPacket) GetSize() int64 {
	return v.fRef.GetSize()
}

func (v *OGGUniqPacket) SetBOS(AValue bool) {
	v.fRef.SetBOS(AValue)
}

func (v *OGGUniqPacket) SetData(AValue unsafe.Pointer) {
	v.fRef.SetData(AValue)
}

func (v *OGGUniqPacket) SetEOS(AValue bool) {
	v.fRef.SetEOS(AValue)
}

func (v *OGGUniqPacket) SetGranulePos(AValue int64) {
	v.fRef.SetGranulePos(AValue)
}

func (v *OGGUniqPacket) SetPacketNum(AValue int64) {
	v.fRef.SetPacketNum(AValue)
}

func (v *OGGUniqPacket) SetSize(AValue int64) {
	v.fRef.SetSize(AValue)
}

func (v *OGGUniqPacket) Clear() {
	v.fRef.Clear()
}

type OGGRefPage struct {
	fPRef *C.ogg_page
}

func NewOGGRefPage(v *C.ogg_page) *OGGRefPage {
	res := OGGRefPage{fPRef: v}
	return &res
}

func (v *OGGRefPage) Ref() *C.ogg_page {
	return v.fPRef
}

func (v *OGGRefPage) ChecksumSet() {
	C.ogg_page_checksum_set(v.fPRef)
}

func (v *OGGRefPage) Version() int {
	return int(C.ogg_page_version(v.fPRef))
}

func (v *OGGRefPage) Continued() bool {
	return C.ogg_page_continued(v.fPRef) > 0
}

func (v *OGGRefPage) BoS() bool {
	return C.ogg_page_bos(v.fPRef) > 0
}

func (v *OGGRefPage) EoS() bool {
	return C.ogg_page_eos(v.fPRef) > 0
}

func (v *OGGRefPage) GranulePos() int64 {
	return int64(C.ogg_page_granulepos(v.fPRef))
}

func (v *OGGRefPage) SerialNo() int32 {
	return int32(C.ogg_page_serialno(v.fPRef))
}

func (v *OGGRefPage) PageNo() int64 {
	return int64(C.ogg_page_pageno(v.fPRef))
}

func (v *OGGRefPage) Packets() int {
	return int(C.ogg_page_packets(v.fPRef))
}

type OGGUniqPage struct {
	fRef   *OGGRefPage
	fValue *C.ogg_page
}

func NewOGGUniqPage() (*OGGUniqPage, error) {
	value := new(OGGUniqPage)
	mem, err := C.calloc(1, (C.size_t)(C.size_of_struct_ogg_page()))
	if mem == nil {
		return nil, errOGGOutOfMemory{err}
	}
	value.fValue = (*C.ogg_page)(mem)
	runtime.SetFinalizer(value, func(a *OGGUniqPage) {
		if a.fValue != nil {
			C.free(unsafe.Pointer(a.fValue))
		}
	})
	value.fRef = NewOGGRefPage(value.fValue)

	return value, nil
}

func (v *OGGUniqPage) Ref() *C.ogg_page {
	return v.fRef.Ref()
}

func (v *OGGUniqPage) ChecksumSet() {
	v.fRef.ChecksumSet()
}

func (v *OGGUniqPage) Version() int {
	return v.fRef.Version()
}

func (v *OGGUniqPage) Continued() bool {
	return v.fRef.Continued()
}

func (v *OGGUniqPage) BoS() bool {
	return v.fRef.BoS()
}

func (v *OGGUniqPage) EoS() bool {
	return v.fRef.EoS()
}

func (v *OGGUniqPage) GranulePos() int64 {
	return v.fRef.GranulePos()
}

func (v *OGGUniqPage) SerialNo() int32 {
	return v.fRef.SerialNo()
}

func (v *OGGUniqPage) PageNo() int64 {
	return v.fRef.PageNo()
}

func (v *OGGUniqPage) Packets() int {
	return v.fRef.Packets()
}

type OGGIOVecListed struct {
	veclist []C.ogg_iovec_t
}

func NewOGGIOVecListed() *OGGIOVecListed {
	return &OGGIOVecListed{veclist: make([]C.ogg_iovec_t, 0)}
}

func (v *OGGIOVecListed) AddBuffer(iov_base unsafe.Pointer, iov_len uint64) {
	var p = C.ogg_iovec_t{
		iov_base: iov_base,
		iov_len:  C.size_t(iov_len)}
	v.veclist = append(v.veclist, p)
}

func (v *OGGIOVecListed) GetBufferAt(pos int) *C.ogg_iovec_t {
	return &(v.veclist[pos])
}

func (v *OGGIOVecListed) Ref() *C.ogg_iovec_t {
	return (*C.ogg_iovec_t)(&v.veclist[0])
}

func (v *OGGIOVecListed) Count() int {
	return len(v.veclist)
}

type OGGIOVecStatic struct {
	fRef   *C.ogg_iovec_t
	fCount int
	fOwned bool
}

func NewOGGIOVecStatic(aRef *C.ogg_iovec_t, aCount int, aOwned bool) *OGGIOVecStatic {
	return &OGGIOVecStatic{
		fRef:   aRef,
		fCount: aCount,
		fOwned: aOwned,
	}
}

func (v *OGGIOVecStatic) Done() {
	if v.fOwned {
		C.free(unsafe.Pointer(v.fRef))
	}
}

func (v *OGGIOVecStatic) AddBuffer(iov_base unsafe.Pointer, iov_len uint64) {
	//not supported
}

func (v *OGGIOVecStatic) GetBufferAt(pos int) *C.ogg_iovec_t {
	return &(*(*[0x7fffffff]C.ogg_iovec_t)(unsafe.Pointer(v.fRef)))[pos]
}

func (v *OGGIOVecStatic) Count() int {
	return v.fCount
}

func (v *OGGIOVecStatic) Ref() *C.ogg_iovec_t {
	return v.fRef
}

type OGGIOVecStream struct {
	fRef []byte
}

func NewOGGIOVecStream(abuf []byte) *OGGIOVecStream {
	return &OGGIOVecStream{fRef: abuf}
}

func (v *OGGIOVecStream) AddBuffer(iov_base unsafe.Pointer, iov_len uint64) {
	wr := bytes.NewBuffer(v.fRef)
	wr.Write((*(*[0x7fffffff]byte)(iov_base))[0:iov_len])
	binary.Write(wr, binary.NativeEndian, iov_len)
}

func (v *OGGIOVecStream) GetBufferAt(pos int) *C.ogg_iovec_t {
	return &(*(*[0x7fffffff]C.ogg_iovec_t)(Ptr(v.fRef)))[pos]
}

func (v *OGGIOVecStream) Count() int {
	return len(v.fRef) / int(C.size_of_struct_ogg_iovec_t())
}

func (v *OGGIOVecStream) Ref() *C.ogg_iovec_t {
	return v.GetBufferAt(0)
}

/* OGG funcs */

func NewPackBuffer(aEndian binary.ByteOrder) (IOGGPackBuffer, error) {
	return NewOGGUniqPackBuffer(aEndian)
}

func RefPackBuffer(aRef *C.oggpack_buffer, aEndian binary.ByteOrder) IOGGPackBuffer {
	return NewOGGRefPackBuffer(aRef, aEndian)
}

func NewSyncState() (IOGGSyncState, error) {
	return NewOGGUniqSyncState()
}

func RefSyncState(st *C.ogg_sync_state) IOGGSyncState {
	return NewOGGRefSyncState(st)
}

func NewStream(serialno int32) (IOGGStreamState, error) {
	return NewOGGUniqStreamState(serialno)
}

func RefStream(st *C.ogg_stream_state) IOGGStreamState {
	return NewOGGRefStreamState(st)
}

func NewPacket() (IOGGPacket, error) {
	return NewOGGUniqPacket()
}

func NewPacketExt(data unsafe.Pointer, sz, num, granule int64, isEOS, isBOS bool) (IOGGPacket, error) {
	Result, err := NewPacket()
	if err != nil {
		return nil, err
	}

	Result.SetData(data)
	Result.SetSize(sz)
	Result.SetGranulePos(granule)
	Result.SetPacketNum(num)
	Result.SetBOS(isBOS)
	Result.SetEOS(isEOS)
	return Result, nil
}

func NewSizedPacket(data unsafe.Pointer, sz, num, granule int64) (IOGGPacket, error) {
	return NewPacketExt(data, sz, num, granule, false, false)
}

func NewEOSPacket(data unsafe.Pointer, sz, num, granule int64) (IOGGPacket, error) {
	return NewPacketExt(data, sz, num, granule, true, false)
}

func NewBOSPacket(data unsafe.Pointer, sz, num, granule int64) (IOGGPacket, error) {
	return NewPacketExt(data, sz, num, granule, false, true)
}

func RefPacket(st *C.ogg_packet) IOGGPacket {
	return NewOGGRefPacket(st)
}

func NewPage() (IOGGPage, error) {
	return NewOGGUniqPage()
}

func RefPage(st *C.ogg_page) IOGGPage {
	return NewOGGRefPage(st)
}

func NewIOVecListed() IOGGIOVec {
	return NewOGGIOVecListed()
}

func ewIOVecStatic(aRef *C.ogg_iovec_t, aCount int, aOwned bool) IOGGIOVec {
	return NewOGGIOVecStatic(aRef, aCount, aOwned)
}

func NewIOVecStream(aRef []byte) IOGGIOVec {
	return NewOGGIOVecStream(aRef)
}

type IEncoder interface {
	WriteHeader(id int) error
	WriteEOS() error
	Flush() error
	Reset() error
	Done()
	GranuleFromBytes(pos int64) int64
}

type ByteGranuleConvertFunc = func(pos int64) int64
type WriteHeaderFunc = func(id int) ([]byte, error)

type Encoder struct {
	fChunkSize       int
	fStream          IOGGStreamState
	fCurGranulePos   int64
	fCurCurPacketNum int64
	fWriter          io.Writer

	onGranuleFromBytes ByteGranuleConvertFunc
	onWriteHeader      WriteHeaderFunc
}

func NewEncoder(aChunkSize int, aWriter io.Writer) (*Encoder, error) {
	str, err := NewStream(int32(rand.Int63n(time.Now().UnixMilli())))

	if err != nil {
		return nil, err
	}

	return &(Encoder{
		fChunkSize: aChunkSize,
		fStream:    str,
		fWriter:    aWriter,
		onWriteHeader: func(id int) ([]byte, error) {
			return make([]byte, 0), nil
		},
	}), nil
}

func (v *Encoder) SetGranuleFromBytesConvertFunc(f ByteGranuleConvertFunc) {
	v.onGranuleFromBytes = f
}

func (v *Encoder) SetWriteHeaderFunc(f WriteHeaderFunc) {
	v.onWriteHeader = f
}

func (v *Encoder) Reset() error {
	v.fCurCurPacketNum = 0
	v.fCurGranulePos = 0
	return v.fStream.Reset()
}

func (v *Encoder) WriteOggStream(aFlush bool) error {
	var err error
	if aFlush {
		err = v.fStream.PagesFlushToStream(v.fWriter)
	} else {
		err = v.fStream.PagesOutToStream(v.fWriter)
	}
	return err
}

func (v *Encoder) WriteEOS() error {
	og, err := NewEOSPacket(nil, 0, v.fCurCurPacketNum, v.fCurGranulePos)

	if err != nil {
		return err
	}

	return v.fStream.PacketIn(og)
}

func (v *Encoder) WriteHeader(id int) error {

	if v.onWriteHeader != nil {
		seq, err := v.onWriteHeader(id)
		if err != nil {
			return err
		}

		if len(seq) > 0 {
			aHeader := Ptr(seq[0:])
			var header IOGGPacket
			if id == 0 {
				header, err = NewBOSPacket(aHeader,
					int64(len(seq)),
					v.fCurCurPacketNum, v.fCurGranulePos)
			} else {
				header, err = NewSizedPacket(aHeader,
					int64(len(seq)),
					v.fCurCurPacketNum, v.fCurGranulePos)
			}
			if err != nil {
				return err
			}

			v.fCurCurPacketNum++

			err = v.fStream.PacketIn(header)
			runtime.KeepAlive(seq)
			return err
		}
	}

	return nil
}

func (v *Encoder) GranuleFromBytes(pos int64) int64 {
	if v.onGranuleFromBytes != nil {
		return v.onGranuleFromBytes(pos)
	}
	return pos
}

func (v *Encoder) Write(buffer []byte) (int, error) {
	sz := int64(len(buffer))
	loc := 0
	for sz > 0 {
		ChunkSize := v.fChunkSize
		if int64(ChunkSize) > sz {
			ChunkSize = int(sz)
			v.fCurGranulePos += v.GranuleFromBytes(sz)
		} else {
			v.fCurGranulePos += v.GranuleFromBytes(int64(ChunkSize))
		}
		buf := Ptr(buffer[loc:])
		var og IOGGPacket
		var err error
		if v.fCurCurPacketNum == 0 {
			og, err = NewBOSPacket(buf, int64(ChunkSize), v.fCurCurPacketNum, v.fCurGranulePos)
		} else {
			og, err = NewSizedPacket(buf, int64(ChunkSize), v.fCurCurPacketNum, v.fCurGranulePos)
		}
		if err != nil {
			return loc, err
		}

		v.fCurCurPacketNum++

		err = v.fStream.PacketIn(og)
		if err != nil {
			return loc, err
		}

		sz -= int64(ChunkSize)
		loc += ChunkSize
	}
	err := v.WriteOggStream(true)

	return loc, err
}

func (v *Encoder) Done() {
	if v.fStream != nil {
		v.fStream.Done()
		v.fStream = nil
	}
}

func (v *Encoder) Close() error {
	if v.fStream != nil {
		err := v.WriteEOS()
		if err != nil {
			return err
		}
		err = v.Flush()
		if err != nil {
			return err
		}
		v.Done()
	}
	return nil
}

func (v *Encoder) Flush() error {
	return v.WriteOggStream(true)
}

type ReadHeaderFunc = func(id int, data []byte) (bool, error)

type IDecoder interface {
	ResetToStart() error
	ReadHeaders(maxheaders int) error
	GranuleSeek(pos int64) error
	ByteSeek(pos int64) error
	GranuleFromBytes(pos int64) int64
	BytesFromGranule(pos int64) int64
}

type linkedPage struct {
	fOffset   int64
	fSize     int
	fSerialNo int32
	fGranule  int64
}

type Decoder struct {
	fSync        IOGGSyncState
	fStream      IOGGStreamState
	fCurPacket   IOGGPacket
	fCurPage     IOGGPage
	fLinked      []linkedPage
	fLastGranule int64
	fStartPos    int64
	fCurIntPos   int64
	fCurLink     int
	fBufferSize  int64
	fTotalSize   int64

	onBytesFromGranule ByteGranuleConvertFunc
	onGranuleFromBytes ByteGranuleConvertFunc
	onReadHeader       ReadHeaderFunc

	fReader io.Reader
}

func NewDecoder(aBufferSize int64, aReader io.Reader) (*Decoder, error) {
	st, err := NewSyncState()

	if err != nil {
		return nil, err
	}

	return &Decoder{
		fSync:       st,
		fStream:     nil,
		fReader:     aReader,
		fBufferSize: aBufferSize,
		fCurLink:    -1,
		fStartPos:   -1,
		onReadHeader: func(id int, data []byte) (bool, error) {
			return true, nil
		},
		fLinked: make([]linkedPage, 0, 8),
	}, nil
}

func (v *Decoder) SetOnReadHeader(f ReadHeaderFunc) {
	v.onReadHeader = f
}

func (v *Decoder) SetGranuleFromBytesConvertFunc(f ByteGranuleConvertFunc) {
	v.onGranuleFromBytes = f
}

func (v *Decoder) SetBytesFromGranuleConvertFunc(f ByteGranuleConvertFunc) {
	v.onBytesFromGranule = f
}

func (v *Decoder) OggPosition() int64 {
	s, ok := v.fReader.(io.Seeker)
	if ok && (v.fSync != nil) {
		pos, err := s.Seek(0, io.SeekCurrent)
		if err == nil {
			return int64(pos) - int64(v.fSync.Ref().fill) + int64(v.fSync.Ref().returned)
		}
	}
	return -1
}

func (v *Decoder) findLinkBisect(sample int64) int {
	if len(v.fLinked) > 0 {
		ma_p := len(v.fLinked) - 1
		ma_s := v.fLinked[ma_p].fGranule
		if sample >= ma_s {
			return -1
		}

		mi_p := 0
		mi_s := v.fLinked[mi_p].fGranule
		if sample < mi_s {
			return -1
		}

		for (ma_p - mi_p) > 1 {
			p := (mi_p + ma_p) / 2
			s := v.fLinked[p].fGranule
			if s == sample {
				return p + 1
			} else {
				if s > sample {
					ma_s = s
					ma_p = p
				} else {
					mi_s = s
					mi_p = p
				}
			}
		}
		return ma_p
	}
	return -1
}

func (v *Decoder) ReadHeaders(maxheaders int) error {
	const OGG_HEADER_CHUNK = 128
	headernum := 0
	Result := true
	Opage, err := NewPage()
	if err != nil {
		return err
	}

	for Result && (headernum < maxheaders) {
		buffer := (*(*[0x7fffffff]byte)(v.fSync.Buffer(OGG_HEADER_CHUNK)))[0:OGG_HEADER_CHUNK]
		sz, err := v.fReader.Read(buffer)
		if err != nil && err != io.EOF {
			return err
		}
		if sz > 0 {
			v.fSync.Wrote(int64(sz))
			if v.fSync.PageOut(Opage) == 1 {
				if (v.fSync.Check() == 0) && (Opage != nil) &&
					((Opage.Ref().header_len > 0) || (Opage.Ref().body_len > 0)) &&
					(Opage.Packets() > 0) {
					if v.fStream == nil {
						v.fStream, err = NewStream(Opage.SerialNo())
						if err != nil {
							return err
						}
					}
					Opack, err := NewPacket()
					if err != nil {
						return err
					}
					if v.fStream.PageInIgnoreErrors(Opage) == 0 {
						for loop := true; loop; {
							ok, err := v.fStream.PacketOut(Opack)
							header_size := Opack.GetSize()
							if ok && (header_size > 1) {
								if v.onReadHeader != nil {
									header_buffer := (*(*[0x7fffffff]byte)(Opack.GetData()))[0:header_size]

									loop, err = v.onReadHeader(headernum, header_buffer)
									if err != nil {
										return err
									}
									headernum++
									if headernum >= maxheaders {
										loop = false
									}
								}
							} else {
								if err != nil {
									return err
								}
								loop = false
							}
						}
					} else {
						Result = false
					}
				} else {
					Result = false
				}
			}
		} else {
			Result = false
		}
	}
	if Result {
		s, ok := v.fReader.(io.Seeker)
		if ok {
			v.fStartPos = v.OggPosition()
			s.Seek(v.fStartPos, io.SeekStart)
			v.fSync.Reset()
			Result = v.parseAllStream(s)
		}
	}
	return nil
}

func (v *Decoder) oggStreamSeekToPos(bytepos int64) error {
	var err error
	v.fCurPacket, err = NewPacket()
	if err != nil {
		return err
	}

	var p int64 = 0
	for true {
		ret := v.fStream.PacketOutIgnoreErrors(v.fCurPacket)

		if ret == 1 {
			if (v.fCurPacket.GetSize() + p) > bytepos {
				v.fCurIntPos = bytepos - p
				break
			}
			p += v.fCurPacket.GetSize()
		} else if ret == 0 {
			v.fCurPacket = nil
			v.fCurIntPos = 0
			break
		}
	}
	return nil
}

func (v *Decoder) curLink() *linkedPage {
	if v.fCurLink < 0 || v.fCurLink >= len(v.fLinked) {
		return nil
	}
	return &v.fLinked[v.fCurLink]
}

func (v *Decoder) oggPreparePage(linkno int) bool {
	s, ok := v.fReader.(io.Seeker)
	if !ok {
		return false
	}

	v.fCurLink = linkno
	link := v.curLink()
	v.fCurPacket = nil

	if link != nil {
		Result := true
		if linkno > 0 {
			v.fLastGranule = v.fLinked[linkno-1].fGranule
		} else {
			v.fLastGranule = 0
		}

		v.fSync.Reset()
		v.fStream.ResetSerialNo(link.fSerialNo)
		s.Seek(link.fOffset, io.SeekStart)
		v.fCurIntPos = 0

		var err error
		v.fCurPage, err = NewPage()
		if err != nil {
			return false
		}

		for true {
			ret := v.oggGetData(v.fBufferSize)
			if ret <= 0 {
				return false
			}

			ret = v.fSync.PageOut(v.fCurPage)

			if ret > 0 {
				if (v.fSync.Check() == 0) && (v.fCurPage != nil) &&
					((v.fCurPage.Ref().header_len > 0) || (v.fCurPage.Ref().body_len > 0)) &&
					(v.fCurPage.Packets() > 0) {
					Result = (v.fStream.PageInIgnoreErrors(v.fCurPage) == 0)
				} else {
					Result = false
				}
				break
			}
		}
		return Result
	} else {
		v.fCurPage = nil
		return false
	}
}

func (v *Decoder) oggGetData(_nbytes int64) int {
	if _nbytes <= 0 {
		return -1
	}
	buffer := (*(*[0x7fffffff]byte)(v.fSync.Buffer(_nbytes)))[0:_nbytes]
	if buffer == nil {
		return -1
	}
	n, _ := v.fReader.Read(buffer)
	if n > 0 {
		v.fSync.Wrote(int64(n))
	}
	return n
}

func (v *Decoder) oggGetNextPage(og IOGGPage, offset *int64) int64 {
	for true {
		more := v.fSync.PageSeek(og)
		// Skipped (-more) bytes.
		if more < 0 {
			*offset -= more
		} else if more == 0 {
			read_nbytes := v.fBufferSize
			ret := v.oggGetData(read_nbytes)
			if ret < 0 {
				return -1
			}
			if ret == 0 {
				/*Only fail cleanly on EOF if we didn't have a known boundary.
				  Otherwise, we should have been able to reach that boundary, and this
				   is a fatal error.*/
				return -1
			}
		} else {
			/*Got a page.
			  Return the page start offset and advance the internal offset past the
			   page end.*/
			*offset += more
			return *offset
		}
	}
	return 0
}

func (v *Decoder) parseAllStream(s io.Seeker) bool {
	v.fLinked = make([]linkedPage, 0, 8)

	p, err := NewPage()
	if err != nil {
		return false
	}

	v.fTotalSize = 0

	offset, _ := s.Seek(0, io.SeekCurrent)
	total, _ := s.Seek(0, io.SeekEnd)
	s.Seek(offset, io.SeekStart)

	for offset < total {
		fromoff := offset
		ret := v.oggGetNextPage(p, &offset)
		if ret < 0 {
			return false
		} else if ret == 0 {
			break
		}

		serialno := p.SerialNo()
		//Save the information for this page

		link := linkedPage{}

		link.fOffset = fromoff
		link.fSerialNo = serialno
		link.fSize = int(offset - fromoff)
		link.fGranule = p.GranulePos()
		v.fTotalSize += int64(link.fSize)
		v.fLinked = append(v.fLinked, link)
	}
	if len(v.fLinked) > 0 {
		return v.oggPreparePage(0)
	} else {
		return false
	}
}

func (v *Decoder) Read(buf []byte) (int, error) {
	total := len(buf)
	p := 0
	var sz int64
	for p < total {
		cl := v.curLink()
		if cl != nil {
			if v.fCurPacket != nil {
				sz = v.fCurPacket.GetSize() - int64(v.fCurIntPos)
			} else {
				var err error
				v.fCurPacket, err = NewPacket()
				if err != nil {
					return p, err
				}

				ret := -1
				for ret < 0 {
					ret = v.fStream.PacketOutIgnoreErrors(v.fCurPacket)

					if ret == 1 {
						v.fCurIntPos = 0
						sz = v.fCurPacket.GetSize()
					} else {
						sz = 0
					}
				}
			}

			if sz == 0 {
				if !v.oggPreparePage(v.fCurLink + 1) {
					break
				}
			} else {
				if sz > int64(total-p) {
					sz = int64(total - p)
				}

				buffer := (*(*[0x7fffffff]byte)(v.fCurPacket.GetData()))[v.fCurIntPos:]

				copy(buf[p:(p+int(sz))], buffer)

				p += int(sz)
				v.fCurIntPos += sz

				if v.fCurPacket.GetSize() == v.fCurIntPos {
					v.fCurPacket = nil
				}
			}
		} else {
			break
		}
	}
	if p == 0 {
		return 0, io.EOF
	}
	return p, nil
}

func (v *Decoder) ResetToStart() error {
	s, ok := v.fReader.(io.Seeker)

	if ok && (v.fStartPos >= 0) {
		s.Seek(v.fStartPos, io.SeekStart)
	}
	v.fSync.Reset()
	return nil
}

func (v *Decoder) ByteSeek(pos int64) error {
	_, ok := v.fReader.(io.Seeker)

	if ok {
		return v.GranuleSeek(v.GranuleFromBytes(pos))
	} else {
		return nil
	}
}

func (v *Decoder) GranuleSeek(pos int64) error {
	_, ok := v.fReader.(io.Seeker)

	if ok {
		linkno := v.findLinkBisect(pos)
		if (linkno >= 0) && v.oggPreparePage(linkno) {
			return v.oggStreamSeekToPos(v.BytesFromGranule(pos - v.fLastGranule))
		}
		return nil
	} else {
		return nil
	}
}

func (v *Decoder) GranuleFromBytes(pos int64) int64 {
	if v.onGranuleFromBytes != nil {
		return v.onGranuleFromBytes(pos)
	}
	return pos
}

func (v *Decoder) BytesFromGranule(pos int64) int64 {
	if v.onBytesFromGranule != nil {
		return v.onBytesFromGranule(pos)
	}
	return pos
}

func (v *Decoder) Close() error {
	if v.fStream != nil {
		v.fStream.Done()
	}
	if v.fSync != nil {
		v.fSync.Done()
	}

	v.fSync = nil
	v.fStream = nil
	v.fCurPage = nil
	v.fCurPacket = nil

	return nil
}
